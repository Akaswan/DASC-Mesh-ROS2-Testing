#!/usr/bin/env python3
"""Run mesh_ping tests and plot the results. See docs/TESTING.md for the test procedures.

Each run is stored as results/<campaign>/<run_id>/ with:
  raw.csv    one row per message from ping_node (seq, send_ns, rtt_ms, status)
  meta.json  what was tested (distance, tags, parameters, signal, marks) and computed stats
  ping.log   ping_node's console output

  run       run ping_node once per payload x rate combination (x --repeat)
  list      print a table of the runs in one or more campaigns
  plot      write summary.csv and a PNG of chosen metrics against distance, a tag, or load
  timeline  plot one run over time (walk, failover and soak tests)

Only `plot` and `timeline` need matplotlib, so `run` works on a bare Pi.
"""

import argparse
import csv
import datetime
import itertools
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

RESULTS = Path(__file__).resolve().parent.parent / 'results'
DOCKER_PING = '/ws/install/mesh_ping/lib/mesh_ping/ping_node'


def csv_list(cast):
    return lambda s: [cast(v) for v in s.split(',')]


def parse_tag(s):
    key, sep, value = s.partition('=')
    if not sep:
        raise argparse.ArgumentTypeError(f'expected key=value, got {s!r}')
    try:
        value = float(value)
    except ValueError:
        pass
    return key, value


# ---------------------------------------------------------------- stats

def percentile(sorted_vals, q):
    idx = int(q * (len(sorted_vals) - 1) + 0.5)
    return sorted_vals[min(idx, len(sorted_vals) - 1)]


def read_raw(raw_csv):
    with open(raw_csv) as f:
        return [{'seq': int(r['seq']), 'send_ns': int(r['send_ns']),
                 'rtt_ms': float(r['rtt_ms']) if r['rtt_ms'] else None, 'status': r['status']}
                for r in csv.DictReader(f)]


def compute_stats(rows, sent, payload, rate):
    rtts = [r['rtt_ms'] for r in rows if r['status'] in ('ok', 'reordered')]
    lost_seqs = sorted(r['seq'] for r in rows if r['status'] == 'lost')
    count = {s: sum(r['status'] == s for r in rows) for s in ('ok', 'reordered', 'lost', 'late')}

    received = count['ok'] + count['reordered']
    resolved = received + count['lost']
    send_times = [r['send_ns'] for r in rows if r['status'] != 'late']
    span_s = (max(send_times) - min(send_times)) / 1e9 + 1.0 / rate if send_times else None
    stats = {
        'sent': sent,
        'received': received,
        'lost': count['lost'],
        'late': count['late'],
        'reordered': count['reordered'],
        'unresolved': max(sent - resolved, 0),
        'loss_pct': 100.0 * count['lost'] / resolved if resolved else None,
        'max_loss_burst': 0,
        'max_gap_s': None,
        # One direction only; the echo doubles what crosses the mesh unless echo_payload:=false.
        'offered_kbps': rate * payload * 8 / 1000,
        'goodput_kbps': received * payload * 8 / 1000 / span_s if span_s else None,
    }

    # Longest run of consecutive lost sequence numbers: one long dropout and many
    # scattered drops can have the same loss % but feel very different to a robot.
    burst = 0
    for i, seq in enumerate(lost_seqs):
        burst = burst + 1 if i and seq == lost_seqs[i - 1] + 1 else 1
        stats['max_loss_burst'] = max(stats['max_loss_burst'], burst)

    # Longest time between two consecutive *delivered* messages (by send time):
    # the outage length a subscriber would actually see, e.g. during a reroute.
    delivered = sorted(r['send_ns'] for r in rows if r['status'] in ('ok', 'reordered'))
    if len(delivered) > 1:
        stats['max_gap_s'] = max(b - a for a, b in zip(delivered, delivered[1:])) / 1e9

    if rtts:
        s = sorted(rtts)
        stats.update({
            'rtt_min_ms': s[0],
            'rtt_mean_ms': sum(s) / len(s),
            'rtt_p50_ms': percentile(s, 0.50),
            'rtt_p95_ms': percentile(s, 0.95),
            'rtt_p99_ms': percentile(s, 0.99),
            'rtt_max_ms': s[-1],
            # Rows are written in arrival order, matching ping_node's own jitter.
            'jitter_ms': (sum(abs(b - a) for a, b in zip(rtts, rtts[1:])) / (len(rtts) - 1)
                          if len(rtts) > 1 else 0.0),
        })
    return stats


# ---------------------------------------------------------------- run

def capture_signal(cmd):
    """Run a user-supplied command (e.g. `iw ... station get` over ssh) and pull out the dBm value."""
    if not cmd:
        return None
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        return {'raw': '', 'error': 'timeout'}
    dbm = [int(v) for v in re.findall(r'signal(?: avg)?:\s*\[?(-?\d+)', out)]
    return {'raw': out, 'dbm': dbm[0] if dbm else None}


def ping_command(args, out_dir, payload, rate, count):
    params = {
        'rate_hz': float(rate),
        'payload_size': payload,
        'count': count,
        'timeout_s': args.timeout,
        'report_period_s': 1.0,
        'reliable': 'true' if args.qos == 'reliable' else 'false',
        'depth': args.depth,
    }
    if args.docker:
        csv_path = '/out/raw.csv'
        cmd = ['docker', 'run', '--rm', '--network', 'host', '--ipc', 'host',
               '-v', f'{out_dir}:/out']
        if 'ROS_DOMAIN_ID' in os.environ:
            cmd += ['-e', f'ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}']
        # Call the binary directly so SIGINT reaches ping_node as PID 1 (ros2 run wouldn't forward it).
        cmd += [args.docker, DOCKER_PING]
    else:
        csv_path = str(out_dir / 'raw.csv')
        cmd = ['ros2', 'run', 'mesh_ping', 'ping_node']
    cmd += ['--ros-args']
    for k, v in {**params, 'csv_path': csv_path}.items():
        cmd += ['-p', f'{k}:={v}']
    return cmd, params


def read_marks(marks, stop):
    """Each Enter during a run stores a mark; any text typed first becomes its label."""
    while not stop.is_set():
        try:
            label = sys.stdin.readline()
        except (OSError, ValueError):
            return
        if not label or stop.is_set():
            return
        t = time.monotonic_ns()
        label = label.strip() or str(len(marks) + 1)
        marks.append({'t_ns': t, 'label': label})
        print(f'--- mark "{label}"', flush=True)


def run_one(args, payload, rate):
    # Duration 0 means "until Ctrl-C" (count=0 in ping_node).
    count = int(round(rate * args.duration))
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    where = f'{args.distance:g}m' if args.distance is not None else 'na'
    run_id = f'{where}_{payload}B_{rate:g}Hz_{args.qos}_{stamp}'
    out_dir = RESULTS / args.campaign / run_id
    out_dir.mkdir(parents=True)

    cmd, params = ping_command(args, out_dir, payload, rate, count)
    length = f'{count} msgs over ~{args.duration:g}s' if count else 'until Ctrl-C'
    print(f'\n>>> {run_id}: {length}', flush=True)
    if args.marks:
        print('>>> type a label (optional) and press Enter to mark a moment', flush=True)

    meta = {
        'campaign': args.campaign,
        'run_id': run_id,
        'distance_m': args.distance,
        'tags': dict(args.tag),
        'note': args.note,
        'qos': args.qos,
        'params': params,
        'host': socket.gethostname(),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'started': datetime.datetime.now().isoformat(timespec='seconds'),
        'signal_before': capture_signal(args.signal_cmd),
    }

    marks, stop_marks = [], threading.Event()
    if args.marks:
        threading.Thread(target=read_marks, args=(marks, stop_marks), daemon=True).start()

    # Discovery + the final timeout window come on top of the send time.
    deadline = args.duration + args.timeout + args.discovery_timeout if count else None
    with open(out_dir / 'ping.log', 'w') as log:
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, start_new_session=True)

        def tee():
            for line in proc.stdout:
                if not args.quiet or 'matched' in line or 'summary' in line or 'sent=' in line:
                    sys.stdout.write(line)
                log.write(line)
        reader = threading.Thread(target=tee)
        reader.start()
        interrupted = False
        try:
            proc.wait(timeout=deadline)
            meta['completed'] = True
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as e:
            interrupted = isinstance(e, KeyboardInterrupt)
            # An open-ended run (--duration 0) is meant to end with Ctrl-C.
            meta['completed'] = interrupted and not count
            print(f'!!! stopping run ({type(e).__name__})', flush=True)
            # SIGINT makes ping_node print its summary and flush raw.csv.
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait()
        reader.join()
        stop_marks.set()

    meta['finished'] = datetime.datetime.now().isoformat(timespec='seconds')
    meta['signal_after'] = capture_signal(args.signal_cmd)
    meta['marks'] = marks

    log_text = (out_dir / 'ping.log').read_text()
    discovery = re.findall(r'matched after (\d+) ms', log_text)
    raw = out_dir / 'raw.csv'
    if raw.exists():
        sent = re.findall(r'sent=(\d+)', log_text)
        meta['stats'] = compute_stats(read_raw(raw), int(sent[-1]) if sent else count, payload, rate)
        meta['stats']['discovery_ms'] = float(discovery[0]) if discovery else None
    else:
        meta['stats'] = None
        print('!!! no raw.csv written; see ping.log', flush=True)
    (out_dir / 'meta.json').write_text(json.dumps(meta, indent=2))

    if interrupted and count:
        raise KeyboardInterrupt
    return meta


def cmd_run(args):
    if args.distance is None and not args.tag:
        sys.exit('give --distance and/or at least one --tag so the run can be placed on a plot')
    single = len(args.payload) == 1 and len(args.rate) == 1 and args.repeat == 1
    if (args.duration == 0 or args.marks) and not single:
        sys.exit('--duration 0 and --marks run a single configuration (one payload, one rate, no --repeat)')
    if not args.yes:
        where = f'{args.distance:g} m apart' if args.distance is not None else 'in position'
        input(f'Nodes {where}, pong_node running with qos={args.qos}? Enter to start ')
    combos = list(itertools.product(range(args.repeat), args.payload, args.rate))
    for i, (_, payload, rate) in enumerate(combos, 1):
        print(f'\n=== run {i}/{len(combos)}', flush=True)
        meta = run_one(args, payload, rate)
        if meta['stats']:
            print_table([meta])
        if i < len(combos) and args.pause:
            time.sleep(args.pause)


# ---------------------------------------------------------------- list / plot

def load_runs(campaigns):
    runs = []
    for campaign in campaigns:
        for meta_file in sorted((RESULTS / campaign).glob('*/meta.json')):
            meta = json.loads(meta_file.read_text())
            if meta.get('stats') and meta['stats']['sent']:
                runs.append(meta)
    if not runs:
        sys.exit(f'no runs with data under {", ".join(str(RESULTS / c) for c in campaigns)}')
    return sorted(runs, key=lambda m: m['started'])


def signal_dbm(meta):
    return (meta.get('signal_before') or {}).get('dbm')


# x axis: built-in variables, or any --tag name.
X_BUILTIN = {
    'distance': ('Distance (m)', lambda m: m['distance_m']),
    'rate': ('Send rate (Hz)', lambda m: m['params']['rate_hz']),
    'payload': ('Payload (bytes)', lambda m: m['params']['payload_size']),
    'load': ('Offered load, one way (kbit/s)', lambda m: m['stats']['offered_kbps']),
    'signal': ('Signal (dBm)', signal_dbm),
}


def x_label(x):
    return X_BUILTIN[x][0] if x in X_BUILTIN else x


def x_value(meta, x):
    return X_BUILTIN[x][1](meta) if x in X_BUILTIN else meta['tags'].get(x)


# Series dimensions: whichever of these vary across the plotted runs (minus the x variable).
SERIES_DIMS = {
    'campaign': lambda m: m['campaign'],
    'payload': lambda m: f"{m['params']['payload_size']}B",
    'rate': lambda m: f"{m['params']['rate_hz']:g}Hz",
    'qos': lambda m: m['qos'],
}


def series_namer(runs, x):
    dims = [d for d, get in SERIES_DIMS.items()
            if d != x and not (x == 'load' and d in ('payload', 'rate'))
            and len({get(m) for m in runs}) > 1]
    if x == 'load' and len({m['params']['payload_size'] for m in runs}) > 1:
        dims.append('payload')  # load sweeps read best as one line per payload
    return lambda m: ' · '.join(SERIES_DIMS[d](m) for d in dims) or 'all runs'


def fmt(v, spec='.2f'):
    return '-' if v is None else format(v, spec)


def print_table(runs, x='distance'):
    print(f"{x:>9} {'config':<28} {'sent':>6} {'loss%':>7} {'burst':>5} {'gap s':>6} "
          f"{'p50ms':>8} {'p99ms':>8} {'jit ms':>7} {'kbps':>8} {'disc ms':>7} {'dBm':>5}")
    for m in runs:
        s = m['stats']
        p = m['params']
        config = f"{p['payload_size']}B @ {p['rate_hz']:g}Hz {m['qos']}"
        print(f"{fmt(x_value(m, x), 'g') if not isinstance(x_value(m, x), str) else x_value(m, x):>9} "
              f"{config:<28} {s['sent']:>6} {fmt(s['loss_pct']):>7} {s['max_loss_burst']:>5} "
              f"{fmt(s.get('max_gap_s')):>6} {fmt(s.get('rtt_p50_ms')):>8} {fmt(s.get('rtt_p99_ms')):>8} "
              f"{fmt(s.get('jitter_ms')):>7} {fmt(s.get('goodput_kbps'), '.1f'):>8} "
              f"{fmt(s.get('discovery_ms'), '.0f'):>7} {fmt(signal_dbm(m), 'd'):>5}"
              f"{'' if m['completed'] else '  (incomplete)'}")


def cmd_list(args):
    print_table(load_runs(args.campaign), args.x)


STAT_KEYS = ['sent', 'received', 'lost', 'loss_pct', 'max_loss_burst', 'max_gap_s', 'late',
             'reordered', 'rtt_min_ms', 'rtt_mean_ms', 'rtt_p50_ms', 'rtt_p95_ms', 'rtt_p99_ms',
             'rtt_max_ms', 'jitter_ms', 'offered_kbps', 'goodput_kbps', 'discovery_ms']


def write_summary(runs, path):
    tag_keys = sorted({k for m in runs for k in m['tags']})
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['campaign', 'run_id', 'started', 'distance_m', *tag_keys, 'payload_size',
                    'rate_hz', 'qos', 'signal_dbm', 'completed', *STAT_KEYS, 'note'])
        for m in runs:
            w.writerow([m['campaign'], m['run_id'], m['started'], m['distance_m'],
                        *(m['tags'].get(k) for k in tag_keys), m['params']['payload_size'],
                        m['params']['rate_hz'], m['qos'], signal_dbm(m), m['completed'],
                        *(m['stats'].get(k) for k in STAT_KEYS), m['note']])


# name: (panel title, getter on a run's meta)
METRICS = {
    'loss': ('Loss (%)', lambda m: m['stats']['loss_pct']),
    'p50': ('Median RTT (ms)', lambda m: m['stats'].get('rtt_p50_ms')),
    'p99': ('p99 RTT (ms)', lambda m: m['stats'].get('rtt_p99_ms')),
    'jitter': ('Jitter (ms)', lambda m: m['stats'].get('jitter_ms')),
    'burst': ('Longest loss burst (msgs)', lambda m: m['stats']['max_loss_burst']),
    'gap': ('Longest delivery gap (s)', lambda m: m['stats'].get('max_gap_s')),
    'goodput': ('Goodput, one way (kbit/s)', lambda m: m['stats'].get('goodput_kbps')),
    'discovery': ('Discovery time (ms)', lambda m: m['stats'].get('discovery_ms')),
    'signal': ('Signal (dBm)', signal_dbm),
}

# Reference categorical palette (dataviz skill), assigned in fixed order.
SERIES_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100',
                 '#e87ba4', '#008300', '#6250d6', '#e34948']
INK, INK_2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e6e5e0', '#fcfcfb'


def setup_matplotlib():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#b5b4ae', 'axes.labelcolor': INK_2,
                         'xtick.color': INK_2, 'ytick.color': INK_2})
    return plt


def style_axis(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc='left', fontsize=10, color=INK)
    ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax.spines[['top', 'right']].set_visible(False)


def out_dir_for(args):
    out = RESULTS / ('+'.join(args.campaign) if len(args.campaign) > 1 else args.campaign[0])
    out.mkdir(parents=True, exist_ok=True)
    return out


def cmd_plot(args):
    plt = setup_matplotlib()
    bad = [p for p in args.panels if p not in METRICS]
    if bad:
        sys.exit(f'unknown --panels {bad}; choose from {list(METRICS)}')

    runs = [m for m in load_runs(args.campaign) if x_value(m, args.x) is not None]
    if not runs:
        sys.exit(f'no runs have a value for --x {args.x}')
    out_dir = out_dir_for(args)
    write_summary(runs, out_dir / 'summary.csv')

    # String tags (e.g. obstacle=door) become a categorical axis in order of first run.
    categorical = any(isinstance(x_value(m, args.x), str) for m in runs)
    if categorical:
        order = list(dict.fromkeys(str(x_value(m, args.x)) for m in runs))
        position = lambda m: order.index(str(x_value(m, args.x)))  # noqa: E731
    else:
        position = lambda m: x_value(m, args.x)  # noqa: E731

    name_of = series_namer(runs, args.x)
    series = {}
    for m in runs:
        series.setdefault(name_of(m), []).append(m)
    if len(series) > len(SERIES_COLORS):
        sys.exit(f'{len(series)} series is too many for one plot; split the campaign')

    fig, axes = plt.subplots(len(args.panels), 1, figsize=(8, 2.4 * len(args.panels)),
                             sharex=True, squeeze=False)
    axes = axes[:, 0]
    fig.patch.set_facecolor(SURFACE)
    for panel, ax in zip(args.panels, axes):
        title, get = METRICS[panel]
        style_axis(ax, title)
        for color, (name, ms) in zip(SERIES_COLORS, series.items()):
            pts = sorted((position(m), get(m)) for m in ms if get(m) is not None)
            if not pts:
                continue
            # Repeated runs at the same x are averaged into the line and shown as faint dots.
            xs = sorted({p[0] for p in pts})
            means = [sum(y for xx, y in pts if xx == xv) / sum(xx == xv for xx, _ in pts) for xv in xs]
            # Categories have no order, so a connecting line would imply a trend.
            ax.plot(xs, means, color=color, linewidth=2, linestyle='none' if categorical else '-',
                    marker='o', markersize=8 if categorical else 6,
                    markeredgecolor=SURFACE, markeredgewidth=1.5, label=name, zorder=3)
            if len(pts) > len(xs):
                ax.scatter([p[0] for p in pts], [p[1] for p in pts], color=color, s=12,
                           alpha=0.4, zorder=2)
        if panel == 'goodput' and args.logx:
            ax.set_yscale('log')  # goodput tracks offered load, so match the x scale
        elif panel != 'signal':
            ax.set_ylim(bottom=0)
        if panel == 'loss':
            ax.set_ylim(0, max(5, ax.get_ylim()[1]))
    if categorical:
        axes[-1].set_xticks(range(len(order)))
        axes[-1].set_xticklabels(order)
    if args.logx:
        axes[-1].set_xscale('log')
    axes[-1].set_xlabel(x_label(args.x))
    if len(series) > 1:
        axes[0].legend(frameon=False, fontsize=9, loc='upper left')
    fig.suptitle(f'mesh_ping: {" + ".join(args.campaign)}', x=0.01, ha='left', fontsize=12, color=INK)
    fig.tight_layout()

    png = out_dir / f'{out_dir.name}_vs_{args.x}.png'
    fig.savefig(png, dpi=150, facecolor=SURFACE)
    print(f'wrote {out_dir / "summary.csv"}\nwrote {png}')


def cmd_timeline(args):
    plt = setup_matplotlib()
    run_dirs = sorted((RESULTS / args.campaign[0]).glob('*/meta.json'),
                      key=lambda p: json.loads(p.read_text())['started'])
    if args.run:
        run_dirs = [p for p in run_dirs if p.parent.name.startswith(args.run)]
    if not run_dirs:
        sys.exit('no matching run')
    run_dir = run_dirs[-1].parent
    meta = json.loads((run_dir / 'meta.json').read_text())
    rows = read_raw(run_dir / 'raw.csv')
    if not rows:
        sys.exit(f'{run_dir} has no messages')

    t0 = min(r['send_ns'] for r in rows)
    span = (max(r['send_ns'] for r in rows) - t0) / 1e9
    # Bins keep hour-long soak runs readable: ~300 bins, never finer than 1 s.
    bin_s = args.bin or max(1.0, span / 300)
    nbins = int(span // bin_s) + 1
    bins = [{'rtts': [], 'lost': 0, 'got': 0} for _ in range(nbins)]
    for r in rows:
        b = bins[int((r['send_ns'] - t0) / 1e9 // bin_s)]
        if r['status'] in ('ok', 'reordered'):
            b['got'] += 1
            b['rtts'].append(r['rtt_ms'])
        elif r['status'] == 'lost':
            b['lost'] += 1

    xs, loss, p50, p99 = [], [], [], []
    for i, b in enumerate(bins):
        xs.append((i + 0.5) * bin_s)
        n = b['got'] + b['lost']
        loss.append(100.0 * b['lost'] / n if n else None)
        s = sorted(b['rtts'])
        p50.append(percentile(s, 0.5) if s else None)
        p99.append(percentile(s, 0.99) if s else None)

    fig, (ax_loss, ax_rtt) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.patch.set_facecolor(SURFACE)
    style_axis(ax_loss, f'Loss per {bin_s:g} s bin (%)')
    style_axis(ax_rtt, f'RTT per {bin_s:g} s bin (ms)')

    ax_loss.fill_between(xs, [v or 0 for v in loss], step='mid', color=SERIES_COLORS[1], alpha=0.25,
                         linewidth=0)
    ax_loss.step(xs, [v if v is not None else float('nan') for v in loss], where='mid',
                 color=SERIES_COLORS[1], linewidth=2)
    ax_loss.set_ylim(0, 105)

    nan = float('nan')
    ax_rtt.plot(xs, [v if v is not None else nan for v in p50], color=SERIES_COLORS[0], linewidth=2,
                label='median')
    ax_rtt.plot(xs, [v if v is not None else nan for v in p99], color=SERIES_COLORS[0], linewidth=1,
                linestyle='--', label='p99')
    ax_rtt.set_ylim(bottom=0)
    if args.logy:
        ax_rtt.set_yscale('log')
    ax_rtt.legend(frameon=False, fontsize=9, loc='upper left')

    for mark in meta.get('marks', []):
        t = (mark['t_ns'] - t0) / 1e9
        for ax in (ax_loss, ax_rtt):
            ax.axvline(t, color=INK_2, linewidth=1, linestyle=':')
        ax_loss.text(t, 104, f' {mark["label"]}', color=INK, fontsize=9, va='top', ha='left')

    ax_rtt.set_xlabel('Time since first message (s)')
    s = meta['stats']
    fig.suptitle(f'{meta["run_id"]}\nloss {fmt(s["loss_pct"])}% · longest gap {fmt(s.get("max_gap_s"))} s'
                 f' · p50 {fmt(s.get("rtt_p50_ms"))} ms', x=0.01, ha='left', fontsize=11, color=INK)
    fig.tight_layout()
    png = run_dir / 'timeline.png'
    fig.savefig(png, dpi=150, facecolor=SURFACE)
    print(f'wrote {png}')


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('run', help='run ping_node at the current position')
    r.add_argument('--campaign', required=True, help='groups runs into one results folder / plot')
    r.add_argument('--distance', type=float, help='distance between the two nodes in metres')
    r.add_argument('--tag', type=parse_tag, action='append', default=[],
                   help='extra variable, e.g. --tag walls=2 or --tag hops=3 (repeatable)')
    r.add_argument('--note', default='', help='free text, e.g. "line of sight, hallway"')
    r.add_argument('--payload', type=csv_list(int), default=[64], help='bytes, comma list (default 64)')
    r.add_argument('--rate', type=csv_list(float), default=[10.0], help='Hz, comma list (default 10)')
    r.add_argument('--qos', choices=['best_effort', 'reliable'], default='best_effort',
                   help='must match how pong_node was started (default best_effort)')
    r.add_argument('--duration', type=float, default=30.0,
                   help='seconds of sending per run (default 30); 0 = until Ctrl-C')
    r.add_argument('--repeat', type=int, default=1, help='run every combination this many times')
    r.add_argument('--pause', type=float, default=0.0, help='seconds to wait between runs')
    r.add_argument('--timeout', type=float, default=2.0, help='seconds before a message counts lost')
    r.add_argument('--depth', type=int, default=10)
    r.add_argument('--discovery-timeout', type=float, default=30.0,
                   help='seconds to wait for pong_node before giving up')
    r.add_argument('--docker', metavar='IMAGE', help='run ping_node in this image instead of ros2 run')
    r.add_argument('--signal-cmd', help='shell command whose output contains "signal: -NN", '
                   'e.g. "ssh root@router-a iw dev mesh0 station get <mac>"; saved with each run')
    r.add_argument('--marks', action='store_true',
                   help='press Enter during the run to timestamp moments (shown on the timeline)')
    r.add_argument('-q', '--quiet', action='store_true', help="hide ping_node's per-second lines")
    r.add_argument('-y', '--yes', action='store_true', help="don't prompt before starting")
    r.set_defaults(func=cmd_run)

    for name, func, help_ in [('list', cmd_list, 'print a table of runs'),
                              ('plot', cmd_plot, 'write summary.csv and a PNG')]:
        p = sub.add_parser(name, help=help_)
        p.add_argument('--campaign', type=csv_list(str), required=True,
                       help='one campaign, or a comma list to compare several')
        p.add_argument('--x', default='distance',
                       help=f'x axis: {", ".join(X_BUILTIN)}, or a --tag name (default distance)')
        p.set_defaults(func=func)
        if name == 'plot':
            p.add_argument('--panels', type=csv_list(str), default=['loss', 'p50', 'p99', 'jitter'],
                           help=f'comma list from {", ".join(METRICS)} (default loss,p50,p99,jitter)')
            p.add_argument('--logx', action='store_true', help='log-scale x axis (load sweeps)')

    t = sub.add_parser('timeline', help='plot one run over time')
    t.add_argument('--campaign', type=csv_list(str), required=True)
    t.add_argument('--run', help='run_id prefix (default: the latest run in the campaign)')
    t.add_argument('--bin', type=float, help='seconds per bin (default: auto)')
    t.add_argument('--logy', action='store_true', help='log-scale RTT axis')
    t.set_defaults(func=cmd_timeline)

    args = ap.parse_args()
    try:
        args.func(args)
    except KeyboardInterrupt:
        sys.exit('\ninterrupted')


if __name__ == '__main__':
    main()
