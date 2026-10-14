# Sequence number assigned by ping_node, starting at 0.
uint64 seq

# ping_node's steady-clock time in nanoseconds at send. pong_node echoes it
# back unchanged, so RTT is computed entirely on the ping machine and the two
# clocks never need to be synchronized.
int64 send_time_ns

# Filler bytes used to control the message size on the wire.
uint8[] payload
