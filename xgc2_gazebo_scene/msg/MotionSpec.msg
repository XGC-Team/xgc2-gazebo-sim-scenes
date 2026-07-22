string MODE_HOLD=hold
string MODE_CONSTANT_TWIST=constant_twist
string MODE_PING_PONG=ping_pong

# Logical obstacle name without the xgc2_obstacle_ prefix.
string name
string mode

# Used by constant_twist. Linear and angular components are expressed in the
# world frame and start from the observed pose at configuration time.
geometry_msgs/Twist twist

# Used by ping_pong. Exactly two world-frame points are required.
geometry_msgs/Point[] waypoints
float64 speed
