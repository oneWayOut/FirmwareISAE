# Global position setpoint triplet in WGS84 coordinates.
# This are the three next waypoints (or just the next two or one).

uint64 timestamp		# time since system start (microseconds)

px4/position_setpoint previous
px4/position_setpoint current
px4/position_setpoint next

uint8 cmdstage  # 0=none; 1=begin scout; 2= stop scout; 3=close to target;
