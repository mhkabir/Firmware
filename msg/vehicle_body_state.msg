# Vehicle body-frame state data.
uint64 timestamp		# time since system start (microseconds)

# Angular rate in body-fixed frame (rad/s). NaN if invalid/unknown
float32 rollspeed		# Angular velocity about X body axis
float32 pitchspeed		# Angular velocity about Y body axis
float32 yawspeed		# Angular velocity about Z body axis

# Velocity in meters/second. NaN if invalid/unknown
float32 vx			# Velocity along X body axis
float32 vy			# Velocity along Y body axis
float32 vz			# Velocity along Z body axis

# Acceleration in meters/second^2. NaN if invalid/unknown
float32 ax			# Acceleration along X body axis
float32 ay			# Acceleration along Y body axis
float32 az			# Acceleration along Z body axis