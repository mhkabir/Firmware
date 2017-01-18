# GPS position in WGS84 coordinates.
# the auto-generated field 'timestamp' is for the position & velocity (microseconds)

uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

int32 lat			# Latitude in 1E-7 degrees
int32 lon			# Longitude in 1E-7 degrees
int32 alt			# Altitude in 1E-3 meters above MSL, (millimetres)
int32 alt_ellipsoid 		# Altitude in 1E-3 meters bove Ellipsoid, (millimetres)

float32 vel_n		# GPS North velocity, (metres/sec)
float32 vel_e		# GPS East velocity, (metres/sec)
float32 vel_d		# GPS Down velocity, (metres/sec)
bool vel_ned_valid		# True if NED velocity is valid

float32 cog 		# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)

float32 pos_acc_n	# GPS North position standard deviation (metres)
float32 pos_acc_e	# GPS East position standard deviation (metres)
float32 pos_acc_d	# GPS Down position standard deviation (metres)

float32 vel_acc_n	# GPS North velocity standard deviation (metres/sec)
float32 vel_acc_e	# GPS East velocity standard deviation (metres/sec)
float32 vel_acc_d	# GPS Down velocity standard deviation (metres/sec)

float32 cog_acc		# GPS course over ground standard deviation, (radians)

float32 hdop			# Horizontal dilution of precision
float32 vdop			# Vertical dilution of precision

int32 noise_per_ms		# GPS noise per millisecond
int32 jamming_indicator		# indicates jamming is occurring

int32 timestamp_time_relative	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

uint8 satellites_used		# Number of satellites used
