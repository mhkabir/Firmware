uint64 timestamp		# time since system start (microseconds)

bool manual_passthrough			# passthrough normalized speed and steering commands

float32 steering_angle          # desired virtual angle, NaN to ignore (radians)
float32 steering_angle_velocity # desired rate of change, NaN to ignore (radians/s)

float32 speed                   # desired forward speed, NaN to ignore (m/s)
float32 acceleration            # desired acceleration, NaN to ignore (m/s^2)
float32 jerk                    # desired jerk, NaN to ignore (m/s^3)