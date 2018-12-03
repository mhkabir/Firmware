# Position of landing target in body frame from a single detection

uint64 timestamp		# time since system start (microseconds)

float32 x			# X/forward position of target, relative to vehicle (body frame) [meters]
float32 y			# Y/right position of target, relative to vehicle (body frame) [meters]
float32 z			# Z/down position of target, relative to vehicle (body frame) [meters]

float32 var_x		# X/forward position variance [meters^2]
float32 var_y		# Y/right position variance [meters^2]
float32 var_z		# Z/down position variance [meters^2]