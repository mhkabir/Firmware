uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_utc # UTC timestamp

uint8 camera_id # ID of the camera this event is from
uint32 seq		# Image sequence number
bool feedback	# Trigger feedback from camera

# TOPICS camera_trigger camera_trigger_secondary