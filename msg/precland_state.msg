# Main state, i.e. what user wants. Controlled by RC or from ground station via telemetry link.
uint64 timestamp			# time since system start (microseconds)

uint8 START = 0
uint8 HORIZONTAL_APPROACH = 1
uint8 DESCEND_ABOVE_TARGET = 2
uint8 FINAL_APPROACH = 3
uint8 SEARCH = 4
uint8 FALLBACK = 5
uint8 FINISHED = 6
uint8 FAILED = 7

uint8 state
