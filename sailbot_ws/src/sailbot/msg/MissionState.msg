builtin_interfaces/Time stamp

# Mission modes
uint8 MODE_INIT          = 0
uint8 MODE_WAIT_GNSS     = 1
uint8 MODE_SAIL_TO_WP    = 2
uint8 MODE_STOPPED       = 3

uint8 mode

bool gnss_ready
bool waypoint_reached

float32 distance_to_wp_m
