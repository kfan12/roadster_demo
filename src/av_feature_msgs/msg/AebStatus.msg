std_msgs/Header header

uint8 INACTIVE = 0
uint8 MONITORING = 1
uint8 WARNING=2
uint8 BRAKING=3
uint8 STOP_HOLD=4
uint8 FAULT=5
uint8 state

bool emergency_active

float64 obstacle_distance_m
float64 relative_speed_mps
float64 time_to_collision_sec
float64 safe_braking_distance_m
float64 requested_deceleration_mps2

string obstacle_id
string reason  # for logs and RViz overlays only 
