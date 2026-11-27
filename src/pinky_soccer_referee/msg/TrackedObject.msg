# Metric ground-plane estimate; temporary track ID, NOT team identity.
std_msgs/Header header
uint32 id
bool visible
bool velocity_valid
bool heading_valid
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 heading
float64 angular_velocity
# Team is observed from the rear LED; white means unknown, not a team.
string team
bool team_valid
bool angular_velocity_valid
# Heuristic quality indicators, not calibrated probabilities.
float64 heading_confidence
float64 tracking_confidence
bool reacquired
