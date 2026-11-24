# Result for the exact GoToPose request; no independent BT tolerance check.
uint8 RUNNING=0
uint8 SUCCESS=1
uint8 FAILURE=2
std_msgs/Header header
uint64 goal_id
uint8 state
string reason
float64 position_error
float64 heading_error
