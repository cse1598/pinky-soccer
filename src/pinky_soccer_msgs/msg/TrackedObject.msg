# One tracked object in the map frame (self, ball or opponent).
uint8 OBSERVED=0   # measured in the latest perception cycle
uint8 ESTIMATED=1  # predicted from earlier measurements, still trusted
uint8 LOST=2       # too old or too uncertain; pose is the last estimate

string id
geometry_msgs/Pose2D pose          # map frame [m, m, rad]
geometry_msgs/Vector3 velocity     # map frame [m/s], only x and y are used
float64[9] covariance              # row-major 3x3 over (x, y, theta)
uint8 state
builtin_interfaces/Time last_seen  # stamp of the last real observation
float32 confidence                 # 0..1
