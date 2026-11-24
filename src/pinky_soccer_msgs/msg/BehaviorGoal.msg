# Target chosen by the behaviour tree; the planning/control layer tracks it.
uint8 NAVIGATE=0   # far target, Nav2 later
uint8 REACTIVE=1   # near the ball, direct control
uint8 STOP=2

std_msgs/Header header             # frame: map
string branch                      # ATTACK | DEFEND | SEARCH | STOP
string action                      # running leaf node name
uint8 mode
geometry_msgs/Pose2D target        # map frame
float32 max_speed                  # [m/s], 0 = controller default
nav_msgs/Path planned_path

# Nonzero for a specific GoToPose invocation; zero for streaming goals.
uint64 goal_id
