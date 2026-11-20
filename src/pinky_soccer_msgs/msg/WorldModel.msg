# Fused world state for decision making. Frame: map (field centre origin, +X blue goal).
std_msgs/Header header
TrackedObject self
TrackedObject ball
TrackedObject[] opponents
geometry_msgs/Point own_goal       # mouth centre of the goal we defend
geometry_msgs/Point opp_goal       # mouth centre of the goal we attack
string our_goal_color              # yellow | blue
bool playing                       # latest /game/playing
nav_msgs/OccupancyGrid ball_search_grid  # ball presence probability 0..100
