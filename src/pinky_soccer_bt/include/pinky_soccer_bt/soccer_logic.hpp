// Pure geometry used by the behaviour tree nodes. No ROS types, so it is unit tested
// directly. All coordinates are in the map frame (field centre origin, +X blue goal).
#pragma once

#include <array>
#include <vector>

namespace pinky_soccer_bt
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

double wrapAngle(double a);
double distance(const Vec2& a, const Vec2& b);
double bearing(const Vec2& from, const Vec2& to);
Vec2 position(const Pose2& p);

// Largest standard deviation of the (x, y) block of a row-major 3x3 covariance.
double maxPositionSigma(const std::array<double, 9>& cov);

// Ball is "known" when it is not LOST and its worst-axis sigma is small enough.
bool ballKnown(bool lost, const std::array<double, 9>& cov, double max_sigma);

// Ball on our side of the centre line, extended by margin towards the opponent half.
bool ballInOwnHalf(const Vec2& ball, const Vec2& own_goal, double margin);

// Any opponent strictly closer to the ball than we are.
bool opponentCloserToBall(const Vec2& ball, const Pose2& self, const std::vector<Vec2>& opponents);

// Pose on the (opp_goal -> ball) line, offset behind the ball, facing the goal.
Pose2 behindBallPose(const Vec2& ball, const Vec2& opp_goal, double offset);

// Robot is behind the ball on the shot line: behind it (but not farther than max_behind),
// within tol_dist of the line, and heading within tol_angle of the shot direction.
bool isBehindBall(const Pose2& self, const Vec2& ball, const Vec2& opp_goal, double tol_dist,
                  double tol_angle, double max_behind);

// Heading differs from the bearing to our own goal by at least min_angle.
bool notFacingOwnGoal(const Pose2& self, const Vec2& own_goal, double min_angle);

// Approach target for GoBehindBall. When the robot is not yet behind the ball, a side
// waypoint (clearance from the ball, on the robot's side of the shot line) is returned
// first so the robot does not drive through the ball towards its own goal.
// Simplified detour: no collision check on the way to the waypoint (TODO).
Pose2 approachPose(const Pose2& self, const Vec2& ball, const Vec2& opp_goal, double offset,
                   double clearance);

// Push target: past the ball, at the opponent goal centre shifted `beyond` along the shot.
Pose2 pushTarget(const Vec2& ball, const Vec2& opp_goal, double beyond);

// Point at `ratio` from our goal towards the ball, facing the ball.
Pose2 blockPose(const Vec2& ball, const Vec2& own_goal, double ratio);

// Stay in place, face a point.
Pose2 facePose(const Pose2& self, const Vec2& point);

bool reachedPosition(const Pose2& self, const Pose2& target, double tol);
bool reachedHeading(const Pose2& self, double heading, double tol);

}  // namespace pinky_soccer_bt
