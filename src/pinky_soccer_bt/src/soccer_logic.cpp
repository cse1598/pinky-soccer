#include "pinky_soccer_bt/soccer_logic.hpp"

#include <algorithm>
#include <cmath>

namespace pinky_soccer_bt
{

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double distance(const Vec2& a, const Vec2& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double bearing(const Vec2& from, const Vec2& to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

Vec2 position(const Pose2& p)
{
  return {p.x, p.y};
}

double maxPositionSigma(const std::array<double, 9>& cov)
{
  // Eigenvalues of the symmetric 2x2 block [[a, b], [b, d]].
  const double a = cov[0], d = cov[4], b = 0.5 * (cov[1] + cov[3]);
  const double mean = 0.5 * (a + d);
  const double radius = std::sqrt(std::max(0.0, 0.25 * (a - d) * (a - d) + b * b));
  return std::sqrt(std::max(0.0, mean + radius));
}

bool ballKnown(bool lost, const std::array<double, 9>& cov, double max_sigma)
{
  return !lost && maxPositionSigma(cov) <= max_sigma;
}

bool ballInOwnHalf(const Vec2& ball, const Vec2& own_goal, double margin)
{
  // own_goal.x < 0 means we defend -X. "Own half" extends `margin` past the centre line.
  const double side = own_goal.x < 0.0 ? -1.0 : 1.0;
  return side * ball.x > -margin;
}

bool opponentCloserToBall(const Vec2& ball, const Pose2& self, const std::vector<Vec2>& opponents)
{
  const double mine = distance(position(self), ball);
  return std::any_of(opponents.begin(), opponents.end(),
                     [&](const Vec2& o) { return distance(o, ball) < mine; });
}

namespace
{
// Unit vector from the ball towards the opponent goal (shot direction).
Vec2 shotDirection(const Vec2& ball, const Vec2& opp_goal)
{
  const double dx = opp_goal.x - ball.x, dy = opp_goal.y - ball.y;
  const double n = std::hypot(dx, dy);
  if(n < 1e-9)
  {
    return {opp_goal.x >= 0.0 ? 1.0 : -1.0, 0.0};
  }
  return {dx / n, dy / n};
}
}  // namespace

Pose2 behindBallPose(const Vec2& ball, const Vec2& opp_goal, double offset)
{
  const Vec2 u = shotDirection(ball, opp_goal);
  return {ball.x - u.x * offset, ball.y - u.y * offset, std::atan2(u.y, u.x)};
}

bool isBehindBall(const Pose2& self, const Vec2& ball, const Vec2& opp_goal, double tol_dist,
                  double tol_angle, double max_behind)
{
  const Vec2 u = shotDirection(ball, opp_goal);
  const double rx = self.x - ball.x, ry = self.y - ball.y;
  const double along = rx * u.x + ry * u.y;          // < 0: robot behind the ball
  const double lateral = std::abs(u.x * ry - u.y * rx);
  const double heading_err = std::abs(wrapAngle(self.theta - std::atan2(u.y, u.x)));
  return along < 0.0 && -along <= max_behind && lateral <= tol_dist && heading_err <= tol_angle;
}

bool notFacingOwnGoal(const Pose2& self, const Vec2& own_goal, double min_angle)
{
  return std::abs(wrapAngle(self.theta - bearing(position(self), own_goal))) >= min_angle;
}

Pose2 approachPose(const Pose2& self, const Vec2& ball, const Vec2& opp_goal, double offset,
                   double clearance)
{
  const Vec2 u = shotDirection(ball, opp_goal);
  const double rx = self.x - ball.x, ry = self.y - ball.y;
  const double along = rx * u.x + ry * u.y;
  const double heading = std::atan2(u.y, u.x);
  if(along < -0.5 * offset)
  {
    return behindBallPose(ball, opp_goal, offset);
  }
  // Robot is beside or in front of the ball: go around on its own side first.
  const double cross = u.x * ry - u.y * rx;
  const double side = cross >= 0.0 ? 1.0 : -1.0;
  const Vec2 n{-u.y * side, u.x * side};
  return {ball.x + n.x * clearance - u.x * offset, ball.y + n.y * clearance - u.y * offset,
          heading};
}

Pose2 pushTarget(const Vec2& ball, const Vec2& opp_goal, double beyond)
{
  const Vec2 u = shotDirection(ball, opp_goal);
  return {opp_goal.x + u.x * beyond, opp_goal.y + u.y * beyond, std::atan2(u.y, u.x)};
}

Pose2 blockPose(const Vec2& ball, const Vec2& own_goal, double ratio)
{
  const Vec2 p{own_goal.x + ratio * (ball.x - own_goal.x),
               own_goal.y + ratio * (ball.y - own_goal.y)};
  return {p.x, p.y, bearing(p, ball)};
}

Pose2 facePose(const Pose2& self, const Vec2& point)
{
  return {self.x, self.y, bearing(position(self), point)};
}

bool reachedPosition(const Pose2& self, const Pose2& target, double tol)
{
  return std::hypot(self.x - target.x, self.y - target.y) <= tol;
}

bool reachedHeading(const Pose2& self, double heading, double tol)
{
  return std::abs(wrapAngle(self.theta - heading)) <= tol;
}

}  // namespace pinky_soccer_bt
