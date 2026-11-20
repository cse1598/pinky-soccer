#include <gtest/gtest.h>

#include <cmath>

#include "pinky_soccer_bt/soccer_logic.hpp"

using namespace pinky_soccer_bt;

namespace
{
const Vec2 kBlue{1.095, 0.0};     // opponent goal when we are yellow
const Vec2 kYellow{-1.095, 0.0};  // own goal when we are yellow
}  // namespace

TEST(Logic, MaxSigmaUsesLargestEigenvalue)
{
  std::array<double, 9> cov{0.04, 0, 0, 0, 0.01, 0, 0, 0, 1.0};
  EXPECT_NEAR(maxPositionSigma(cov), 0.2, 1e-9);   // theta variance ignored
  std::array<double, 9> rotated{0.025, 0.015, 0, 0.015, 0.025, 0, 0, 0, 0};
  EXPECT_NEAR(maxPositionSigma(rotated), 0.2, 1e-9);
  EXPECT_TRUE(ballKnown(false, cov, 0.3));
  EXPECT_FALSE(ballKnown(false, cov, 0.1));
  EXPECT_FALSE(ballKnown(true, cov, 0.3));
}

TEST(Logic, OwnHalfFollowsGoalSide)
{
  EXPECT_TRUE(ballInOwnHalf({-0.5, 0}, kYellow, 0.3));
  EXPECT_TRUE(ballInOwnHalf({0.2, 0}, kYellow, 0.3));    // within margin past the centre
  EXPECT_FALSE(ballInOwnHalf({0.4, 0}, kYellow, 0.3));
  EXPECT_TRUE(ballInOwnHalf({0.5, 0}, kBlue, 0.3));      // we defend +X
  EXPECT_FALSE(ballInOwnHalf({-0.5, 0}, kBlue, 0.3));
}

TEST(Logic, OpponentCloser)
{
  EXPECT_TRUE(opponentCloserToBall({0.5, 0}, {0, 0, 0}, {{0.6, 0}}));
  EXPECT_FALSE(opponentCloserToBall({0.5, 0}, {0, 0, 0}, {{1.2, 0}}));
  EXPECT_FALSE(opponentCloserToBall({0.5, 0}, {0, 0, 0}, {}));
}

TEST(Logic, BehindBallPoseIsOnShotLine)
{
  const auto p = behindBallPose({0.5, 0.3}, kBlue, 0.15);
  const double ux = 0.595, uy = -0.3, n = std::hypot(ux, uy);
  EXPECT_NEAR(p.x, 0.5 - 0.15 * ux / n, 1e-9);
  EXPECT_NEAR(p.y, 0.3 - 0.15 * uy / n, 1e-9);
  EXPECT_NEAR(p.theta, std::atan2(uy, ux), 1e-9);
}

TEST(Logic, IsBehindBall)
{
  // Ball ahead on the shot line, facing the goal.
  EXPECT_TRUE(isBehindBall({0.1, 0, 0}, {0.3, 0}, kBlue, 0.08, 0.35, 0.3));
  // Too far sideways, wrong heading, in front of the ball, too far behind.
  EXPECT_FALSE(isBehindBall({0.1, 0.1, 0}, {0.3, 0}, kBlue, 0.08, 0.35, 0.3));
  EXPECT_FALSE(isBehindBall({0.1, 0, 1.0}, {0.3, 0}, kBlue, 0.08, 0.35, 0.3));
  EXPECT_FALSE(isBehindBall({0.5, 0, 0}, {0.3, 0}, kBlue, 0.08, 0.35, 0.3));
  EXPECT_FALSE(isBehindBall({-0.2, 0, 0}, {0.3, 0}, kBlue, 0.08, 0.35, 0.3));
}

TEST(Logic, NotFacingOwnGoalPreventsOwnGoals)
{
  EXPECT_TRUE(notFacingOwnGoal({0, 0, 0}, kYellow, 1.57));          // facing away
  EXPECT_FALSE(notFacingOwnGoal({0, 0, M_PI}, kYellow, 1.57));      // facing own goal
  EXPECT_FALSE(notFacingOwnGoal({0, 0, M_PI - 1.0}, kYellow, 1.57));
}

TEST(Logic, ShotDirectionAlwaysTowardsOpponentSide)
{
  // Own-goal guarantee of Shoot: the push heading always has a +X component when we attack
  // +X, so PushBall never drives the ball towards our own goal line.
  for(double bx = -1.05; bx <= 1.05; bx += 0.15)
  {
    for(double by = -0.8; by <= 0.8; by += 0.2)
    {
      const auto p = behindBallPose({bx, by}, kBlue, 0.1);
      ASSERT_TRUE(isBehindBall(p, {bx, by}, kBlue, 0.08, 0.35, 0.3));
      EXPECT_GT(std::cos(p.theta), 0.0) << bx << "," << by;
      EXPECT_GT(std::cos(pushTarget({bx, by}, kBlue, 0.1).theta), 0.0);
    }
  }
}

TEST(Logic, NotFacingOwnGoalAllowsCentralShots)
{
  for(double bx = -0.75; bx <= 1.0; bx += 0.25)
  {
    for(double by = -0.35; by <= 0.35; by += 0.35)
    {
      EXPECT_TRUE(notFacingOwnGoal(behindBallPose({bx, by}, kBlue, 0.1), kYellow, 1.57))
          << bx << "," << by;
    }
  }
}

TEST(Logic, NotFacingOwnGoalBlocksCornerShotsKnownLimit)
{
  // Known limitation with min_angle=1.57 (appendix C start value): near the far corners
  // the shot heading is within 90 deg of the bearing to our own goal, so Shoot is blocked
  // and the robot keeps re-positioning. Documented in README; tune min_angle in sim.
  EXPECT_FALSE(notFacingOwnGoal(behindBallPose({1.0, 0.7}, kBlue, 0.1), kYellow, 1.57));
  EXPECT_TRUE(notFacingOwnGoal(behindBallPose({1.0, 0.7}, kBlue, 0.1), kYellow, 1.0));
}

TEST(Logic, ApproachGoesAroundWhenInFrontOfBall)
{
  // Robot between ball and opponent goal: first target is a side waypoint, not through ball.
  const Pose2 robot{0.8, 0.05, M_PI};
  const Vec2 ball{0.5, 0.0};
  const auto t = approachPose(robot, ball, kBlue, 0.15, 0.2);
  EXPECT_NEAR(std::abs(t.y), 0.2, 1e-9);
  EXPECT_GT(t.y, 0.0);   // same side as the robot
  // Robot already behind: target is the plain behind-ball pose.
  const auto b = approachPose({0.1, 0.0, 0}, ball, kBlue, 0.15, 0.2);
  EXPECT_NEAR(b.x, 0.35, 1e-9);
  EXPECT_NEAR(b.y, 0.0, 1e-9);
}

TEST(Logic, PushAndBlockTargets)
{
  const auto push = pushTarget({0.5, 0}, kBlue, 0.1);
  EXPECT_NEAR(push.x, 1.195, 1e-9);
  EXPECT_NEAR(push.theta, 0.0, 1e-9);
  const auto block = blockPose({0.0, 0.4}, kYellow, 0.35);
  EXPECT_NEAR(block.x, -1.095 + 0.35 * 1.095, 1e-9);
  EXPECT_NEAR(block.y, 0.14, 1e-9);
  EXPECT_NEAR(block.theta, std::atan2(0.4 - 0.14, 1.095 - 0.35 * 1.095), 1e-9);
}

TEST(Logic, FaceAndReach)
{
  const auto f = facePose({0, 0, 0}, {0, 1});
  EXPECT_NEAR(f.theta, M_PI / 2, 1e-9);
  EXPECT_TRUE(reachedPosition({0, 0, 0}, {0.03, 0.03, 0}, 0.05));
  EXPECT_FALSE(reachedPosition({0, 0, 0}, {0.1, 0, 0}, 0.05));
  EXPECT_TRUE(reachedHeading({0, 0, M_PI - 0.05}, -M_PI + 0.05, 0.15));
}
