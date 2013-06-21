/**
* @file BH2011BallSymbols.cpp
*
* Implementation of class BallSymbols.
*
* @author Max Risler
* \author Colin Graf
*/

#include "BH2011BallSymbols.h"
//#include "Tools/Debugging/Modify.h"
#include "Modules/Infrastructure/TeamDataProvider.h"

void BH2011BallSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("ball.position.field.x", &ballPositionField.x);
  engine.registerDecimalInputSymbol("ball.position.field.y", &ballPositionField.y);
  engine.registerDecimalInputSymbol("ball.end_position.field.x", &ballEndPositionField.x);
  engine.registerDecimalInputSymbol("ball.end_position.field.y", &ballEndPositionField.y);

  engine.registerDecimalInputSymbol("ball.x", &ballPositionRel.x);
  engine.registerDecimalInputSymbol("ball.y", &ballPositionRel.y);
  engine.registerDecimalInputSymbol("ball.kick.x", &ballKickRel.x);
  engine.registerDecimalInputSymbol("ball.kick.y", &ballKickRel.y);
  engine.registerDecimalInputSymbol("ball.end_position.x", &ballEndPositionRel.x);
  engine.registerDecimalInputSymbol("ball.end_position.y", &ballEndPositionRel.y);
  //This is a little bit faster with the Position because it's just using the last percept
  engine.registerDecimalInputSymbol("ball.seen.x", &seenBallPositionRel.x);
  engine.registerDecimalInputSymbol("ball.seen.y", &seenBallPositionRel.y);
  engine.registerDecimalInputSymbol("ball.seen.estimate.x", this, &BH2011BallSymbols::getBallLastSeenEstimateX);
  engine.registerDecimalInputSymbol("ball.seen.estimate.y", this, &BH2011BallSymbols::getBallLastSeenEstimateY);
  engine.registerDecimalInputSymbol("ball.position.seen.estimate.x", this, &BH2011BallSymbols::getBallPositionLastSeenEstimateX);
  engine.registerDecimalInputSymbol("ball.position.seen.estimate.y", this, &BH2011BallSymbols::getBallPositionLastSeenEstimateY);
  engine.registerDecimalInputSymbol("ball.seen.angle", this, &BH2011BallSymbols::getSeenBallAngle);
  engine.registerDecimalInputSymbol("ball.seen.distance", this, &BH2011BallSymbols::getBallSeenDistance);

  engine.registerDecimalInputSymbol("ball.distance", this, &BH2011BallSymbols::getBallDistance);
  engine.registerDecimalInputSymbol("ball.angle", this, &BH2011BallSymbols::getBallAngle);
  engine.registerBooleanInputSymbol("ball.was_seen", &ballWasSeen);
  engine.registerDecimalInputSymbol("ball.time_since_last_seen", &timeSinceBallWasSeen);
  engine.registerDecimalInputSymbol("ball.time_since_disappeared", this, &BH2011BallSymbols::getTimeSinceDisappeared);

  engine.registerDecimalInputSymbol("ball.speed.field.x", &ballSpeedField.x);
  engine.registerDecimalInputSymbol("ball.speed.field.y", &ballSpeedField.y);
  engine.registerDecimalInputSymbol("ball.speed.robot.x", &ballSpeedRel.x);
  engine.registerDecimalInputSymbol("ball.speed.robot.y", &ballSpeedRel.y);

  engine.registerDecimalInputSymbol("ball.time_when_own_y_axis_reached", this, &BH2011BallSymbols::getTimeWhenBallReachesOwnYAxis);
  engine.registerDecimalInputSymbol("ball.position_when_ball_reaches_own_y_axis.y", this, &BH2011BallSymbols::getYPosWhenBallReachesOwnYAxis);

  engine.registerDecimalInputSymbol("ball.distance.own_goal", this, &BH2011BallSymbols::getBallDistanceToOwnGoal);

  engine.registerDecimalInputSymbol("ball.on_field.x", this, &BH2011BallSymbols::getOnFieldX);
  engine.registerDecimalInputSymbol("ball.on_field.y", this, &BH2011BallSymbols::getOnFieldY);
  engine.registerDecimalInputSymbol("ball.on_field.angle", this, &BH2011BallSymbols::getOnFieldAngle);
  engine.registerDecimalInputSymbol("ball.on_field.speed.x", this, &BH2011BallSymbols::getOnFieldSpeedX);
  engine.registerDecimalInputSymbol("ball.on_field.speed.y", this, &BH2011BallSymbols::getOnFieldSpeedY);
}

void BH2011BallSymbols::update()
{
  timeSinceBallWasSeen = (float) frameInfo.getTimeSince(ballModel.timeWhenLastSeen);
  ballWasSeen = timeSinceBallWasSeen < 500;
  const BallState& estimate = ballModel.estimate;
  ballPositionRel = estimate.position;
  ballEndPositionRel = ballModel.endPosition;
  ballEndPositionField = Geometry::relative2FieldCoord(robotPose, ballEndPositionRel);
  ballPositionField = estimate.getPositionInFieldCoordinates(robotPose);
  ballSpeedRel = estimate.velocity;
  ballSpeedField = estimate.getVelocityInFieldCoordinates(robotPose);
  seenBallPositionRel = ballModel.lastPerception.position;
  ballLastSeenEstimate = ballModel.lastSeenEstimate.position;
  ballPositionLastSeenEstimate = ballModel.lastSeenEstimate.getPositionInFieldCoordinates(robotPose);
  ballKickRel = calculateBallInRobotOrigin(estimate.position);

}


float BH2011BallSymbols::getBallLastSeenEstimateX()
{
  return ballLastSeenEstimate.x;
}

float BH2011BallSymbols::getBallLastSeenEstimateY()
{
  return ballLastSeenEstimate.y;
}

float BH2011BallSymbols::getBallPositionLastSeenEstimateX()
{
  return ballPositionLastSeenEstimate.x;
}

float BH2011BallSymbols::getBallPositionLastSeenEstimateY()
{
  return ballPositionLastSeenEstimate.y;
}


float BH2011BallSymbols::getBallFieldRobotX()
{
  return ballPositionField.x;
}

float BH2011BallSymbols::getBallFieldRobotY()
{
  return ballPositionField.y;
}

float BH2011BallSymbols::getBallPositionRobotX()
{
  return ballPositionRel.x;
}

float BH2011BallSymbols::getBallPositionRobotY()
{
  return ballPositionRel.y;
}

float BH2011BallSymbols::getBallAngle()
{
  return toDegrees(ballModel.estimate.getAngle());
}

float BH2011BallSymbols::getSeenBallAngle()
{
  return toDegrees(ballModel.lastPerception.getAngle());
}

float BH2011BallSymbols::getBallDistance()
{
  return ballModel.estimate.getDistance();
}

float BH2011BallSymbols::getBallSeenDistance()
{
  return ballModel.lastPerception.getDistance();
}

float BH2011BallSymbols::getTimeWhenBallReachesOwnYAxis()
{
  float result = 1000000;

  float decel = (float)fieldDimensions.ballFriction;
  if(ballModel.estimate.velocity.x == 0.0f)
    return result;
  float decelX = ballModel.estimate.velocity.x * decel / ballModel.estimate.velocity.abs();
  float decelTime = ballModel.estimate.velocity.x / decelX;
  float xPosWhenStopping = ballModel.estimate.position.x + decelTime * ballModel.estimate.velocity.x - 0.5f * decel * sqr(decelTime);
  if(xPosWhenStopping * ballModel.estimate.position.x < 0)
  {
    float p(ballModel.estimate.velocity.x * 2 / decelX), q(2.0f / decelX * ballModel.estimate.position.x);
    float temp =  p * -0.5f + sqrt(sqr(p * 0.5f) - q);
    return temp;
  }
  return result;
}

float BH2011BallSymbols::getYPosWhenBallReachesOwnYAxis()
{
  if(ballSpeedRel.x == 0 || ballPositionRel.x * ballSpeedRel.x > 0) // Ball does not move or moves away
  {
    return 0.0f;
  }
  float timeWhenAxisIsReached = abs(ballPositionRel.x / ballSpeedRel.x);
  Vector2<> finalBallPos = ballPositionRel + (ballSpeedRel * timeWhenAxisIsReached);

  //MODIFY("behavior symbols:ball:finalBallPos", finalBallPos);

  return finalBallPos.y;
}

float BH2011BallSymbols::getBallDistanceToOwnGoal()
{
  return (ballPositionField - Vector2<>((float) fieldDimensions.xPosOwnGroundline, 0.f)).abs();
}

float BH2011BallSymbols::getTimeSinceDisappeared()
{
  return float(frameInfo.getTimeSince(ballHypotheses.timeWhenDisappeared));
}

float BH2011BallSymbols::getOnFieldX()
{
  return ballModel.estimate.getPositionInFieldCoordinates(robotPose).x;
}

float BH2011BallSymbols::getOnFieldY()
{
  return ballModel.estimate.getPositionInFieldCoordinates(robotPose).y;
}

float BH2011BallSymbols::getOnFieldAngle()
{
  Vector2<> ballOnField = ballModel.estimate.getPositionInFieldCoordinates(robotPose);
  return toDegrees((ballOnField - robotPose.translation).angle());
}

float BH2011BallSymbols::getOnFieldSpeedX()
{
  return ballModel.estimate.getVelocityInFieldCoordinates(robotPose).x;
}

float BH2011BallSymbols::getOnFieldSpeedY()
{
  return ballModel.estimate.getVelocityInFieldCoordinates(robotPose).y;
}

Vector2<> BH2011BallSymbols::calculateBallInRobotOrigin(const Vector2<>& ballRel)
{
  // calculate "center of hip" position from left foot
  Pose3D fromLeftFoot(torsoMatrix.rotation);
  fromLeftFoot.conc(robotModel.limbs[MassCalibration::footLeft]);
  fromLeftFoot.translate(0, 0, -robotDimensions.heightLeg5Joint);
  //fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoMatrix.rotation;

  // calculate "center of hip" position from right foot
  Pose3D fromRightFoot(torsoMatrix.rotation);
  fromRightFoot.conc(robotModel.limbs[MassCalibration::footRight]);
  fromRightFoot.translate(0, 0, -robotDimensions.heightLeg5Joint);
// fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoMatrix.rotation;

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z < fromRightFoot.translation.z;

  // calculate foot span
  const Vector3<> newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3D newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x / (useLeft ? 2.f : -2.f), newFootSpan.y / (useLeft ? 2.f : -2.f), 0);
  //newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  const Vector3<> foot(useLeft ? fromLeftFoot.translation : fromRightFoot.translation);

  return Vector2<>(ballRel.x + newTorsoMatrix.translation.x + foot.x, ballRel.y + newTorsoMatrix.translation.y + foot.y);
}
