/**
* @file BH2011ObstacleSymbols.cpp
* Implementation of class BH2011ObstacleSymbols.
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#include "BH2011ObstacleSymbols.h"

void BH2011ObstacleSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_left", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterLeft);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_right", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterRight);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center", this, &BH2011ObstacleSymbols::getDistanceToClosestCenter);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest", this, &BH2011ObstacleSymbols::getDistanceToClosest);
  engine.registerDecimalInputSymbol("obstacle.angle_to_closest", this, &BH2011ObstacleSymbols::getAngleToClosest);
  engine.registerDecimalInputSymbol("obstacle.free_kick_angle_left", this, &BH2011ObstacleSymbols::getFreeKickAngleLeft);
  engine.registerDecimalInputSymbol("obstacle.free_kick_angle_right", this, &BH2011ObstacleSymbols::getFreeKickAngleRight);
  engine.registerBooleanInputSymbol("obstacle.arm_left", this, &BH2011ObstacleSymbols::getArmLeft);
  engine.registerBooleanInputSymbol("obstacle.arm_right", this, &BH2011ObstacleSymbols::getArmRight);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_left.x", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterLeft_x);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_left.y", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterLeft_y);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_right.x", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterRight_x);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_right.y", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterRight_y);
  engine.registerDecimalInputSymbol("obstacle.angle_to_closest_left", this, &BH2011ObstacleSymbols::getAngleToClosestLeft);
  engine.registerDecimalInputSymbol("obstacle.angle_to_closest_right", this, &BH2011ObstacleSymbols::getAngleToClosestRight);
}

void BH2011ObstacleSymbols::computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse()
{
  if(lastTime == frameInfo.time)
    return;
  lastTime = frameInfo.time;
  freeKickAngleLeft = 0.f;
  freeKickAngleRight = 0.f;
  const float maxDistance = 3000.f; // infinite
  const float maxSqrDistance = maxDistance * maxDistance;
  const float maxKickAngleDistance = 1000.f;
  const float maxSqrKickAngleDistance = maxKickAngleDistance * maxKickAngleDistance;
  const float minKickOpeningAngle = fromDegrees(15.f) / 2.f;
  distanceToClosest = distanceToClosestCenterLeft = distanceToClosestCenterRight = maxSqrDistance;
  angleToClosest = 0.f;

  for(std::vector<ObstacleModel::Obstacle>::const_iterator iter = obstacleModel.obstacles.begin(), end = obstacleModel.obstacles.end(); iter != end; ++iter)
  {
    const ObstacleModel::Obstacle& obstacle = *iter;

    // Compute distance and angle to closest obstacle:
    float sqrAbs = obstacle.closestPoint.squareAbs();
    if(sqrAbs < distanceToClosest)
    {
      distanceToClosest = sqrAbs;
      angleToClosest = obstacle.closestPoint.angle();
    }

    // Compute distances to sectors:
    if(sqrAbs < distanceToClosestCenterLeft)
      if((obstacle.rightCorner.x > 0.f && obstacle.rightCorner.y >= 0.f && obstacle.rightCorner.y < 150.f) || // right corner point in left rectangle
          (obstacle.leftCorner.x > 0.f && obstacle.leftCorner.y >= 0.f && obstacle.leftCorner.y < 150.f) ||  // left corner point in left rectangle
          ((obstacle.rightCorner.x > 0.f || obstacle.leftCorner.x > 0.f) && obstacle.rightCorner.y < 0.f && obstacle.leftCorner.y > 0.f)) // line segement from left corner to right corner crosses the left rectangle
        distanceToClosestCenterLeft = sqrAbs;
		angleToClosestLeft = obstacle.closestPoint.angle();
    if(sqrAbs < distanceToClosestCenterRight)
      if((obstacle.rightCorner.x > 0.f && obstacle.rightCorner.y > -150.f && obstacle.rightCorner.y <= 0.f) || // right corner point in right rectangle
          (obstacle.leftCorner.x > 0.f && obstacle.leftCorner.y > -150.f && obstacle.leftCorner.y <= 0.f) ||  // left corner point in right rectangle
          ((obstacle.rightCorner.x > 0.f || obstacle.leftCorner.x > 0.f) && obstacle.rightCorner.y < 0.f && obstacle.leftCorner.y > 0.f)) // line segement from left corner to right corner crosses the right rectangle
        distanceToClosestCenterRight = sqrAbs;
		angleToClosestRight = obstacle.closestPoint.angle();

    // Compute kick angles:
    if(sqrAbs < maxSqrKickAngleDistance) // Consider this obstacle
    {
      float leftObstacleAngle = obstacle.leftCorner.angle();
      float rightObstacleAngle = obstacle.rightCorner.angle();
      // Check, if right angle is inside obstacle:
      if((leftObstacleAngle + minKickOpeningAngle > freeKickAngleLeft) && (rightObstacleAngle  - minKickOpeningAngle < freeKickAngleLeft))
      {
        freeKickAngleLeft = leftObstacleAngle + minKickOpeningAngle;
      }
      // Check, if left angle is inside obstacle:
      if((leftObstacleAngle  + minKickOpeningAngle > freeKickAngleRight) && (rightObstacleAngle - minKickOpeningAngle < freeKickAngleRight))
      {
        freeKickAngleRight = rightObstacleAngle - minKickOpeningAngle;
      }
    }
  }

  distanceToClosest = distanceToClosest < maxSqrDistance ? sqrt(distanceToClosest) : maxDistance;
  distanceToClosestCenterLeft = distanceToClosestCenterLeft < maxSqrDistance ? sqrt(distanceToClosestCenterLeft) : maxDistance;
  distanceToClosestCenterRight = distanceToClosestCenterRight < maxSqrDistance ? sqrt(distanceToClosestCenterRight) : maxDistance;
  distanceToClosestCenter = distanceToClosestCenterLeft < distanceToClosestCenterRight ? distanceToClosestCenterLeft : distanceToClosestCenterRight;
  angleToClosest = toDegrees(angleToClosest);
  freeKickAngleLeft = toDegrees(freeKickAngleLeft);
  freeKickAngleRight = toDegrees(freeKickAngleRight);
  distanceToClosestCenterLeft_x = distanceToClosestCenterLeft*cos(angleToClosestLeft);
  distanceToClosestCenterLeft_y = distanceToClosestCenterLeft*sin(angleToClosestLeft);
  distanceToClosestCenterRight_x = distanceToClosestCenterRight*cos(angleToClosestRight);
  distanceToClosestCenterRight_y = distanceToClosestCenterRight*sin(angleToClosestRight);
  angleToClosestLeft = toDegrees(angleToClosestLeft);
  angleToClosestRight = toDegrees(angleToClosestRight);
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterLeft()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterLeft;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterRight()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterRight;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenter()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenter;
}

float BH2011ObstacleSymbols::getDistanceToClosest()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosest;
}

float BH2011ObstacleSymbols::getAngleToClosest()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return angleToClosest;
}

float BH2011ObstacleSymbols::getFreeKickAngleLeft()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return freeKickAngleLeft;
}

float BH2011ObstacleSymbols::getFreeKickAngleRight()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return freeKickAngleRight;
}

bool BH2011ObstacleSymbols::getArmLeft()
{
  return armContactModel.contactLeft;
}

bool BH2011ObstacleSymbols::getArmRight()
{
  return armContactModel.contactRight;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterLeft_x()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterLeft_x;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterLeft_y()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterLeft_y;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterRight_x()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterRight_x;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterRight_y()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterRight_y;
}

float BH2011ObstacleSymbols::getAngleToClosestLeft()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return angleToClosestLeft;
}

float BH2011ObstacleSymbols::getAngleToClosestRight()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return angleToClosestRight;
}