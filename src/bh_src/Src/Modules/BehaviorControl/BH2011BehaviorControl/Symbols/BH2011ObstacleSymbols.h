/**
* @file BH2011ObstacleSymbols.h
* Declaration of class BH2011ObstacleSymbols
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ArmContactModel.h"

/**
* @class BH2011ObstacleSymbols
* Symbols for duelling
*/
class BH2011ObstacleSymbols : public Symbols
{
public:
  /** Constructor */
  BH2011ObstacleSymbols(const FrameInfo& frameInfo, const ObstacleModel& obstacleModel, const ArmContactModel& armContactModel):
    frameInfo(frameInfo), obstacleModel(obstacleModel), armContactModel(armContactModel), lastTime(0) {}

  /** Registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

private:
  const FrameInfo& frameInfo;              /**< Current time */
  const ObstacleModel& obstacleModel;      /**< Reference to ObstacleModel which contains obstacle information */
  const ArmContactModel& armContactModel;  /**< Indication of something touching an arm  */

  float distanceToClosestCenterLeft; /**< As the name says... */
  float distanceToClosestCenterRight; /**< As the name says... */
  float distanceToClosestCenter; /**< As the name says... */
  float distanceToClosest;
  float angleToClosest;
  float freeKickAngleLeft;
  float freeKickAngleRight;
  float distanceToClosestCenterLeft_x;
  float distanceToClosestCenterLeft_y;
  float distanceToClosestCenterRight_x;
  float distanceToClosestCenterRight_y;
  float angleToClosestLeft;
  float angleToClosestRight;

  unsigned int lastTime; /**< The time when \c distanceToClisestCenterLeft, \c distanceToClosestCenterRight and \c distanceToClosestCenter was computed */

  void computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  float getDistanceToClosestCenterLeft();
  float getDistanceToClosestCenterRight();
  float getDistanceToClosestCenter();
  float getDistanceToClosest();
  float getAngleToClosest();
  float getFreeKickAngleLeft();
  float getFreeKickAngleRight();
  float getDistanceToClosestCenterLeft_x();
  float getDistanceToClosestCenterLeft_y();
  float getDistanceToClosestCenterRight_x();
  float getDistanceToClosestCenterRight_y();
  float getAngleToClosestLeft();
  float getAngleToClosestRight();
  bool  getArmLeft();
  bool  getArmRight();
};
