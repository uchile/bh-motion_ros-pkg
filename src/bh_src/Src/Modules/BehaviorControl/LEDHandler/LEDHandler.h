/**
* @file LEDHandler.h
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"

/*MODULE(LEDHandler)
  REQUIRES(FrameInfo)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(BallModel)
  REQUIRES(GoalPercept)
  REQUIRES(FilteredSensorData)
  REQUIRES(TeamMateData)
  REQUIRES(GroundContactState)
  REQUIRES(BehaviorControlOutput)
  REQUIRES(BehaviorLEDRequest)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(LEDRequest)
END_MODULE*/

class LEDHandler //: public LEDHandlerBase
{
public:
  void update(LEDRequest& ledRequest,const FrameInfo& theFrameInfo, const BehaviorLEDRequest& theBehaviorLEDRequest, const FilteredSensorData& theFilteredSensorData, const TeamMateData& theTeamMateData, const GameInfo& theGameInfo, const BallModel& theBallModel, const GroundContactState& theGroundContactState, const OwnTeamInfo& theOwnTeamInfo, const BehaviorControlOutput& theBehaviorControlOutput, const RobotInfo& theRobotInfo, const GoalPercept& theGoalPercept);
private:
  void setEyeColor(LEDRequest& ledRequest,
                   bool left,
                   BehaviorLEDRequest::EyeColor col,
                   LEDRequest::LEDState s);

  void setRightEar(LEDRequest& ledRequest, const BehaviorLEDRequest& theBehaviorLEDRequest, const FilteredSensorData theFilteredSensorData);
  void setLeftEar(LEDRequest& ledRequest, const BehaviorLEDRequest& theBehaviorLEDRequest, const TeamMateData& theTeamMateData, const FrameInfo& theFrameInfo, const GameInfo& theGameInfo);
  void setLeftEye(LEDRequest& ledRequest, const BehaviorLEDRequest theBehaviorLEDRequest, const GroundContactState& theGroundContactState , const FrameInfo& theFrameInfo, const BallModel& theBallModel, const GoalPercept& theGoalPercept);
  void setRightEye(LEDRequest& ledRequest, const BehaviorLEDRequest& theBehaviorLEDRequest, const GroundContactState& theGroundContactState, const BehaviorControlOutput& theBehaviorControlOutput, const RobotInfo& theRobotInfo, const GameInfo& theGameInfo, const FrameInfo& theFrameInfo);
  void setChestButton(LEDRequest& ledRequest, const RobotInfo& theRobotInfo, const GameInfo& theGameInfo);
  void setLeftFoot(LEDRequest& ledRequest, const OwnTeamInfo& theOwnTeamInfo);
  void setRightFoot(LEDRequest& ledRequest, const GameInfo& theGameInfo, const OwnTeamInfo& theOwnTeamInfo);
};

