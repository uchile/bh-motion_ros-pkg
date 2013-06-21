/**
* @file GameDataProvider.h
* This file implements a module that provides the data received from the game controller.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Tools/ProcessFramework/GameHandler.h"

/*
MODULE(GameDataProvider)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(RobotInfo)
  USES(BehaviorControlOutput)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotInfo)
  PROVIDES(OwnTeamInfo)
  PROVIDES_WITH_MODIFY(OpponentTeamInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(GameInfo)
END_MODULE
*/

class GameDataProvider // : public GameDataProviderBase
{
private:
  GAME_CONTROL; /**< The connection to the game controller. */
  unsigned gameControlTimeStamp; /**< The time when the last GameControlData was received. */
  unsigned lastSent; /**< The time when the last game controller return message was sent */
  bool ownTeamInfoSet; /**< Was the own team info already set? */
  RoboCup::RoboCupGameControlData gameControlData; /**< The game control data received from the game controller. */

public:
  void update(RobotInfo& robotInfo, const FrameInfo& theFrameInfo, const BehaviorControlOutput& theBehaviorControlOutput);
  void update(OwnTeamInfo& ownTeamInfo, const FrameInfo& theFrameInfo, const BehaviorControlOutput& theBehaviorControlOutput, const FieldDimensions& theFieldDimensions);
  void update(OpponentTeamInfo& opponentTeamInfo);
  void update(GameInfo& gameInfo, const FrameInfo& theFrameInfo, const BehaviorControlOutput& theBehaviorControlOutput);

private:
  void init(const FrameInfo& theFrameInfo);

public:
  /**
  * Default constructor.
  */
  GameDataProvider();
};
