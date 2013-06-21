/**
* @file GameDataProvider.cpp
* This file implements a module that provides the data received from the game controller.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include "GameDataProvider.h"
//#include "Tools/Settings.h"



GameDataProvider::GameDataProvider() :
  gameControlTimeStamp(0),
  lastSent(0),
  ownTeamInfoSet(false)
{
  memset(&gameControlData, 0, sizeof(gameControlData));
  // TODO: setear teamNumber y teamColor
  gameControlData.teams[0].teamNumber = 0;
  gameControlData.teams[0].teamColor = 0;

  //gameControlData.teams[0].teamNumber = Global::getSettings().teamNumber;
  //gameControlData.teams[0].teamColor = Global::getSettings().teamColor;
  START_GAME_CONTROL;
}

void GameDataProvider::init(const FrameInfo& theFrameInfo)
{
  gameControlTimeStamp = theFrameInfo.time;
}

void GameDataProvider::update(RobotInfo& robotInfo, const FrameInfo& theFrameInfo, const BehaviorControlOutput& theBehaviorControlOutput)
{
  unsigned timeStamp = gameControlTimeStamp;
  if(RECEIVE_GAME_CONTROL(gameControlData))
    timeStamp = theFrameInfo.time;

  bool received = false;
  if(theFrameInfo.getTimeSince(timeStamp) < 500)
  {
    for(int i = 0; i < 2; ++i)
      //if(Global::getSettings().teamNumber == (int)gameControlData.teams[i].teamNumber)
      //TODO:
      if( 0 == (int)gameControlData.teams[i].teamNumber)
      {
        gameControlTimeStamp = timeStamp;
        const RoboCup::TeamInfo& t = gameControlData.teams[i];
        //ASSERT(Global::getSettings().playerNumber >= 1 && Global::getSettings().playerNumber <= MAX_NUM_PLAYERS);
        ASSERT(2 >= 1 && 2 <= MAX_NUM_PLAYERS);
        RoboCup::RoboCupGameControlReturnData returnPacket;
        memcpy(returnPacket.header, GAMECONTROLLER_RETURN_STRUCT_HEADER, sizeof(returnPacket.header));
        returnPacket.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
        //returnPacket.team = Global::getSettings().teamNumber;
        returnPacket.team = 0;
        //returnPacket.player = Global::getSettings().playerNumber;
        returnPacket.player = 2;

        if(&theBehaviorControlOutput && robotInfo.penalty != theBehaviorControlOutput.robotInfo.penalty)
        {
          returnPacket.message = theBehaviorControlOutput.robotInfo.penalty != PENALTY_NONE
                                 ? GAMECONTROLLER_RETURN_MSG_MAN_PENALISE
                                 : GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE;
          lastSent = theFrameInfo.time;
          (void) SEND_GAME_CONTROL(returnPacket);
        }
        else if(theFrameInfo.getTimeSince(lastSent) > 500)
        {
          returnPacket.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
          lastSent = theFrameInfo.time;
          (void) SEND_GAME_CONTROL(returnPacket);
        }

        //(RoboCup::RobotInfo&) robotInfo = t.players[Global::getSettings().playerNumber - 1];
        (RoboCup::RobotInfo&) robotInfo = t.players[2 - 1];
        received = true;
        break;
      }
  }

  //if(!received && &theBehaviorControlOutput && Global::getSettings().teamNumber == (int)theBehaviorControlOutput.ownTeamInfo.teamNumber)
  if(!received && &theBehaviorControlOutput && 0 == (int)theBehaviorControlOutput.ownTeamInfo.teamNumber)
    robotInfo = theBehaviorControlOutput.robotInfo;

  //robotInfo.number = Global::getSettings().playerNumber;
  robotInfo.number = 2;
}

void GameDataProvider::update(OwnTeamInfo& ownTeamInfo, const FrameInfo& theFrameInfo, const BehaviorControlOutput& theBehaviorControlOutput, const FieldDimensions& theFieldDimensions)
{
  if(!ownTeamInfoSet)
  {
    (RoboCup::TeamInfo&) ownTeamInfo = gameControlData.teams[0];
    //ownTeamInfo.teamNumber = Global::getSettings().teamNumber;
    ownTeamInfo.teamNumber = 0;
    ownTeamInfoSet = true;
  }

  bool received = false;
  if(theFrameInfo.getTimeSince(gameControlTimeStamp) < 500)
  {
    for(int i = 0; i < 2; ++i)
      //if(Global::getSettings().teamNumber == (int)gameControlData.teams[i].teamNumber)
      if(0 == (int)gameControlData.teams[i].teamNumber)
      {
        (RoboCup::TeamInfo&) ownTeamInfo = gameControlData.teams[i];
        received = true;
        break;
      }

  }
  //if(!received && &theBehaviorControlOutput && Global::getSettings().teamNumber == (int)theBehaviorControlOutput.ownTeamInfo.teamNumber)
  if(!received && &theBehaviorControlOutput && 0 == (int)theBehaviorControlOutput.ownTeamInfo.teamNumber)
    ownTeamInfo = theBehaviorControlOutput.ownTeamInfo;

  //MODIFY("representation:OwnTeamInfo", ownTeamInfo);
  //theFieldDimensions.drawPolygons(ownTeamInfo.teamColor);
}

void GameDataProvider::update(OpponentTeamInfo& opponentTeamInfo)
{
  for(int i = 0; i < 2; ++i)
    //if(Global::getSettings().teamNumber == (int)gameControlData.teams[i].teamNumber)
    if(0 == (int)gameControlData.teams[i].teamNumber)
    {
      (RoboCup::TeamInfo&) opponentTeamInfo = gameControlData.teams[1 - i];
      break;
    }
}

void GameDataProvider::update(GameInfo& gameInfo, const FrameInfo& theFrameInfo, const BehaviorControlOutput& theBehaviorControlOutput)
{
  //if((theFrameInfo.getTimeSince(gameControlTimeStamp) < 500 || !&theBehaviorControlOutput || Global::getSettings().teamNumber != (int)theBehaviorControlOutput.ownTeamInfo.teamNumber) &&
  //   (Global::getSettings().teamNumber == (int)gameControlData.teams[0].teamNumber || Global::getSettings().teamNumber == (int)gameControlData.teams[1].teamNumber))

  if((theFrameInfo.getTimeSince(gameControlTimeStamp) < 500 || !&theBehaviorControlOutput || 0 != (int)theBehaviorControlOutput.ownTeamInfo.teamNumber) &&
         (0 == (int)gameControlData.teams[0].teamNumber || 0 == (int)gameControlData.teams[1].teamNumber))
    memcpy(&(RoboCup::RoboCupGameControlData&) gameInfo, &gameControlData, (char*) gameControlData.teams - (char*) &gameControlData);
  //else if(&theBehaviorControlOutput && Global::getSettings().teamNumber == (int)theBehaviorControlOutput.ownTeamInfo.teamNumber)
  else if(&theBehaviorControlOutput && 0 == (int)theBehaviorControlOutput.ownTeamInfo.teamNumber)
    gameInfo = theBehaviorControlOutput.gameInfo;
  gameInfo.timeSinceLastPackageReceived = (float) theFrameInfo.getTimeSince(gameControlTimeStamp);
}

//MAKE_MODULE(GameDataProvider, Infrastructure)
