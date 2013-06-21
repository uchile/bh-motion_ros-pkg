/**
* @file LEDHandler.cpp
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#include "LEDHandler.h"

void LEDHandler::update(LEDRequest& ledRequest,const FrameInfo& theFrameInfo, const BehaviorLEDRequest& theBehaviorLEDRequest, const FilteredSensorData& theFilteredSensorData, const TeamMateData& theTeamMateData, const GameInfo& theGameInfo, const BallModel &theBallModel, const GroundContactState& theGroundContactState, const OwnTeamInfo& theOwnTeamInfo, const BehaviorControlOutput& theBehaviorControlOutput, const RobotInfo& theRobotInfo, const GoalPercept& theGoalPercept)
{
  //reset
  for(int i = 0; i < ledRequest.numOfLEDs; ++i)
    ledRequest.ledStates[i] = LEDRequest::off;

  //update
  setRightEar(ledRequest, theBehaviorLEDRequest,theFilteredSensorData);
  setLeftEar(ledRequest, theBehaviorLEDRequest, theTeamMateData, theFrameInfo,theGameInfo);
  setRightEye(ledRequest,theBehaviorLEDRequest,theGroundContactState,theBehaviorControlOutput, theRobotInfo, theGameInfo, theFrameInfo);
  setLeftEye(ledRequest, theBehaviorLEDRequest,theGroundContactState,theFrameInfo,theBallModel, theGoalPercept);
  setChestButton(ledRequest,theRobotInfo,theGameInfo);
  setLeftFoot(ledRequest,theOwnTeamInfo);
  setRightFoot(ledRequest,theGameInfo,theOwnTeamInfo);
}

void LEDHandler::setRightEar(LEDRequest& ledRequest, const BehaviorLEDRequest& theBehaviorLEDRequest, const FilteredSensorData theFilteredSensorData)
{
  //right ear -> battery

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar];

  int onLEDs = min((int)(theFilteredSensorData.data[FilteredSensorData::batteryLevel] / 0.1f), 9);

  for(int i = 0; i <= onLEDs; i++)
    ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = state;
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest, const BehaviorLEDRequest& theBehaviorLEDRequest, const TeamMateData& theTeamMateData, const FrameInfo& theFrameInfo, const GameInfo& theGameInfo)
{
  //left ear -> connected players
  //          + GameController connection lost -> freaky blinking

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEar];

  unsigned runTo = theTeamMateData.numOfConnectedPlayers * 3;
    if(runTo == 9)
      runTo = 10;
  if(theGameInfo.timeSinceLastPackageReceived < 2000)
    for(unsigned i = 0; i < runTo; ++i)
      ledRequest.ledStates[LEDRequest::earsLeft0Deg + i] = state;
  else
  {
    if(runTo == 0)
      runTo = 1;
    for(unsigned i = 0; i < runTo; i += 2)
      ledRequest.ledStates[LEDRequest::earsLeft0Deg + i] = theFrameInfo.time & 512 ? state : LEDRequest::off;
    for(unsigned i = 1; i < runTo; i += 2)
      ledRequest.ledStates[LEDRequest::earsLeft0Deg + i] = theFrameInfo.time & 512 ? LEDRequest::off : state;
  }
}

void LEDHandler::setEyeColor(LEDRequest& ledRequest,
                             bool left,
                             BehaviorLEDRequest::EyeColor col,
                             LEDRequest::LEDState s)
{
  LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

  static const int redOffset = 0,
                   greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
                   blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg,
                   numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg;

  LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

  switch(col)
  {
  case BehaviorLEDRequest::default_color:
    ASSERT(false);
    break;
  case BehaviorLEDRequest::red:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = s;
    break;
  case BehaviorLEDRequest::green:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = s;
    break;
  case BehaviorLEDRequest::blue:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  case BehaviorLEDRequest::white:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = s;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = s;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  case BehaviorLEDRequest::magenta:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = halfState;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = s;
    break;
  case BehaviorLEDRequest::yellow:
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = halfState;
    for(int i = 0; i <= numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = s;
    break;
  default:
    ASSERT(false);
    break;
  }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest, const BehaviorLEDRequest theBehaviorLEDRequest, const GroundContactState& theGroundContactState , const FrameInfo& theFrameInfo, const BallModel& theBallModel, const GoalPercept& theGoalPercept)
{
  //left eye -> groundContact ? ballSeen and GoalSeen : blue

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye];

  //no groundContact
  if(!theGroundContactState.contact && (theFrameInfo.time & 512))
    setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
  //overwrite
  else if(theBehaviorLEDRequest.leftEyeColor != BehaviorLEDRequest::default_color)
    //blue
    setEyeColor(ledRequest, true, theBehaviorLEDRequest.leftEyeColor, state);
  //default
  else
  {
    bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;
    bool ownGoalSeen =    theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::LEFT_OWN].timeWhenLastSeen) < 250
                          && theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::RIGHT_OWN].timeWhenLastSeen) < 250;
    bool oppGoalSeen =     theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].timeWhenLastSeen) < 250
                           && theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].timeWhenLastSeen) < 250;
    bool goalSeen = ownGoalSeen || oppGoalSeen;

    if(ballSeen && goalSeen)
      //red
      setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
    else if(ballSeen)
      //white
      setEyeColor(ledRequest, true, BehaviorLEDRequest::white, state);
    else if(goalSeen)
      //green
      setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);
  }
}

void LEDHandler::setRightEye(LEDRequest& ledRequest, const BehaviorLEDRequest& theBehaviorLEDRequest, const GroundContactState& theGroundContactState, const BehaviorControlOutput& theBehaviorControlOutput, const RobotInfo& theRobotInfo, const GameInfo& theGameInfo, const FrameInfo& theFrameInfo)
{
  //right eye -> groundContact ? role : role -> blinking
  //           + penalty shootout: native_{striker,keeper} ? {striker,keeper} : off

  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye];

  //no groundContact
  if(!theGroundContactState.contact && (theFrameInfo.time & 512))
    setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
  //overwrite
  else if(theBehaviorLEDRequest.rightEyeColor != BehaviorLEDRequest::default_color)
    setEyeColor(ledRequest, false, theBehaviorLEDRequest.rightEyeColor, state);
  else
  {
    int role = theBehaviorControlOutput.behaviorData.role;
    //penalty shootout
    if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
    {
      switch(theRobotInfo.number)
      {
      case BehaviorData::keeper:
        role = BehaviorData::keeper;
        break;
      case BehaviorData::striker:
        role = BehaviorData::striker;
        break;
      default:
        role = BehaviorData::undefined;
        break;
      }
    }
    switch(role)
    {
    case BehaviorData::keeper:
      //blue
      setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
      break;
    case BehaviorData::defender:
      //white
      setEyeColor(ledRequest, false, BehaviorLEDRequest::white, state);
      break;
    case BehaviorData::supporter:
      //green
      setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state);
      break;
    case BehaviorData::striker:
      //red
      setEyeColor(ledRequest, false, BehaviorLEDRequest::red, state);
      break;
    case BehaviorData::offensiveSupporter:
      //yellow
      setEyeColor(ledRequest, false, BehaviorLEDRequest::yellow, state);
      break;
    case BehaviorData::defensiveSupporter:
      //magenta
      setEyeColor(ledRequest, false, BehaviorLEDRequest::magenta, state);
      break;
    case BehaviorData::undefined:
      //off
      break;
    default:
      ASSERT(false);
    }
  }
}

void LEDHandler::setChestButton(LEDRequest& ledRequest, const RobotInfo& theRobotInfo, const GameInfo& theGameInfo)
{
  //chest button -> game state

  if(theRobotInfo.penalty != PENALTY_NONE)
    //red
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
  else
    switch(theGameInfo.state)
    {
    case STATE_INITIAL:
      break;
    case STATE_READY:
      //blue
      ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
      break;
    case STATE_SET:
      //yellow
      ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
      ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
      break;
    case STATE_PLAYING:
      //green
      ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
      break;
    case STATE_FINISHED:
      break;
    default:
      ASSERT(false);
    }
}

void LEDHandler::setLeftFoot(LEDRequest& ledRequest, const OwnTeamInfo& theOwnTeamInfo)
{
  //left foot -> team color

  if(theOwnTeamInfo.teamColour == TEAM_BLUE)
    ledRequest.ledStates[LEDRequest::footLeftBlue] = LEDRequest::on;
  else if(theOwnTeamInfo.teamColour == TEAM_RED)
    ledRequest.ledStates[LEDRequest::footLeftRed] = LEDRequest::on;
}

void LEDHandler::setRightFoot(LEDRequest& ledRequest, const GameInfo& theGameInfo, const OwnTeamInfo& theOwnTeamInfo)
{
  //right foot -> kickoff, penaltyshoot

  //penalty shootout -> green
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
    ledRequest.ledStates[LEDRequest::footRightGreen] = LEDRequest::on;
  //own team kickoff -> white
  else if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamColour)
  {
    ledRequest.ledStates[LEDRequest::footRightRed] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::footRightGreen] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::footRightBlue] = LEDRequest::on;
  }
  //opponent team kickoff -> off
  else
    ;
}

//MAKE_MODULE(LEDHandler, Behavior Control)

