/**
* @file BH2011BehaviorControl.cpp
*
* Implementation of class BH2011BehaviorControl.
*
* @author Max Risler
* @author Judith Müller
*/

#include "BH2011BehaviorControl.h"
//#include "Tools/Debugging/ReleaseOptions.h"
#include "Modules/BehaviorControl/CommonSymbols/CommonBIKESymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/KeySymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/MathSymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/MotionSymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/SoundSymbols.h"
#include "Symbols/BH2011BallSymbols.h"
#include "Symbols/BH2011BIKESymbols.h"
#include "Symbols/BH2011FallDownSymbols.h"
#include "Symbols/BH2011FieldSymbols.h"
#include "Symbols/BH2011GameSymbols.h"
#include "Symbols/BH2011GoalSymbols.h"
#include "Symbols/BH2011HeadSymbols.h"
#include "Symbols/BH2011BehaviorLEDSymbols.h"
#include "Symbols/BH2011LocatorSymbols.h"
#include "Symbols/BH2011ObstacleSymbols.h"

#include <iostream>



//PROCESS_WIDE_STORAGE(BH2011BehaviorControl) BH2011BehaviorControl::theInstance = 0;

BH2011BehaviorControl::BH2011BehaviorControl(UChBlackBoard* cognitionBlackboard)
  :blackboard(cognitionBlackboard), BHXabslEngineExecutor("bh2011", cognitionBlackboard->theFrameInfo.time /*, syncTicks(15.f)*/)
{


  // Common Symbols
    symbols.push_back(new CommonBIKESymbols(behaviorControlOutput.motionRequest, blackboard->theBikeEngineOutput, blackboard->theMotionInfo));
    symbols.push_back(new KeySymbols(blackboard->theKeyStates));
    symbols.push_back(new MathSymbols());
    symbols.push_back(new MotionSymbols(behaviorControlOutput.motionRequest, blackboard->theMotionInfo, blackboard->theRobotInfo));
    symbols.push_back(new SoundSymbols(behaviorControlOutput.soundRequest));

    // Symbols
    symbols.push_back(new BH2011BallSymbols(blackboard->theRobotInfo, blackboard->theBallModel, blackboard->theBallHypotheses, blackboard->theFrameInfo, blackboard->theRobotPose, blackboard->theTeamMateData, blackboard->theFieldDimensions, blackboard->theRobotsModel, blackboard->theOwnTeamInfo, blackboard->theTorsoMatrix, blackboard->theRobotModel, blackboard->theRobotDimensions));
    symbols.push_back(new BH2011BIKESymbols(behaviorControlOutput.motionRequest, blackboard->theFieldDimensions, blackboard->theRobotPose));
    symbols.push_back(new BH2011FallDownSymbols(blackboard->theFallDownState));
    symbols.push_back(new BH2011FieldSymbols(blackboard->theFieldDimensions));
    symbols.push_back(new BH2011GameSymbols(behaviorControlOutput.behaviorData, behaviorControlOutput.robotInfo, behaviorControlOutput.ownTeamInfo, behaviorControlOutput.gameInfo, blackboard->theFrameInfo));
    symbols.push_back(new BH2011GoalSymbols(blackboard->theFreePartOfOpponentGoalModel, blackboard->theRobotPose, blackboard->theFallDownState, blackboard->theGoalPercept, blackboard->theGroundContactState, blackboard->theGameInfo, blackboard->theFrameInfo, blackboard->theFieldDimensions, blackboard->theDamageConfiguration));
    symbols.push_back(new BH2011HeadSymbols(behaviorControlOutput.headMotionRequest, blackboard->theHeadJointRequest, blackboard->theFieldDimensions, blackboard->theRobotPose));
    symbols.push_back(new BH2011BehaviorLEDSymbols(behaviorControlOutput.behaviorLEDRequest));
    symbols.push_back(new BH2011LocatorSymbols(blackboard->theRobotPose, blackboard->theRobotPoseInfo, blackboard->theFrameInfo, blackboard->theGroundContactState, blackboard->theFieldDimensions));
    symbols.push_back(new BH2011ObstacleSymbols(blackboard->theFrameInfo, blackboard->theObstacleModel, blackboard->theArmContactModel));

  BHXabslEngineExecutor::init();
  ASSERT(pEngine);

  if(!errorHandler.errorsOccurred)
    currentAgent = pEngine->getSelectedAgentName();
}

void BH2011BehaviorControl::init()
{
  // init symbols
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    (*i)->init();
}

BH2011BehaviorControl::~BH2011BehaviorControl()
{
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    delete *i;
}

void BH2011BehaviorControl::registerSymbolsAndBasicBehaviors()
{
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    (*i)->registerSymbols(*pEngine);
}

void BH2011BehaviorControl::executeIfEngineCouldNotBeCreated()
{
#ifdef TARGET_ROBOT
  ASSERT(false);
#endif
}

void BH2011BehaviorControl::printGeneratedMainActionToString(char* buf) const
{
  behaviorControlOutput.motionRequest.printOut(buf);
}

/*bool BH2011BehaviorControl::handleMessage(InMessage& message)
{
  if(theInstance)
    return theInstance->handleXabslMessage(message);
  else
    return false;
}*/

void BH2011BehaviorControl::update(BehaviorControlOutput& behaviorControlOutput)
{
  this->behaviorControlOutput.ownTeamInfo = blackboard->theOwnTeamInfo;
  this->behaviorControlOutput.robotInfo = blackboard->theRobotInfo;
  this->behaviorControlOutput.gameInfo = blackboard->theGameInfo;

  // update symbols
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    (*i)->update();

  // set agent
  if(currentAgent != blackboard->theBehaviorConfiguration.agent)
  {
    currentAgent = blackboard->theBehaviorConfiguration.agent;
    if(!setSelectedAgent(currentAgent.c_str()))
    {
      //OUTPUT(idText, text, "BH2011BehaviorControl: Unable to enable selected agent \"" << currentAgent << "\".");
    }
  }

  // execute the engine
  pEngine->setAgentPriority(blackboard->theRobotInfo.number);
  pEngine->setSynchronizationTicks(int(syncTicks));
  pEngine->prepareIncomingMessages();
  for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++)
    if(i != blackboard->theRobotInfo.number && blackboard->theFrameInfo.getTimeSince(blackboard->theTeamMateData.timeStamps[i]) < TeamDataProvider::networkTimeout
       && !blackboard->theTeamMateData.isPenalized[i] && blackboard->theFrameInfo.getTimeSince(blackboard->theTeamMateData.timeSinceLastGroundContact[i] < 2000))
      pEngine->processIncomingMessage(blackboard->theTeamMateData.behaviorData[i].xabslMessage);
  executeEngine();

  pEngine->generateOutgoingMessage(this->behaviorControlOutput.behaviorData.xabslMessage);

  behaviorControlOutput = this->behaviorControlOutput;
  behaviorControlOutput.behaviorData.teamColor = blackboard->theOwnTeamInfo.teamColor == TEAM_BLUE ? BehaviorData::blue : BehaviorData::red;
  //TEAM_OUTPUT_FAST(idTeamMateBehaviorData, bin, behaviorControlOutput.behaviorData);
}

void BH2011BehaviorControl::update(MotionRequest& motionRequest)
{
  motionRequest = blackboard->theBehaviorControlOutput.motionRequest;
  /*if(Global::getReleaseOptions().motionRequest)
  {
    TEAM_OUTPUT_FAST(idMotionRequest, bin, motionRequest);
  }*/
}

void BH2011BehaviorControl::update(BehaviorLEDRequest& behaviorLEDRequest)
{
  behaviorLEDRequest = blackboard->theBehaviorControlOutput.behaviorLEDRequest;
}

//MAKE_MODULE(BH2011BehaviorControl, Behavior Control)
