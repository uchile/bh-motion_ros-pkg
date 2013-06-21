/**
* @file BH2011BehaviorControl.h
*
* Implementation of class BehaviorControl.
*
* @author Martin Lötzsch
* @author Matthias Jüngel
* @author Max Risler
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Tools/Xabsl/B-Human/BHXabslEngineExecutor.h"
#include "Modules/BehaviorControl/Symbols.h"
#include "Modules/BehaviorControl/BasicBehavior.h"
#include "Modules/Infrastructure/TeamDataProvider.h"
#include "Tools/Team.h"
#include "Tools/RingBufferWithSum.h"
//#include "Tools/MessageQueue/MessageQueue.h"

#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/Configuration/BehaviorConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Debugging/XabslInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SoundRequest.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"

#include "../../../../../src/UChProcess/UChBlackBoard.h"

/*MODULE(BH2011BehaviorControl)
  REQUIRES(FrameInfo)
  REQUIRES(BehaviorConfiguration)
  REQUIRES(FieldDimensions)
  REQUIRES(DamageConfiguration)
  REQUIRES(FallDownState)
  REQUIRES(HeadJointRequest)
  REQUIRES(KeyStates)
  REQUIRES(MotionInfo)
  REQUIRES(BallModel)
  REQUIRES(BallHypotheses)
  REQUIRES(BikeEngineOutput)
  REQUIRES(ArmContactModel)
  REQUIRES(GoalPercept)
  REQUIRES(RobotPose)
  REQUIRES(RobotPoseInfo)
  REQUIRES(RobotsModel)
  REQUIRES(RobotInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(GameInfo)
  REQUIRES(TeamMateData)
  REQUIRES(ObstacleModel)
  REQUIRES(FreePartOfOpponentGoalModel)
  REQUIRES(RobotDimensions)
  REQUIRES(GroundContactState)
  REQUIRES(TorsoMatrix)
  REQUIRES(RobotModel)
  PROVIDES_WITH_MODIFY(BehaviorControlOutput)
  REQUIRES(BehaviorControlOutput)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest)
  PROVIDES_WITH_MODIFY(HeadMotionRequest)
  PROVIDES_WITH_MODIFY(SoundRequest)
  PROVIDES_WITH_MODIFY(BehaviorLEDRequest)
END_MODULE*/


/**
* @class BH2011BehaviorControl
*
* A Behavior based on the xabsl::Engine that is used by B-Human
* for the RoboCup 2011.
*
* @author Martin Lötzsch
* @author Matthias Jüngel
* @author Max Risler
* @author Judith Müller
*/
class BH2011BehaviorControl : /*public BH2011BehaviorControlBase,*/ public BHXabslEngineExecutor
{
public:
  /**
  * Constructor.
  */
  BH2011BehaviorControl(UChBlackBoard* cognitionBlackboard);

  /** destructor */
  ~BH2011BehaviorControl();

  //PROCESS_WIDE_STORAGE_STATIC(BH2011BehaviorControl) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  /**
  * Is called for every incoming debug message.
  * @param message An interface to read the message from the queue
  * @return if the message was read
  */
  //static bool handleMessage(InMessage& message);

private:

  /** Registers symbols and basic behaviors at the engine */
  virtual void registerSymbolsAndBasicBehaviors();

  /** Is called if the engine could not be created */
  virtual void executeIfEngineCouldNotBeCreated();

  /**
  * Prints the main action that was generated by the execution of the engine to a string
  * @param buf the string where to print the action
  */
  virtual void printGeneratedMainActionToString(char* buf) const;

  /** a list of symbol definition classes */
  std::list<Symbols*> symbols;

  /** name of currently selected agent */
  std::string currentAgent;

  /** the output generated by the behavior control */
  BehaviorControlOutput behaviorControlOutput;

  /** updates the behavior control output */
public:
  void update(BehaviorControlOutput& behaviorControlOutput);

  /** updates the motion request by copying from behavior control output */
  void update(MotionRequest& motionRequest);

  /** updates the head motion request by copying from behavior control output */
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = blackboard->theBehaviorControlOutput.headMotionRequest;}

  /** updates the sound request by copying from behavior control output */
  void update(SoundRequest& soundRequest) {soundRequest = blackboard->theBehaviorControlOutput.soundRequest;}

  /** update the behavior ledRequest by cpoying from behavior control output */
  void update(BehaviorLEDRequest& behaviorLEDRequest);

  void init();

  /** synchronization ticks for coop states */
  float syncTicks;

  UChBlackBoard* blackboard;
};
