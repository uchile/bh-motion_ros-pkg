/**
* @file Modules/MotionControl/MotionCombinator.h
* This file declares a module that combines the motions created by the different modules.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</A>
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "../../Representations/Configuration/RobotDimensions.h"
#include "../../Representations/Infrastructure/JointData.h"
#include "../../Representations/Modeling/FallDownState.h"
#include "../../Representations/MotionControl/MotionSelection.h"
#include "../../Representations/MotionControl/SpecialActionsOutput.h"
#include "../../Representations/MotionControl/WalkingEngineOutput.h"
#include "../../Representations/MotionControl/WalkingEngineStandOutput.h"
#include "../../Representations/MotionControl/BikeEngineOutput.h"
#include "../../Representations/MotionControl/OdometryData.h"
#include "../../Representations/MotionControl/MotionInfo.h"
#include "../../Representations/MotionControl/HeadJointRequest.h"

/*MODULE(MotionCombinator)
  REQUIRES(RobotDimensions)
  REQUIRES(FilteredJointData)
  REQUIRES(FallDownState)
  REQUIRES(MotionSelection)
  REQUIRES(WalkingEngineOutput)
  REQUIRES(BikeEngineOutput)
  REQUIRES(SpecialActionsOutput)
  REQUIRES(WalkingEngineStandOutput)
  REQUIRES(HeadJointRequest)
  REQUIRES(HardnessSettings)
  PROVIDES_WITH_MODIFY(UnstableJointRequest)
  REQUIRES(UnstableJointRequest)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(OdometryData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(JointRequest)
  PROVIDES_WITH_MODIFY(MotionInfo)
END_MODULE*/

class MotionCombinator //: public MotionCombinatorBase
{
private:
  /* @class Parameters
  * The parameters for MotionCombinator
  */
  class Parameters : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(emergencyOffEnabled)
      STREAM(recoveryTime);
      STREAM_REGISTER_FINISH();
    }

  public:
    bool emergencyOffEnabled;
    unsigned int recoveryTime; /**< The number of frames to interpolate after emergency-stop. */
  };

  Parameters parameters; /**< The parameters of this module. */

  JointData lastJointData; /**< The measured joint angles the last time when not interpolating. */
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  Pose2D specialActionOdometry; /**< workaround for accumulating special action odometry*/
public:
  void update(OdometryData& odometryData, const FallDownState& theFallDownState);
  void update(UnstableJointRequest& jointRequest
              ,const SpecialActionsOutput& theSpecialActionsOutput
              ,const WalkingEngineOutput& theWalkingEngineOutput
              ,const BikeEngineOutput& theBikeEngineOutput
              ,const WalkingEngineStandOutput& theWalkingEngineStandOutput
              ,const HeadJointRequest& theHeadJointRequest
              ,const MotionSelection& theMotionSelection
              ,const FilteredJointData& theFilteredJointData
              ,const HardnessSettings& theHardnessSettings);
  void update(JointRequest& jointRequest
              ,const UnstableJointRequest& theUnstableJointRequest
              ,const FallDownState& theFallDownState
              ,const FilteredJointData& theFilteredJointData);
  void update(MotionInfo& motionInfo) {motionInfo = this->motionInfo;}
private:
  void saveFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest
                  ,const FallDownState& theFallDownState
                  ,const FilteredJointData& theFilteredJointData);

  unsigned int recoveryTime;

  bool headJawInSavePosition;
  bool headPitchInSavePosition;

#ifndef RELEASE
  OdometryData lastOdometryData;
  JointRequest lastJointRequest;
#endif

  /**
  * The method copies all joint angles from one joint request to another,
  * but only those that should not be ignored.
  * @param source The source joint request. All angles != JointData::ignore will be copied.
  * @param target The target joint request.
  */
  void copy(const JointRequest& source, JointRequest& target
             ,const HardnessSettings& theHardnessSettings) const;

  /**
  * The method interpolates between two joint requests.
  * @param from The first source joint request. This one is the starting point.
  * @param to The second source joint request. This one has to be reached over time.
  * @param fromRatio The ratio of "from" in the target joint request.
  * @param target The target joint request.
  * @param interpolateHardness Whether to interpolate hardness.
  */
  void interpolate(const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateHardness,const HardnessSettings& theHardnessSettings) const;

public:
  /**
  * Default constructor.
  */
  MotionCombinator();
};
