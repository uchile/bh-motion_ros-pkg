#include "UChMotionExecutor.h"
#include "Tools/Streams/InStreams.h"
#include <iostream>
#include <time.h>
#include <ctime>


UChMotionExecutor::UChMotionExecutor(process *processPointer_, UChBlackBoard *blackBoard_)

    //Modules
    : //motionConfigurationDataProvider(),

      processPointer(processPointer_),
      bb(blackBoard_),
      naoProvider(),
      jointFilter(),
      robotModelProvider(),
      groundContactDetector(),
      inertiaSensorInspector(),
      inertiaSensorCalibrator(),
      inertiaSensorFilter(),
      sensorFilter(),
      fallDownStateDetector(),
      torsoMatrixProvider(),
      headMotionEngine(),
      motionSelector(),
      blame()
{
    init();
}
UChMotionExecutor::~UChMotionExecutor()
{
    delete processPointer;
    delete bb;
}
void UChMotionExecutor::init()
{
    //TODO: Hacer que funcionen los streams
    //Esto BHuman lo hace en el modulo MotionConfigurationDataProvider
    InConfigMap jointStream("/home/nao/.config/naoqi/Data/Config/Robots/Default/jointCalibration.cfg");
    ASSERT(jointStream.exists());
    if(jointStream.exists())
    {
        std::cout << "Leyendo jointCalibration.cfg" << std::endl;
        jointStream >> bb->theJointCalibration;
    }
    else
    {
        std::cout << "Could not find jointCalibration.cfg" << std::endl;
    }

    InConfigMap sensorStream("/home/nao/.config/naoqi/Data/Config/Robots/Default/sensorCalibration.cfg");
    if(sensorStream.exists())
    {
        std::cout << "Leyendo sensorCalibration.cfg" << std::endl;
        sensorStream >> bb->theSensorCalibration;
    }
    else
    {
        std::cout << "Could not find sensorCalibration.cfg" << std::endl;

    }

    InConfigMap robotStream("/home/nao/.config/naoqi/Data/Config/Robots/Default/robotDimensions.cfg");
    if(robotStream.exists())
    {
        std::cout << "Leyendo robotDimensions.cfg" << std::endl;
        robotStream >> bb->theRobotDimensions;
    }
    else
    {
        std::cout << "Could not find robotDimensions.cfg" << std::endl;
    }

    InConfigMap massesStream("/home/nao/.config/naoqi/Data/Config/Robots/Default/masses.cfg");
    if(massesStream.exists())
    {
        std::cout << "Leyendo masses.cfg" << std::endl;
        massesStream >> bb->theMassCalibration;
    }
    else
    {
        std::cout << "Could not find masses.cfg" << std::endl;
    }

    InConfigMap hardnessStream("/home/nao/.config/naoqi/Data/Config/jointHardness.cfg");
    if(hardnessStream.exists())
    {
        std::cout << "Leyendo jointHardness.cfg" << std::endl;
        hardnessStream >> bb->theHardnessSettings;
    }
    else
    {
        std::cout << "Could not find jointHardness.cfg" << std::endl;
    }

    bb->theDamageConfiguration.useGroundContactDetection = true;
    bb->theDamageConfiguration.useGroundContactDetectionForLEDs = false;
    bb->theDamageConfiguration.useGroundContactDetectionForSafeStates = true;
    bb->theDamageConfiguration.useGroundContactDetectionForSensorCalibration = true;
}

void UChMotionExecutor::setSensors(float* sensors)
{
    for(unsigned int i=0; i<numOfSensorIds;i++)
    {
        naoProvider.sensors[i] = *(sensors);
        sensors++;
    }
}

void UChMotionExecutor::runModules()
{

    //Updates
    naoProvider.update(bb->theJointData,bb->theJointCalibration,bb->theSensorCalibration);
    naoProvider.update(bb->theKeyStates);
    naoProvider.update(bb->theSensorData);
    naoProvider.update(bb->theFrameInfo,bb->theJointData);
    naoProvider.update(bb->theBoardInfo);
    groundContactDetector.update(bb->theGroundContactState,bb->theSensorData,bb->theFrameInfo,bb->theMotionRequest,bb->theMotionInfo);
    inertiaSensorInspector.update(bb->theInspectedInertiaSensorData,bb->theSensorData);
    motionSelector.update(bb->theMotionSelection,bb->theFrameInfo,bb->theMotionRequest,bb->theDamageConfiguration,bb->theGroundContactState,bb->theWalkingEngineOutput,bb->theSpecialActionsOutput,bb->theBikeEngineOutput);
    jointFilter.update(bb->theFilteredJointData,bb->theJointData);
    headMotionEngine.update(bb->theHeadJointRequest,bb->theRobotDimensions,bb->theJointCalibration,bb->theHeadAngleRequest,bb->theFrameInfo,bb->theFilteredJointData);
    robotModelProvider.update(bb->theRobotModel,bb->theFilteredJointData,bb->theRobotDimensions,bb->theMassCalibration);
    specialActions.update(bb->theSpecialActionsOutput,bb->theMotionSelection,bb->theFilteredJointData,bb->theHardnessSettings,bb->theFrameInfo);
    inertiaSensorCalibrator.update(bb->theInertiaSensorData,bb->theFrameInfo,bb->theJointCalibration,bb->theInspectedInertiaSensorData,bb->theRobotModel,bb->theGroundContactState,bb->theMotionSelection,bb->theMotionInfo,bb->theWalkingEngineOutput,bb->theDamageConfiguration/*,theRobotInfo*/);
    inertiaSensorFilter.update(bb->theOrientationData,bb->theFrameInfo,bb->theInertiaSensorData,bb->theRobotModel,bb->theSensorData, bb->theMotionInfo,bb->theWalkingEngineOutput);
    sensorFilter.update(bb->theFilteredSensorData,bb->theSensorData,bb->theInertiaSensorData,bb->theOrientationData);
    torsoMatrixProvider.update(bb->theTorsoMatrix,bb->theFilteredSensorData,bb->theRobotModel,bb->theRobotDimensions,bb->theGroundContactState,bb->theDamageConfiguration);
    fallDownStateDetector.update(bb->theFallDownState,bb->theFilteredSensorData,bb->theInertiaSensorData,bb->theMotionInfo,bb->theFrameInfo);
    walkingEngine.update(bb->theWalkingEngineOutput,bb->theMassCalibration,bb->theRobotModel,bb->theHeadJointRequest,bb->theTorsoMatrix,bb->theFallDownState,bb->theMotionSelection,bb->theFrameInfo,bb->theRobotDimensions,bb->theMotionRequest,bb->theGroundContactState,bb->theDamageConfiguration);
    walkingEngine.update(bb->theWalkingEngineStandOutput);
    blame.update(bb->theBikeEngineOutput,bb->theMotionSelection,bb->theFrameInfo,bb->theRobotModel,bb->theFilteredJointData,bb->theFilteredSensorData,bb->theWalkingEngineStandOutput,bb->theMotionRequest,bb->theRobotDimensions,bb->theTorsoMatrix,bb->theJointCalibration,bb->theMassCalibration);
    motionCombinator.update(bb->theUnstableJointRequest,bb->theSpecialActionsOutput,bb->theWalkingEngineOutput,bb->theBikeEngineOutput,bb->theWalkingEngineStandOutput,bb->theHeadJointRequest,bb->theMotionSelection,bb->theFilteredJointData,bb->theHardnessSettings);
    motionCombinator.update(bb->theJointRequest,bb->theUnstableJointRequest,bb->theFallDownState,bb->theFilteredJointData);
    motionCombinator.update(bb->theMotionInfo);
    motionCombinator.update(bb->theOdometryData,bb->theFallDownState);
    naoProvider.send(bb->theJointRequest,bb->theJointCalibration);
}


void UChMotionExecutor::copyCognition2MotionBB(const bh_motion::Cognition2Motion::ConstPtr& cognition2Motion)
{

    //if (cognition2Motion == NULL) return;

//  ***** MOTION REQUEST ******

    bb->theMotionRequest.motion = MotionRequest::Motion(cognition2Motion->motionRequest.motion);

    //  ------ WalkingRequest -----
    bb->theMotionRequest.walkRequest.mode = WalkRequest::Mode(cognition2Motion->motionRequest.walkRequest.mode);
    bb->theMotionRequest.walkRequest.pedantic = cognition2Motion->motionRequest.walkRequest.pedantic;
    bb->theMotionRequest.walkRequest.dribbling = cognition2Motion->motionRequest.walkRequest.dribbling;
    bb->theMotionRequest.walkRequest.speed.rotation = cognition2Motion->motionRequest.walkRequest.speed.rotation;
    bb->theMotionRequest.walkRequest.speed.translation.x = cognition2Motion->motionRequest.walkRequest.speed.translation.x;
    bb->theMotionRequest.walkRequest.speed.translation.y = cognition2Motion->motionRequest.walkRequest.speed.translation.y;
    bb->theMotionRequest.walkRequest.target.rotation = cognition2Motion->motionRequest.walkRequest.target.rotation;
    bb->theMotionRequest.walkRequest.target.translation.x = cognition2Motion->motionRequest.walkRequest.target.translation.x;
    bb->theMotionRequest.walkRequest.target.translation.y = cognition2Motion->motionRequest.walkRequest.target.translation.y;


    //  ------ BikeRequest -----
    bb->theMotionRequest.bikeRequest.mirror =  cognition2Motion->motionRequest.bikeRequest.mirror;
    bb->theMotionRequest.bikeRequest.dynamical =  cognition2Motion->motionRequest.bikeRequest.dynamical;
    bb->theMotionRequest.bikeRequest.ballSpecial =  cognition2Motion->motionRequest.bikeRequest.ballSpecial;
    bb->theMotionRequest.bikeRequest.bMotionType = BikeRequest::BMotionID(cognition2Motion->motionRequest.bikeRequest.bMotionType);
    bb->theMotionRequest.bikeRequest.dynPoints.resize(2);
    bb->theMotionRequest.bikeRequest.dynPoints[0].limb = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].limb;
    bb->theMotionRequest.bikeRequest.dynPoints[0].phaseNumber = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].phaseNumber;
    bb->theMotionRequest.bikeRequest.dynPoints[0].duration = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].duration;
    bb->theMotionRequest.bikeRequest.dynPoints[0].translation.x = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].translation.x;
    bb->theMotionRequest.bikeRequest.dynPoints[0].translation.y = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].translation.y;
    bb->theMotionRequest.bikeRequest.dynPoints[0].translation.z = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].translation.z;
    bb->theMotionRequest.bikeRequest.dynPoints[0].angle.x = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].angle.x;
    bb->theMotionRequest.bikeRequest.dynPoints[0].angle.y = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].angle.y;
    bb->theMotionRequest.bikeRequest.dynPoints[0].angle.z = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].angle.z;
    bb->theMotionRequest.bikeRequest.dynPoints[0].odometryOffset.x = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].odometryOffset.x;
    bb->theMotionRequest.bikeRequest.dynPoints[0].odometryOffset.y = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].odometryOffset.y;
    bb->theMotionRequest.bikeRequest.dynPoints[0].odometryOffset.z = cognition2Motion->motionRequest.bikeRequest.dynPoints[0].odometryOffset.z;
    bb->theMotionRequest.bikeRequest.dynPoints[1].limb = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].limb;
    bb->theMotionRequest.bikeRequest.dynPoints[1].phaseNumber = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].phaseNumber;
    bb->theMotionRequest.bikeRequest.dynPoints[1].duration = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].duration;
    bb->theMotionRequest.bikeRequest.dynPoints[1].translation.x = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].translation.x ;
    bb->theMotionRequest.bikeRequest.dynPoints[1].translation.y = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].translation.y;
    bb->theMotionRequest.bikeRequest.dynPoints[1].translation.z = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].translation.z;
    bb->theMotionRequest.bikeRequest.dynPoints[1].angle.x = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].angle.x;
    bb->theMotionRequest.bikeRequest.dynPoints[1].angle.y = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].angle.y;
    bb->theMotionRequest.bikeRequest.dynPoints[1].angle.z = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].angle.z;
    bb->theMotionRequest.bikeRequest.dynPoints[1].odometryOffset.x = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].odometryOffset.x;
    bb->theMotionRequest.bikeRequest.dynPoints[1].odometryOffset.y = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].odometryOffset.y;
    bb->theMotionRequest.bikeRequest.dynPoints[1].odometryOffset.z = cognition2Motion->motionRequest.bikeRequest.dynPoints[1].odometryOffset.z;

    //  ------ SpecialActionRequest -----
    bb->theMotionRequest.specialActionRequest.mirror =  cognition2Motion->motionRequest.specialActionRequest.mirror;
    bb->theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID(cognition2Motion->motionRequest.specialActionRequest.specialAction);

    //  ------  DamageConfiguration -----
    bb->theDamageConfiguration.useGroundContactDetection = cognition2Motion->damageConfiguration.useGroundContactDetection;
    bb->theDamageConfiguration.useGroundContactDetectionForLEDs = cognition2Motion->damageConfiguration.useGroundContactDetectionForLEDs;
    bb->theDamageConfiguration.useGroundContactDetectionForSafeStates = cognition2Motion->damageConfiguration.useGroundContactDetectionForSafeStates;
    bb->theDamageConfiguration.useGroundContactDetectionForSensorCalibration = cognition2Motion->damageConfiguration.useGroundContactDetectionForSensorCalibration;

    // ----- HeadAngleRequest ------
    bb->theHeadAngleRequest.pan = cognition2Motion->headAngleRequest.pan;
    bb->theHeadAngleRequest.tilt = cognition2Motion->headAngleRequest.tilt;
    bb->theHeadAngleRequest.speed = cognition2Motion->headAngleRequest.speed;

    // ----- RobotInfo robotInfo ------
    bb->theRobotInfo.penalty = cognition2Motion->robotInfo.penalty;
    bb->theRobotInfo.secsTillUnpenalised = cognition2Motion->robotInfo.secsTillUnpenalised;

}

void UChMotionExecutor::copyMotion2CognitionBB(bh_motion::Motion2Cognition& motion2Cognition)
{

    motion2Cognition.bikeEngineOutput.odometryOffset.translation.x = bb->theBikeEngineOutput.odometryOffset.translation.x;
    motion2Cognition.bikeEngineOutput.odometryOffset.translation.y = bb->theBikeEngineOutput.odometryOffset.translation.y;
    motion2Cognition.bikeEngineOutput.odometryOffset.rotation = bb->theBikeEngineOutput.odometryOffset.rotation;
    motion2Cognition.bikeEngineOutput.isLeavingPossible = bb->theBikeEngineOutput.isLeavingPossible;
    motion2Cognition.bikeEngineOutput.executedBikeRequest.mirror = bb->theBikeEngineOutput.executedBikeRequest.mirror;
    motion2Cognition.bikeEngineOutput.executedBikeRequest.dynamical = bb->theBikeEngineOutput.executedBikeRequest.dynamical;
    motion2Cognition.bikeEngineOutput.executedBikeRequest.ballSpecial = bb->theBikeEngineOutput.executedBikeRequest.ballSpecial;
    motion2Cognition.bikeEngineOutput.executedBikeRequest.bMotionType = bb->theBikeEngineOutput.executedBikeRequest.bMotionType;
    //TODO: agregar los dynPoints
    motion2Cognition.cognitionFrameInfo.cycleTime = bb->theCognitionFrameInfo.cycleTime;
    motion2Cognition.cognitionFrameInfo.time = bb->theCognitionFrameInfo.time;
    motion2Cognition.fallDownState.state = bb->theFallDownState.state;
    motion2Cognition.fallDownState.direction = bb->theFallDownState.direction;
    motion2Cognition.fallDownState.sidewards = bb->theFallDownState.sidewards;
    for(unsigned int i=0; i<JointData::numOfJoints; ++i)
    {
        motion2Cognition.filteredJointData.angles[i] = bb->theFilteredJointData.angles[i];
    }
    motion2Cognition.filteredJointData.cycleTime = bb->theFilteredJointData.cycleTime;
    motion2Cognition.filteredJointData.timeStamp = bb->theFilteredJointData.timeStamp;
    for(unsigned int i=0; i<JointData::numOfJoints; ++i)
    {
        motion2Cognition.filteredJointDataPrev.angles[i] = bb->theFilteredJointDataPrev.angles[i];
    }
    motion2Cognition.filteredJointDataPrev.cycleTime = bb->theFilteredJointDataPrev.cycleTime;
    motion2Cognition.filteredJointDataPrev.timeStamp = bb->theFilteredJointDataPrev.timeStamp;
    for(unsigned int i=0; i<SensorData::numOfSensors; ++i)
    {
        motion2Cognition.filteredSensorData.data[i] = bb->theFilteredSensorData.data[i];
    }
    for(unsigned int i=0; i<JointData::numOfJoints; ++i)
    {
        motion2Cognition.filteredSensorData.currents[i] = bb->theFilteredSensorData.currents[i];
        motion2Cognition.filteredSensorData.temperatures[i] = bb->theFilteredSensorData.temperatures[i];
    }
    motion2Cognition.filteredSensorData.timeStamp = bb->theFilteredSensorData.timeStamp;
    motion2Cognition.filteredSensorData.usActuatorMode = bb->theFilteredSensorData.usActuatorMode;
    motion2Cognition.filteredSensorData.usTimeStamp = bb->theFilteredSensorData.usTimeStamp;
    motion2Cognition.groundContactState.contactSafe = bb->theGroundContactState.contactSafe;
    motion2Cognition.groundContactState.contact = bb->theGroundContactState.contact;
    motion2Cognition.groundContactState.noContactSafe = bb->theGroundContactState.noContactSafe;
    motion2Cognition.headJointRequest.pan = bb->theHeadJointRequest.pan;
    motion2Cognition.headJointRequest.tilt = bb->theHeadJointRequest.tilt;
    motion2Cognition.headJointRequest.reachable = bb->theHeadJointRequest.reachable;
    motion2Cognition.headJointRequest.moving = bb->theHeadJointRequest.moving;
    for(unsigned int i=0; i<JointData::numOfJoints; ++i)
    {
        motion2Cognition.jointRequest.jointData.angles[i] = bb->theJointRequest.angles[i];
    }
    motion2Cognition.jointRequest.jointData.cycleTime = bb->theJointRequest.cycleTime;
    motion2Cognition.jointRequest.jointData.timeStamp = bb->theJointRequest.timeStamp;
    for(unsigned int i=0; i<JointData::numOfJoints; ++i)
    {
        motion2Cognition.jointRequest.jointHardness.hardness[i] = bb->theJointRequest.jointHardness.hardness[i];
    }
    for(unsigned int i=0; i<5; ++i)
    {
        motion2Cognition.keyStates.pressed[i] = bb->theKeyStates.pressed[i];
    }
    motion2Cognition.motionInfo.motionRequest.bikeRequest.ballSpecial = bb->theMotionInfo.bikeRequest.ballSpecial;
    motion2Cognition.motionInfo.motionRequest.bikeRequest.dynamical= bb->theMotionInfo.bikeRequest.dynamical;
    motion2Cognition.motionInfo.motionRequest.bikeRequest.mirror= bb->theMotionInfo.bikeRequest.mirror;
    motion2Cognition.motionInfo.motionRequest.bikeRequest.bMotionType = bb->theMotionInfo.bikeRequest.bMotionType;
    motion2Cognition.motionInfo.motionRequest.specialActionRequest.mirror = bb->theMotionInfo.specialActionRequest.mirror;
    motion2Cognition.motionInfo.motionRequest.specialActionRequest.specialAction= bb->theMotionInfo.specialActionRequest.specialAction;
    motion2Cognition.motionInfo.motionRequest.walkRequest.dribbling = bb->theMotionInfo.walkRequest.dribbling;
    motion2Cognition.motionInfo.motionRequest.walkRequest.mode = bb->theMotionInfo.walkRequest.mode;
    motion2Cognition.motionInfo.motionRequest.walkRequest.pedantic = bb->theMotionInfo.walkRequest.pedantic;
    motion2Cognition.motionInfo.motionRequest.walkRequest.speed.translation.x = bb->theMotionInfo.walkRequest.speed.translation.x;
    motion2Cognition.motionInfo.motionRequest.walkRequest.speed.translation.y = bb->theMotionInfo.walkRequest.speed.translation.y;
    motion2Cognition.motionInfo.motionRequest.walkRequest.speed.rotation = bb->theMotionInfo.walkRequest.speed.rotation;
    motion2Cognition.motionInfo.motionRequest.walkRequest.target.translation.x = bb->theMotionInfo.walkRequest.target.translation.x;
    motion2Cognition.motionInfo.motionRequest.walkRequest.target.translation.y = bb->theMotionInfo.walkRequest.target.translation.y;
    motion2Cognition.motionInfo.motionRequest.walkRequest.target.rotation = bb->theMotionInfo.walkRequest.target.rotation;
    motion2Cognition.motionInfo.motionRequest.walkRequest.kickType = bb->theMotionInfo.walkRequest.kickType;
    motion2Cognition.motionInfo.motionRequest.walkRequest.kickBallPosition.x = bb->theMotionInfo.walkRequest.kickBallPosition.x;
    motion2Cognition.motionInfo.motionRequest.walkRequest.kickBallPosition.y = bb->theMotionInfo.walkRequest.kickBallPosition.y;
    motion2Cognition.motionInfo.motionRequest.walkRequest.kickTarget.x = bb->theMotionInfo.walkRequest.kickTarget.x;
    motion2Cognition.motionInfo.motionRequest.walkRequest.kickTarget.y = bb->theMotionInfo.walkRequest.kickTarget.y;
    motion2Cognition.motionInfo.isMotionStable = bb->theMotionInfo.isMotionStable;
    motion2Cognition.motionInfo.upcomingOdometryOffset.translation.x = bb->theMotionInfo.upcomingOdometryOffset.translation.x;
    motion2Cognition.motionInfo.upcomingOdometryOffset.translation.y = bb->theMotionInfo.upcomingOdometryOffset.translation.y;
    motion2Cognition.motionInfo.upcomingOdometryOffset.rotation = bb->theMotionInfo.upcomingOdometryOffset.rotation;
    motion2Cognition.motionInfo.upcomingOdometryOffsetValid = bb->theMotionInfo.upcomingOdometryOffsetValid;
    motion2Cognition.motionRobotHealth.motionFrameRate = bb->theMotionRobotHealth.motionFrameRate;
    motion2Cognition.odometryData.pose2D.translation.x = bb->theOdometryData.translation.x;
    motion2Cognition.odometryData.pose2D.translation.y = bb->theOdometryData.translation.y;
    motion2Cognition.odometryData.pose2D.rotation = bb->theOdometryData.rotation;
    motion2Cognition.robotModel.centerOfMass.x = bb->theRobotModel.centerOfMass.x;
    motion2Cognition.robotModel.centerOfMass.y = bb->theRobotModel.centerOfMass.y;
    motion2Cognition.robotModel.centerOfMass.z = bb->theRobotModel.centerOfMass.z;
    motion2Cognition.robotModel.totalMass = bb->theRobotModel.totalMass;
    motion2Cognition.robotModel.centerOfMass.x = bb->theRobotModel.centerOfMass.x;
    for(unsigned int i=0; i<23; ++i)
    {
        motion2Cognition.robotModel.limbs[i].translation.x = bb->theRobotModel.limbs[i].translation.x;
        motion2Cognition.robotModel.limbs[i].translation.y = bb->theRobotModel.limbs[i].translation.y;
        motion2Cognition.robotModel.limbs[i].translation.z = bb->theRobotModel.limbs[i].translation.z;
        motion2Cognition.robotModel.limbs[i].rotation.c0.x = bb->theRobotModel.limbs[i].rotation.c0.x;
        motion2Cognition.robotModel.limbs[i].rotation.c0.y = bb->theRobotModel.limbs[i].rotation.c0.y;
        motion2Cognition.robotModel.limbs[i].rotation.c0.z = bb->theRobotModel.limbs[i].rotation.c0.z;
        motion2Cognition.robotModel.limbs[i].rotation.c1.x = bb->theRobotModel.limbs[i].rotation.c1.x;
        motion2Cognition.robotModel.limbs[i].rotation.c1.y = bb->theRobotModel.limbs[i].rotation.c1.y;
        motion2Cognition.robotModel.limbs[i].rotation.c1.z = bb->theRobotModel.limbs[i].rotation.c1.z;
        motion2Cognition.robotModel.limbs[i].rotation.c2.x = bb->theRobotModel.limbs[i].rotation.c2.x;
        motion2Cognition.robotModel.limbs[i].rotation.c2.y = bb->theRobotModel.limbs[i].rotation.c2.y;
        motion2Cognition.robotModel.limbs[i].rotation.c2.z = bb->theRobotModel.limbs[i].rotation.c2.z;
    }
    motion2Cognition.torsoMatrix.isValid = bb->theTorsoMatrix.isValid;
    motion2Cognition.torsoMatrix.offset.translation.x = bb->theTorsoMatrix.translation.x;
    motion2Cognition.torsoMatrix.offset.translation.y = bb->theTorsoMatrix.translation.y;
    motion2Cognition.torsoMatrix.offset.translation.z = bb->theTorsoMatrix.translation.z;
    motion2Cognition.torsoMatrix.offset.rotation.c0.x = bb->theTorsoMatrix.rotation.c0.x;
    motion2Cognition.torsoMatrix.offset.rotation.c0.y = bb->theTorsoMatrix.rotation.c0.y;
    motion2Cognition.torsoMatrix.offset.rotation.c0.z = bb->theTorsoMatrix.rotation.c0.z;
    motion2Cognition.torsoMatrix.offset.rotation.c1.x = bb->theTorsoMatrix.rotation.c1.x;
    motion2Cognition.torsoMatrix.offset.rotation.c1.y = bb->theTorsoMatrix.rotation.c1.y;
    motion2Cognition.torsoMatrix.offset.rotation.c1.z = bb->theTorsoMatrix.rotation.c1.z;
    motion2Cognition.torsoMatrix.offset.rotation.c2.x = bb->theTorsoMatrix.rotation.c2.x;
    motion2Cognition.torsoMatrix.offset.rotation.c2.y = bb->theTorsoMatrix.rotation.c2.y;
    motion2Cognition.torsoMatrix.offset.rotation.c2.z = bb->theTorsoMatrix.rotation.c2.z;


}

