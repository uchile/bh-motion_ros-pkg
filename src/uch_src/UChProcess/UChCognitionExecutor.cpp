#include "UChCognitionExecutor.h"
#include <iostream>
#include <time.h>
#include <ctime>


UChCognitionExecutor::UChCognitionExecutor(UChBlackBoard *blackBoard_)
    : bb(blackBoard_),
      whiteCorrection(),
      //cognitionLogDataProviderBase(),
      cognitionConfigurationDataProvider(),
      cameraCalibrator(bb),
      cameraProvider(),
      gameDataProvider(),
      robotCameraMatrixProvider(),
      coordinateSystemProvider(),
      bodyContourProvider(),
      armContactModelProvider(),
      robotPoseValidator(bb),
      robotLocatorUKF(),
      usObstacleGridProvider(),
      ballLocator(bb),
      freePartOfOpponentGoalProvider(),
      obstacleCombinator(),
      ballPredictor(),
      teamDataProvider(),
      goalPerceptor(),
      regionizer(),
      teamMarkerPerceptor(),
      regionAnalyzer(),
      robotPerceptor(bb->theImage, bb->theColorTable64),
      ballPerceptor(),
      linePerceptor(),
      bh2011BehaviorControl(bb),
      cameraControlEngine(),
      ledHandler(),
      combinedWorldModelProvider(),
      fieldCoverageProvider(bb),
      globalFieldCoverageProvider(bb),
      selfLocator(bb),
      groundTruthProvider(bb),
      groundTruthEvaluator(bb)
{
}
UChCognitionExecutor::~UChCognitionExecutor()
{

    delete bb;
}

void UChCognitionExecutor::init()
{
}

void UChCognitionExecutor::runModules()
{
    if(CameraProvider::isFrameDataComplete())
    {
    //    ORDEN DE UPDATES
        //behaviorConfiguration
        cognitionConfigurationDataProvider.update(bb->theBehaviorConfiguration);
        //cameraCalibration
        cognitionConfigurationDataProvider.update(bb->theCameraCalibration);
        //CameraSettings
        cognitionConfigurationDataProvider.update(bb->theCameraSettings);
        //ColorTable64
        cognitionConfigurationDataProvider.update(bb->theColorTable64);
        //DamageConfiguration
        cognitionConfigurationDataProvider.update(bb->theDamageConfiguration);
        //FieldDimensions
        cognitionConfigurationDataProvider.update(bb->theFieldDimensions);
        //Image
        cameraProvider.update(bb->theImage, bb->theCameraSettings);
        //ImageInfo
        cameraProvider.update(bb->theImageInfo, bb->theImageRequest);
        //RobotDimensions
        cognitionConfigurationDataProvider.update(bb->theRobotDimensions);
        //CameraInfo
        cameraProvider.update(bb->theCameraInfo);
        //cognitionFrameInfo
        cameraProvider.update(bb->theCognitionFrameInfo, bb->theImage);
        //frameInfo
        cameraProvider.update(bb->theFrameInfo, bb->theImage);
        //GroundTruthBallModel
        teamDataProvider.update(bb->theGroundTruthBallModel);
        //GroundTruthRobotPose
        teamDataProvider.update(bb->theGroundTruthRobotPose);
        //CameraMatrix
        robotCameraMatrixProvider.update(bb->theRobotCameraMatrix, bb->theRobotDimensions, bb->theFilteredJointData, bb->theCameraCalibration, bb->theImageInfo);
        //CameraMatrixOther
        robotCameraMatrixProvider.update(bb->theRobotCameraMatrixOther, bb->theRobotDimensions, bb->theFilteredJointData, bb->theCameraCalibration, bb->theImageInfo);
        //CameraMatrixPrev
        robotCameraMatrixProvider.update(bb->theRobotCameraMatrixPrev, bb->theRobotDimensions, bb->theFilteredJointDataPrev, bb->theCameraCalibration, bb->theImageInfo);
        //RobotInfo
        gameDataProvider.update(bb->theRobotInfo, bb->theFrameInfo, bb->theBehaviorControlOutput);
        // ArmContactModel
        armContactModelProvider.update(bb->theArmContactModel, bb->theJointRequest, bb->theFallDownState, bb->theMotionInfo, bb->theFilteredJointData, bb->theFrameInfo);
        //CameraMatrix
        cameraMatrixProvider.update(bb->theCameraMatrix, bb->theTorsoMatrix, bb->theRobotCameraMatrix, bb->theCameraCalibration, bb->theMotionInfo, bb->theFallDownState, bb->theFrameInfo, bb->theFilteredJointData, bb->theRobotInfo );
        //CameraMatrixOther
        cameraMatrixProvider.update(bb->theCameraMatrixOther, bb->theTorsoMatrix, bb->theRobotCameraMatrixOther, bb->theCameraCalibration, bb->theMotionInfo, bb->theFallDownState, bb->theFrameInfo, bb->theFilteredJointData, bb->theRobotInfo);
        //CameraMatrixPrev
        cameraMatrixProvider.update(bb->theCameraMatrixPrev, bb->theTorsoMatrix, bb->theRobotCameraMatrixPrev, bb->theCameraCalibration, bb->theMotionInfo, bb->theFallDownState, bb->theFrameInfo, bb->theFilteredJointDataPrev, bb->theRobotInfo);
        //GameInfo
        gameDataProvider.update(bb->theGameInfo, bb->theFrameInfo, bb->theBehaviorControlOutput);
        //ImageCoordinateSystem
        coordinateSystemProvider.update(bb->theImageCoordinateSystem, bb->theCameraMatrix, bb->theCameraInfo, bb->theImageInfo, bb->theCameraMatrixOther, bb->theCameraMatrixPrev, bb->theRobotDimensions, bb->theFilteredJointData, bb->theFilteredJointDataPrev, bb->theFrameInfo, bb->theImage);
        //OpponentTeamInfo
        gameDataProvider.update(bb->theOpponentTeamInfo);
        //OwnTeamInfo
        gameDataProvider.update(bb->theOwnTeamInfo, bb->theFrameInfo, bb->theBehaviorControlOutput, bb->theFieldDimensions);
        // TeamMateData
        teamDataProvider.update(bb->theTeamMateData, bb->theFrameInfo, bb->theMotionRequest, bb->theMotionInfo, bb->theRobotInfo);
        // BodyContour
        bodyContourProvider.update(bb->theBodyContour, bb->theCameraInfo, bb->theRobotModel, bb->theRobotCameraMatrix, bb->theImageCoordinateSystem);
        // GoalPercept
        goalPerceptor.update(bb->theGoalPercept, bb->theCameraMatrix, bb->theImageCoordinateSystem, bb->theCameraInfo, bb->theImage, bb->theColorTable64, bb->theFieldDimensions, bb->theOwnTeamInfo, bb->theFrameInfo);
        //RegionPercept
        regionizer.update(bb->theRegionPercept, bb->theImage, bb->theColorTable64, bb->theImageCoordinateSystem, bb->theFrameInfo, bb->theBallHypotheses, bb->theBodyContour);
        // TeamMarkerSpots
        teamMarkerPerceptor.update(bb->theTeamMarkerSpots, bb->theRegionPercept, bb->theImage);
        // BallSpots - LineSpots
        regionAnalyzer.update(bb->theBallSpots, bb->theCameraMatrix, bb->theRegionPercept);
        regionAnalyzer.update(bb->theLineSpots, bb->theCameraMatrix, bb->theRegionPercept);
        //RobotPercept
        robotPerceptor.update(bb->theRobotPercept, bb->theTeamMarkerSpots, bb->theImage);
        //BallPercept
        ballPerceptor.update(bb->theBallPercept, bb->theBallSpots, bb->theImageCoordinateSystem, bb->theCameraInfo, bb->theCameraMatrix, bb->theFieldDimensions, bb->theImage);
        //LinePercept
        linePerceptor.update(bb->theLinePercept, bb->theFieldDimensions, bb->theCameraMatrix, bb->theCameraInfo, bb->theImageCoordinateSystem, bb->theLineSpots, bb->theFrameInfo);

        selfLocator.update(bb->thePotentialRobotPose);
        //TODO: agregar Modulo RobotHealthProvider
        selfLocator.update(bb->theRobotPose);
        robotPoseValidator.update(bb->theRobotPose);
        robotPoseValidator.update(bb->theRobotPoseInfo);
        robotLocatorUKF.update(bb->theRobotsModel, bb->theGameInfo, bb->theOdometryData, bb->theRobotPercept, bb->theImageCoordinateSystem, bb->theCameraMatrix, bb->theCameraInfo, bb->theFrameInfo, bb->theImage, bb->theFieldDimensions, bb->theRobotPose);
        usObstacleGridProvider.update(bb->theUSObstacleGrid, bb->theOdometryData, bb->theGameInfo, bb->theRobotInfo, bb->theMotionRequest, bb->theFrameInfo, bb->theRobotPose, bb->theFilteredSensorData);
        ballLocator.update(bb->theBallModel);

        freePartOfOpponentGoalProvider.update(bb->theFreePartOfOpponentGoalModel, bb->theFieldDimensions, bb->theRobotsModel, bb->theRobotPose, bb->theBallModel);
        //TODO: agregar GroundTruthResult
        obstacleCombinator.update(bb->theObstacleModel, bb->theUSObstacleGrid, bb->theRobotsModel, bb->theArmContactModel, bb->theOdometryData);
        ballPredictor.update(bb->theBallHypotheses, bb->theBallModel, bb->theCameraMatrix, bb->theCameraInfo, bb->theImageCoordinateSystem, bb->theFrameInfo);
        bh2011BehaviorControl.update(bb->theBehaviorControlOutput);
        bh2011BehaviorControl.update(bb->theBehaviorLEDRequest);
        combinedWorldModelProvider.update(bb->theCombinedWorldModel, bb->theRobotInfo, bb->theGroundContactState, bb->theRobotPose, bb->theFieldDimensions, bb->theTeamMateData, bb->theBallHypotheses, bb->theFrameInfo, bb->theBallModel, bb->theDamageConfiguration, bb->theOwnTeamInfo, bb->theCameraMatrix, bb->theFallDownState, bb->theObstacleModel, bb->theRobotsModel);
        fieldCoverageProvider.update(bb->theFieldCoverage);
        globalFieldCoverageProvider.update(bb->theGlobalFieldCoverage );
        bh2011BehaviorControl.update(bb->theHeadMotionRequest);
        ledHandler.update(bb->theLEDRequest,bb->theFrameInfo,bb->theBehaviorLEDRequest,bb->theFilteredSensorData,bb->theTeamMateData,bb->theGameInfo,bb->theBallModel,bb->theGroundContactState,bb->theOwnTeamInfo,bb->theBehaviorControlOutput,bb->theRobotInfo,bb->theGoalPercept);
        bh2011BehaviorControl.update(bb->theMotionRequest);
        bh2011BehaviorControl.update(bb->theSoundRequest);
        cameraControlEngine.update(bb->theHeadAngleRequest,bb->theImageInfo,bb->theHeadMotionRequest,bb->theFilteredJointData,bb->theCameraInfo,bb->theTorsoMatrix,bb->theRobotDimensions,bb->theRobotCameraMatrix,bb->theRobotCameraMatrixOther);
        cameraControlEngine.update(bb->theImageRequest);


    }
    cameraProvider.waitForFrameData();

}

void UChCognitionExecutor::copyMotion2CognitionBB(const bh_motion::Motion2Cognition::ConstPtr& motion2Cognition)
{
    //if (motion2Cognition == NULL) return;

    bb->theBikeEngineOutput.odometryOffset.translation.x = motion2Cognition->bikeEngineOutput.odometryOffset.translation.x;
    bb->theBikeEngineOutput.odometryOffset.translation.y = motion2Cognition->bikeEngineOutput.odometryOffset.translation.y;
    bb->theBikeEngineOutput.odometryOffset.rotation = motion2Cognition->bikeEngineOutput.odometryOffset.rotation;
    bb->theBikeEngineOutput.isLeavingPossible = motion2Cognition->bikeEngineOutput.isLeavingPossible;
    bb->theBikeEngineOutput.executedBikeRequest.mirror = motion2Cognition->bikeEngineOutput.executedBikeRequest.mirror;
    bb->theBikeEngineOutput.executedBikeRequest.dynamical = motion2Cognition->bikeEngineOutput.executedBikeRequest.dynamical;
    bb->theBikeEngineOutput.executedBikeRequest.ballSpecial = motion2Cognition->bikeEngineOutput.executedBikeRequest.ballSpecial;
    bb->theBikeEngineOutput.executedBikeRequest.bMotionType = BikeRequest::BMotionID(motion2Cognition->bikeEngineOutput.executedBikeRequest.bMotionType);
               //TODO: agregar los dynPoints
    bb->theCognitionFrameInfo.cycleTime = motion2Cognition->cognitionFrameInfo.cycleTime;
    bb->theCognitionFrameInfo.time = motion2Cognition->cognitionFrameInfo.time;
    bb->theFallDownState.state = FallDownState::State(motion2Cognition->fallDownState.state);
    bb->theFallDownState.direction =  FallDownState::Direction(motion2Cognition->fallDownState.direction);
    bb->theFallDownState.sidewards =  FallDownState::Sidestate(motion2Cognition->fallDownState.sidewards);
    for(unsigned int i = 0; i<JointData::numOfJoints; ++i)
    {
        bb->theFilteredJointData.angles[i] = motion2Cognition->filteredJointData.angles[i];
    }
    bb->theFilteredJointData.cycleTime = motion2Cognition->filteredJointData.cycleTime;
    bb->theFilteredJointData.timeStamp =  motion2Cognition->filteredJointData.timeStamp;
    for(unsigned int i = 0; i<JointData::numOfJoints; ++i)
    {
        bb->theFilteredJointDataPrev.angles[i] = motion2Cognition->filteredJointDataPrev.angles[i];
    }
    bb->theFilteredJointDataPrev.cycleTime = motion2Cognition->filteredJointDataPrev.cycleTime;
    bb->theFilteredJointDataPrev.timeStamp = motion2Cognition->filteredJointDataPrev.timeStamp;
    for(unsigned int i=0; i<SensorData::numOfSensors; ++i)
    {
        bb->theFilteredSensorData.data[i] = motion2Cognition->filteredSensorData.data[i];
    }
    for(unsigned int i = 0; i<JointData::numOfJoints; ++i)
    {
        bb->theFilteredSensorData.currents[i] = motion2Cognition->filteredSensorData.currents[i];
        bb->theFilteredSensorData.temperatures[i] = motion2Cognition->filteredSensorData.temperatures[i];
    }
    bb->theFilteredSensorData.timeStamp = motion2Cognition->filteredSensorData.timeStamp;
    bb->theFilteredSensorData.usActuatorMode = SensorData::UsActuatorMode(motion2Cognition->filteredSensorData.usActuatorMode);
    bb->theFilteredSensorData.usTimeStamp = motion2Cognition->filteredSensorData.usTimeStamp;
    bb->theGroundContactState.contactSafe = motion2Cognition->groundContactState.contactSafe;
    bb->theGroundContactState.contact = motion2Cognition->groundContactState.contact;
    bb->theGroundContactState.noContactSafe = motion2Cognition->groundContactState.noContactSafe;
    bb->theHeadJointRequest.pan = motion2Cognition->headJointRequest.pan;
    bb->theHeadJointRequest.tilt = motion2Cognition->headJointRequest.tilt;
    bb->theHeadJointRequest.reachable = motion2Cognition->headJointRequest.reachable;
    bb->theHeadJointRequest.moving = motion2Cognition->headJointRequest.moving;
    for(unsigned int i = 0; i<JointData::numOfJoints; ++i)
    {
        bb->theJointRequest.angles[i] = motion2Cognition->jointRequest.jointData.angles[i];
    }
    bb->theJointRequest.cycleTime = motion2Cognition->jointRequest.jointData.cycleTime;
    bb->theJointRequest.timeStamp = motion2Cognition->jointRequest.jointData.timeStamp;
    for(unsigned int i = 0; i<JointData::numOfJoints; ++i)
    {
        bb->theJointRequest.jointHardness.hardness[i] = motion2Cognition->jointRequest.jointHardness.hardness[i];
    }
     for(unsigned int i = 0; i<5; ++i)
    {
        bb->theKeyStates.pressed[i] = motion2Cognition->keyStates.pressed[i];
    }
    bb->theMotionInfo.bikeRequest.ballSpecial = motion2Cognition->motionInfo.motionRequest.bikeRequest.ballSpecial;
    bb->theMotionInfo.bikeRequest.dynamical = motion2Cognition->motionInfo.motionRequest.bikeRequest.dynamical;
    bb->theMotionInfo.bikeRequest.mirror = motion2Cognition->motionInfo.motionRequest.bikeRequest.mirror;
    bb->theMotionInfo.bikeRequest.bMotionType = BikeRequest::BMotionID(motion2Cognition->motionInfo.motionRequest.bikeRequest.bMotionType);
    bb->theMotionInfo.specialActionRequest.mirror = motion2Cognition->motionInfo.motionRequest.specialActionRequest.mirror;
    bb->theMotionInfo.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID(motion2Cognition->motionInfo.motionRequest.specialActionRequest.specialAction);
    bb->theMotionInfo.walkRequest.dribbling = motion2Cognition->motionInfo.motionRequest.walkRequest.dribbling;
    bb->theMotionInfo.walkRequest.mode = WalkRequest::Mode(motion2Cognition->motionInfo.motionRequest.walkRequest.mode);
    bb->theMotionInfo.walkRequest.pedantic = motion2Cognition->motionInfo.motionRequest.walkRequest.pedantic;
    bb->theMotionInfo.walkRequest.speed.translation.x = motion2Cognition->motionInfo.motionRequest.walkRequest.speed.translation.x;
    bb->theMotionInfo.walkRequest.speed.translation.y = motion2Cognition->motionInfo.motionRequest.walkRequest.speed.translation.y;
    bb->theMotionInfo.walkRequest.speed.rotation = motion2Cognition->motionInfo.motionRequest.walkRequest.speed.rotation;
    bb->theMotionInfo.walkRequest.target.translation.x = motion2Cognition->motionInfo.motionRequest.walkRequest.target.translation.x;
    bb->theMotionInfo.walkRequest.target.translation.y = motion2Cognition->motionInfo.motionRequest.walkRequest.target.translation.y;
    bb->theMotionInfo.walkRequest.target.rotation = motion2Cognition->motionInfo.motionRequest.walkRequest.target.rotation;
    bb->theMotionInfo.walkRequest.kickType = WalkRequest::KickType(motion2Cognition->motionInfo.motionRequest.walkRequest.kickType);
    bb->theMotionInfo.walkRequest.kickBallPosition.x = motion2Cognition->motionInfo.motionRequest.walkRequest.kickBallPosition.x;
    bb->theMotionInfo.walkRequest.kickBallPosition.y = motion2Cognition->motionInfo.motionRequest.walkRequest.kickBallPosition.y;
    bb->theMotionInfo.walkRequest.kickTarget.x = motion2Cognition->motionInfo.motionRequest.walkRequest.kickTarget.x;
    bb->theMotionInfo.walkRequest.kickTarget.y = motion2Cognition->motionInfo.motionRequest.walkRequest.kickTarget.y;
    bb->theMotionInfo.isMotionStable = motion2Cognition->motionInfo.isMotionStable;
    bb->theMotionInfo.upcomingOdometryOffset.translation.x = motion2Cognition->motionInfo.upcomingOdometryOffset.translation.x;
    bb->theMotionInfo.upcomingOdometryOffset.translation.y = motion2Cognition->motionInfo.upcomingOdometryOffset.translation.y;
    bb->theMotionInfo.upcomingOdometryOffset.rotation = motion2Cognition->motionInfo.upcomingOdometryOffset.rotation;
    bb->theMotionInfo.upcomingOdometryOffsetValid = motion2Cognition->motionInfo.upcomingOdometryOffsetValid;
    bb->theMotionRobotHealth.motionFrameRate = motion2Cognition->motionRobotHealth.motionFrameRate;
    bb->theOdometryData.translation.x = motion2Cognition->odometryData.pose2D.translation.x;
    bb->theOdometryData.translation.y = motion2Cognition->odometryData.pose2D.translation.y;
    bb->theOdometryData.rotation = motion2Cognition->odometryData.pose2D.rotation;
    bb->theRobotModel.centerOfMass.x = motion2Cognition->robotModel.centerOfMass.x;
    bb->theRobotModel.centerOfMass.y = motion2Cognition->robotModel.centerOfMass.y;
    bb->theRobotModel.centerOfMass.z = motion2Cognition->robotModel.centerOfMass.z;
    bb->theRobotModel.totalMass = motion2Cognition->robotModel.totalMass;
    bb->theRobotModel.centerOfMass.x = motion2Cognition->robotModel.centerOfMass.x;
    for(unsigned int i = 0; i<23; ++i)
    {
        bb->theRobotModel.limbs[i].translation.x = motion2Cognition->robotModel.limbs[i].translation.x;
        bb->theRobotModel.limbs[i].translation.y = motion2Cognition->robotModel.limbs[i].translation.y;
        bb->theRobotModel.limbs[i].translation.z = motion2Cognition->robotModel.limbs[i].translation.z;
        bb->theRobotModel.limbs[i].rotation.c0.x = motion2Cognition->robotModel.limbs[i].rotation.c0.x;
        bb->theRobotModel.limbs[i].rotation.c0.y = motion2Cognition->robotModel.limbs[i].rotation.c0.y;
        bb->theRobotModel.limbs[i].rotation.c0.z = motion2Cognition->robotModel.limbs[i].rotation.c0.z;
        bb->theRobotModel.limbs[i].rotation.c1.x = motion2Cognition->robotModel.limbs[i].rotation.c1.x;
        bb->theRobotModel.limbs[i].rotation.c1.y = motion2Cognition->robotModel.limbs[i].rotation.c1.y;
        bb->theRobotModel.limbs[i].rotation.c1.z = motion2Cognition->robotModel.limbs[i].rotation.c1.z;
        bb->theRobotModel.limbs[i].rotation.c2.x = motion2Cognition->robotModel.limbs[i].rotation.c2.x;
        bb->theRobotModel.limbs[i].rotation.c2.y = motion2Cognition->robotModel.limbs[i].rotation.c2.y;
        bb->theRobotModel.limbs[i].rotation.c2.z = motion2Cognition->robotModel.limbs[i].rotation.c2.z;
    }
    bb->theTorsoMatrix.isValid = motion2Cognition->torsoMatrix.isValid;
    bb->theTorsoMatrix.translation.x = motion2Cognition->torsoMatrix.offset.translation.x;
    bb->theTorsoMatrix.translation.y = motion2Cognition->torsoMatrix.offset.translation.y;
    bb->theTorsoMatrix.translation.z = motion2Cognition->torsoMatrix.offset.translation.z;
    bb->theTorsoMatrix.rotation.c0.x = motion2Cognition->torsoMatrix.offset.rotation.c0.x;
    bb->theTorsoMatrix.rotation.c0.y = motion2Cognition->torsoMatrix.offset.rotation.c0.y;
    bb->theTorsoMatrix.rotation.c0.z = motion2Cognition->torsoMatrix.offset.rotation.c0.z;
    bb->theTorsoMatrix.rotation.c1.x = motion2Cognition->torsoMatrix.offset.rotation.c1.x;
    bb->theTorsoMatrix.rotation.c1.y = motion2Cognition->torsoMatrix.offset.rotation.c1.y;
    bb->theTorsoMatrix.rotation.c1.z = motion2Cognition->torsoMatrix.offset.rotation.c1.z;
    bb->theTorsoMatrix.rotation.c2.x = motion2Cognition->torsoMatrix.offset.rotation.c2.x;
    bb->theTorsoMatrix.rotation.c2.y = motion2Cognition->torsoMatrix.offset.rotation.c2.y;
    bb->theTorsoMatrix.rotation.c2.z = motion2Cognition->torsoMatrix.offset.rotation.c2.z;
}




void UChCognitionExecutor::copyCognition2MotionBB(bh_motion::Cognition2Motion& cognition2Motion)
{
    cognition2Motion.motionRequest.motion = bb->theMotionRequest.motion;

    //  ------ WalkingRequest -----
    cognition2Motion.motionRequest.walkRequest.mode = bb->theMotionRequest.walkRequest.mode;
    cognition2Motion.motionRequest.walkRequest.pedantic = bb->theMotionRequest.walkRequest.pedantic;
    cognition2Motion.motionRequest.walkRequest.dribbling = bb->theMotionRequest.walkRequest.dribbling;
    cognition2Motion.motionRequest.walkRequest.speed.rotation = bb->theMotionRequest.walkRequest.speed.rotation;
    cognition2Motion.motionRequest.walkRequest.speed.translation.x = bb->theMotionRequest.walkRequest.speed.translation.x;
    cognition2Motion.motionRequest.walkRequest.speed.translation.y = bb->theMotionRequest.walkRequest.speed.translation.y;
    cognition2Motion.motionRequest.walkRequest.target.rotation = bb->theMotionRequest.walkRequest.target.rotation;
    cognition2Motion.motionRequest.walkRequest.target.translation.x = bb->theMotionRequest.walkRequest.target.translation.x;
    cognition2Motion.motionRequest.walkRequest.target.translation.y = bb->theMotionRequest.walkRequest.target.translation.y;

    //------BikeRequest-----
    cognition2Motion.motionRequest.bikeRequest.mirror = bb->theMotionRequest.bikeRequest.mirror;
    cognition2Motion.motionRequest.bikeRequest.dynamical = bb->theMotionRequest.bikeRequest.dynamical;
    cognition2Motion.motionRequest.bikeRequest.ballSpecial = bb->theMotionRequest.bikeRequest.ballSpecial;
    cognition2Motion.motionRequest.bikeRequest.bMotionType = bb->theMotionRequest.bikeRequest.bMotionType;
    bb->theMotionRequest.bikeRequest.dynPoints.resize(2);
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].limb = bb->theMotionRequest.bikeRequest.dynPoints[0].limb;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].phaseNumber = bb->theMotionRequest.bikeRequest.dynPoints[0].phaseNumber;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].duration = bb->theMotionRequest.bikeRequest.dynPoints[0].duration;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].translation.x = bb->theMotionRequest.bikeRequest.dynPoints[0].translation.x;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].translation.y = bb->theMotionRequest.bikeRequest.dynPoints[0].translation.y;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].translation.z = bb->theMotionRequest.bikeRequest.dynPoints[0].translation.z;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].angle.x = bb->theMotionRequest.bikeRequest.dynPoints[0].angle.x;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].angle.y = bb->theMotionRequest.bikeRequest.dynPoints[0].angle.y;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].angle.z = bb->theMotionRequest.bikeRequest.dynPoints[0].angle.z;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].odometryOffset.x = bb->theMotionRequest.bikeRequest.dynPoints[0].odometryOffset.x;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].odometryOffset.y = bb->theMotionRequest.bikeRequest.dynPoints[0].odometryOffset.y;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[0].odometryOffset.z = bb->theMotionRequest.bikeRequest.dynPoints[0].odometryOffset.z;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].limb = bb->theMotionRequest.bikeRequest.dynPoints[1].limb;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].phaseNumber = bb->theMotionRequest.bikeRequest.dynPoints[1].phaseNumber;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].duration = bb->theMotionRequest.bikeRequest.dynPoints[1].duration;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].translation.x = bb->theMotionRequest.bikeRequest.dynPoints[1].translation.x;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].translation.y = bb->theMotionRequest.bikeRequest.dynPoints[1].translation.y;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].translation.z = bb->theMotionRequest.bikeRequest.dynPoints[1].translation.z;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].angle.x = bb->theMotionRequest.bikeRequest.dynPoints[1].angle.x;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].angle.y = bb->theMotionRequest.bikeRequest.dynPoints[1].angle.y;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].angle.z = bb->theMotionRequest.bikeRequest.dynPoints[1].angle.z;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].odometryOffset.x = bb->theMotionRequest.bikeRequest.dynPoints[1].odometryOffset.x;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].odometryOffset.y = bb->theMotionRequest.bikeRequest.dynPoints[1].odometryOffset.y;
    cognition2Motion.motionRequest.bikeRequest.dynPoints[1].odometryOffset.z = bb->theMotionRequest.bikeRequest.dynPoints[1].odometryOffset.z;

    //  ------ SpecialActionRequest -----
    cognition2Motion.motionRequest.specialActionRequest.mirror = bb->theMotionRequest.specialActionRequest.mirror;
    cognition2Motion.motionRequest.specialActionRequest.specialAction = bb->theMotionRequest.specialActionRequest.specialAction;

    //------DamageConfiguration-----
    cognition2Motion.damageConfiguration.useGroundContactDetection = bb->theDamageConfiguration.useGroundContactDetection;
    cognition2Motion.damageConfiguration.useGroundContactDetectionForLEDs = bb->theDamageConfiguration.useGroundContactDetectionForLEDs;
    cognition2Motion.damageConfiguration.useGroundContactDetectionForSafeStates = bb->theDamageConfiguration.useGroundContactDetectionForSafeStates;
    cognition2Motion.damageConfiguration.useGroundContactDetectionForSensorCalibration = bb->theDamageConfiguration.useGroundContactDetectionForSensorCalibration;

    //-----HeadAngleRequest------
    cognition2Motion.headAngleRequest.pan = bb->theHeadAngleRequest.pan;
    cognition2Motion.headAngleRequest.tilt = bb->theHeadAngleRequest.tilt;
    cognition2Motion.headAngleRequest.speed = bb->theHeadAngleRequest.speed;

    //-----RobotInforobotInfo------
    cognition2Motion.robotInfo.penalty = bb->theRobotInfo.penalty;
    cognition2Motion.robotInfo.secsTillUnpenalised = bb->theRobotInfo.secsTillUnpenalised;

}
// DLF ---->
