#ifndef _UCHBLACKBOARD_
#define _UCHBLACKBOARD_

#pragma once

#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/Configuration/BehaviorConfiguration.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/SensorCalibration.h"
#include "Representations/Infrastructure/BoardInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/SoundRequest.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/GroundTruthResult.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/SSLVisionData.h"
#include "Representations/Modeling/USObstacleGrid.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineStandOutput.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ImageInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Perception/TeamMarkerSpots.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

#include <boost/thread/mutex.hpp>

class UChBlackBoard
{
public :

    UChBlackBoard(){};
    ~UChBlackBoard(){};

    //Representaciones.

    //Motion
    JointCalibration theJointCalibration;
    MassCalibration theMassCalibration;
    RobotDimensions theRobotDimensions;
    SensorCalibration theSensorCalibration;
    HardnessSettings theHardnessSettings;
    JointData theJointData;
    BoardInfo theBoardInfo;
    SensorData theSensorData;
    FrameInfo theFrameInfo;
    InspectedInertiaSensorData theInspectedInertiaSensorData;
    InertiaSensorData theInertiaSensorData;
    MotionSelection theMotionSelection;
    WalkingEngineOutput theWalkingEngineOutput;
    OrientationData theOrientationData;
    SpecialActionsOutput theSpecialActionsOutput;
    WalkingEngineStandOutput theWalkingEngineStandOutput;
    UnstableJointRequest theUnstableJointRequest;

    //Cognition
    CameraSettings theCameraSettings;
    Image theImage;
    CameraCalibration theCameraCalibration;
    CameraInfo theCameraInfo;
    ImageInfo theImageInfo;
    ImageRequest theImageRequest;
    GameInfo theGameInfo;
    RobotPose theRobotPose;
    BehaviorControlOutput theBehaviorControlOutput;
    OpponentTeamInfo theOpponentTeamInfo;
    OwnTeamInfo theOwnTeamInfo;
    FieldDimensions theFieldDimensions;
    RobotCameraMatrix theRobotCameraMatrix;
    RobotCameraMatrixPrev theRobotCameraMatrixPrev;
    RobotCameraMatrixOther theRobotCameraMatrixOther;
    CameraMatrix theCameraMatrix;
    CameraMatrixPrev theCameraMatrixPrev;
    CameraMatrixOther theCameraMatrixOther;
    TeamMateData theTeamMateData;
    GoalPercept theGoalPercept;
    RobotsModel theRobotsModel;
    BallModel theBallModel;
    FreePartOfOpponentGoalModel theFreePartOfOpponentGoalModel;
    BallHypotheses theBallHypotheses;
    ObstacleModel theObstacleModel;
    RobotPoseInfo theRobotPoseInfo;
    ArmContactModel theArmContactModel;
    BehaviorConfiguration theBehaviorConfiguration;
    SoundRequest theSoundRequest;
    BehaviorLEDRequest theBehaviorLEDRequest;
    HeadMotionRequest theHeadMotionRequest;
    LEDRequest theLEDRequest;
    CombinedWorldModel theCombinedWorldModel;
    FieldCoverage theFieldCoverage;
    ImageCoordinateSystem theImageCoordinateSystem;
    ColorTable64 theColorTable64;
    GroundTruthRobotPose theGroundTruthRobotPose;
    GroundTruthBallModel theGroundTruthBallModel;
    BodyContour theBodyContour;
    RegionPercept theRegionPercept;
    TeamMarkerSpots theTeamMarkerSpots;
    BallPercept theBallPercept;
    BallSpots theBallSpots;
    LinePercept theLinePercept;
    LineSpots theLineSpots;
    RobotPercept theRobotPercept;
    SSLVisionData theSSLVisionData;
    PotentialRobotPose thePotentialRobotPose;
    GlobalFieldCoverage theGlobalFieldCoverage;
    USObstacleGrid theUSObstacleGrid;
    GroundTruthResult theGroundTruthResult;
    GroundTruthRobotsModel theGroundTruthRobotsModel;

    //Shared theMotionToCognition theRepresentations
    BikeEngineOutput theBikeEngineOutput;
    CognitionFrameInfo theCognitionFrameInfo;
    FallDownState theFallDownState;
    FilteredJointData theFilteredJointData;
    FilteredJointDataPrev theFilteredJointDataPrev;
    FilteredSensorData theFilteredSensorData;
    GroundContactState theGroundContactState;
    HeadJointRequest theHeadJointRequest;
    JointRequest theJointRequest;
    KeyStates theKeyStates;
    MotionInfo theMotionInfo;
    MotionRobotHealth theMotionRobotHealth;
    OdometryData theOdometryData;
    RobotModel theRobotModel;
    TorsoMatrix theTorsoMatrix;

    //Shared theCognitionToMotion theRepresentations
    DamageConfiguration theDamageConfiguration;
    HeadAngleRequest theHeadAngleRequest;
    MotionRequest theMotionRequest;
    RobotInfo theRobotInfo;

    //friend class UChMotionExecutor; //No me funcionó :-(
};




class UChBlackBoardFromMotion
{
public :
    UChBlackBoardFromMotion() {};

    //Shared MotionToCognition representations
    BikeEngineOutput theBikeEngineOutput;   //sacable
    CognitionFrameInfo theCognitionFrameInfo;  //??
    FallDownState theFallDownState;    // caidas
    FilteredJointData theFilteredJointData;    // sensores angulo joints
    FilteredJointDataPrev theFilteredJointDataPrev;   // sensores angulo joints  k-1
    FilteredSensorData theFilteredSensorData;        // todos los sensores
    GroundContactState theGroundContactState;            //sacable  contacto con pies piso
    HeadJointRequest theHeadJointRequest;               // solicitud joints cabeza
    JointRequest theJointRequest;                   // solicitud joints
    KeyStates theKeyStates;                         //  bumpers y boton
    MotionInfo theMotionInfo;                       // q estoy haciendo
    MotionRobotHealth theMotionRobotHealth;         //???
    OdometryData theOdometryData;                   //odometria
    RobotModel theRobotModel;                       //??
    TorsoMatrix theTorsoMatrix;                     // cinematica torso

};

class UChBlackBoardFromCognition
{
public :
    UChBlackBoardFromCognition() {};

    //Shared CognitionToMotion representations
    DamageConfiguration theDamageConfiguration;   //sacable
    HeadAngleRequest theHeadAngleRequest;         //cabeza
    MotionRequest theMotionRequest;             //motion
    RobotInfo theRobotInfo;                     //lo q esta haciendo

};


#endif
