#ifndef _UCH_COGNITIONEXECUTOR_
#define _UCH_COGNITIONEXECUTOR_

#include "process.h"
#include <boost/thread/thread_time.hpp>


// MODULES


#include "Modules/BehaviorControl/BH2011BehaviorControl/BH2011BehaviorControl.h"
#include "Modules/BehaviorControl/CameraControlEngine/CameraControlEngine.h"
#include "Modules/BehaviorControl/LEDHandler/LEDHandler.h"
#include "Modules/Configuration/CognitionConfigurationDataProvider.h"
#include "Modules/Configuration/CameraCalibrator.h"
#include "Modules/Configuration/WhiteCorrection.h"
#include "Modules/Infrastructure/CameraProvider.h"
#include "Modules/Infrastructure/GameDataProvider.h"
#include "Modules/Infrastructure/TeamDataProvider.h"
#include "Modules/Modeling/ParticleFilterSelfLocator/SelfLocator.h"
#include "Modules/Modeling/RobotPoseValidator/RobotPoseValidator.h"
#include "Modules/Modeling/BallLocator/BallLocator.h"
#include "Modules/Modeling/ArmContactProvider/ArmContactModelProvider.h"
#include "Modules/Modeling/RobotLocator/RobotLocatorUKF.h"
#include "Modules/Modeling/ObstacleModelProvider/USObstacleGridProvider.h"
#include "Modules/Modeling/FreePartOfOpponentGoalProvider/FreePartOfOpponentGoalProvider.h"
#include "Modules/Modeling/ObstacleModelProvider/ObstacleCombinator.h"
#include "Modules/Modeling/BallPredictor/BallPredictor.h"
#include "Modules/Modeling/CombinedWorldModelProvider/CombinedWorldModelProvider.h"
#include "Modules/Modeling/FieldCoverageProvider/FieldCoverageProvider.h"
#include "Modules/Modeling/GlobalFieldCoverageProvider/GlobalFieldCoverageProvider.h"
#include "Modules/Modeling/GroundTruthProvider/GroundTruthEvaluator.h"
#include "Modules/Modeling/GroundTruthProvider/GroundTruthProvider.h"
#include "Modules/Modeling/RobotLocator/RobotLocatorUKF.h"
#include "Modules/Perception/BallPerceptor.h"
#include "Modules/Perception/BodyContourProvider.h"
#include "Modules/Perception/CameraMatrixProvider.h"
#include "Modules/Perception/CoordinateSystemProvider.h"
#include "Modules/Perception/GoalPerceptor.h"
#include "Modules/Perception/LinePerceptor.h"
#include "Modules/Perception/RegionAnalyzer.h"
#include "Modules/Perception/Regionizer.h"
#include "Modules/Perception/RobotCameraMatrixProvider.h"
#include "Modules/Perception/RobotPerceptor.h"
#include "Modules/Perception/TeamMarkerPerceptor.h"


class process;

class UChCognitionExecutor
{
public:
    UChCognitionExecutor(UChBlackBoard* blackBoard_);
    ~UChCognitionExecutor();

    //void copyMotion2CognitionBB(const bh_motion::Motion2Cognition& motion2Cognition);  //DLF
    void copyMotion2CognitionBB(const bh_motion::Motion2Cognition::ConstPtr& motion2Cognition);
    void copyCognition2MotionBB(bh_motion::Cognition2Motion& cognition2Motion);  //DLF

    void init();
    void runModules();


    UChBlackBoard* bb;

    boost::system_time initialTime;
    boost::system_time finalTime;
    //-------- Modules ------------------
    CognitionConfigurationDataProvider cognitionConfigurationDataProvider;
    WhiteCorrection whiteCorrection;
    //CognitionLogDataProviderBase cognitionLogDataProviderBase;
    CameraCalibrator cameraCalibrator;
    CameraProvider cameraProvider;
    GameDataProvider gameDataProvider;
    RobotCameraMatrixProvider robotCameraMatrixProvider;
    CameraMatrixProvider cameraMatrixProvider;
    CoordinateSystemProvider coordinateSystemProvider;
    BodyContourProvider bodyContourProvider;
    ArmContactModelProvider armContactModelProvider;
    TeamDataProvider teamDataProvider;
    GoalPerceptor goalPerceptor;
    Regionizer regionizer;
    TeamMarkerPerceptor teamMarkerPerceptor;
    RegionAnalyzer regionAnalyzer;
    RobotPerceptor robotPerceptor;
    BallPerceptor ballPerceptor;
    LinePerceptor linePerceptor;
    BH2011BehaviorControl bh2011BehaviorControl;
    LEDHandler ledHandler;
    CameraControlEngine cameraControlEngine;
    SelfLocator selfLocator;
    RobotPoseValidator robotPoseValidator;
    BallLocator ballLocator;
    RobotLocatorUKF robotLocatorUKF;
    USObstacleGridProvider usObstacleGridProvider;
    FreePartOfOpponentGoalProvider freePartOfOpponentGoalProvider;
    ObstacleCombinator obstacleCombinator;
    BallPredictor ballPredictor;
    CombinedWorldModelProvider combinedWorldModelProvider;
    FieldCoverageProvider fieldCoverageProvider;
    GlobalFieldCoverageProvider globalFieldCoverageProvider;
    GroundTruthEvaluator groundTruthEvaluator;
    GroundTruthProvider groundTruthProvider;
};

#endif
