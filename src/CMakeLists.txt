cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(uchrt_bhmotion)

include(/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/cross-config.cmake)

set( srcs_uchrobotcomm
    uch_src/UChRobotComm/uchrobotcomm.h
    uch_src/UChRobotComm/uchrobotcomm.cpp
    uch_src/UChRobotComm/UChRobotData.h
)

aux_source_directory(bh_src/Src/Platform/Win32Linux Win32Linux)

message(${Win32Linux})

set( srcs_uchprocess
    uch_src/UChProcess/main.cpp
    uch_src/UChProcess/process.h
    uch_src/UChProcess/process.cpp
    uch_src/UChRobotComm/UChRobotData.h
    uch_src/UChProcess/UChBlackBoard.h
    uch_src/UChProcess/UChCognitionExecutor.h
    uch_src/UChProcess/UChCognitionExecutor.cpp
    uch_src/UChProcess/UChMotionExecutor.h
    uch_src/UChProcess/UChMotionExecutor.cpp

    #Src/Controller/RoboCupCtrl.h
    #Src/Controller/RoboCupCtrl.cpp
    #Src/Controller/RobotConsole.h

    /home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/linux/videodev2.h
    /home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/linux/version.h
    /home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/unistd.h

    bh_src/Src/Modules/BehaviorControl/Symbols.h
    bh_src/Src/Modules/BehaviorControl/BasicBehavior.h
    bh_src/Src/Modules/BehaviorControl/LEDHandler/LEDHandler.cpp
    bh_src/Src/Modules/BehaviorControl/CameraControlEngine/CameraControlEngine.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/BH2011BehaviorControl.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011BallSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011BehaviorLEDSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011BIKESymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011FallDownSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011FieldSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011GameSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011GoalSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011HeadSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011LocatorSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011LocatorSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/BH2011BehaviorControl/Symbols/BH2011ObstacleSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/CommonSymbols/CommonBIKESymbols.cpp
    bh_src/Src/Modules/BehaviorControl/CommonSymbols/KeySymbols.cpp
    bh_src/Src/Modules/BehaviorControl/CommonSymbols/MathSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/CommonSymbols/MotionSymbols.cpp
    bh_src/Src/Modules/BehaviorControl/CommonSymbols/SoundSymbols.cpp
    bh_src/Src/Modules/Configuration/CognitionConfigurationDataProvider.cpp
    bh_src/Src/Modules/Configuration/CognitionConfigurationDataProvider.h
    bh_src/Src/Modules/Configuration/WhiteCorrection.cpp
    bh_src/Src/Modules/Configuration/CameraCalibrator.cpp
    #Src/Modules/Configuration/MotionConfigurationDataProvider.h
    #Src/Modules/Configuration/MotionConfigurationDataProvider.cpp
    #Src/Modules/Infrastructure/MotionLogDataProvider.h
    #Src/Modules/Infrastructure/MotionLogDataProvider.cpp
    bh_src/Src/Modules/Infrastructure/CameraProvider.cpp
    bh_src/Src/Modules/Infrastructure/GameDataProvider.cpp
    bh_src/Src/Modules/Infrastructure/TeamDataProvider.cpp
    bh_src/Src/Modules/Infrastructure/NaoProvider.h
    bh_src/Src/Modules/Infrastructure/NaoProvider.cpp
    bh_src/Src/Modules/Modeling/ArmContactProvider/ArmContactModelProvider.cpp
    bh_src/Src/Modules/Modeling/BallLocator/BallLocator.cpp
    bh_src/Src/Modules/Modeling/BallPredictor/BallPredictor.cpp
    bh_src/Src/Modules/Modeling/CombinedWorldModelProvider/CombinedWorldModelProvider.cpp
    bh_src/Src/Modules/Modeling/GlobalFieldCoverageProvider/GlobalFieldCoverageProvider.cpp
    bh_src/Src/Modules/Modeling/GroundTruthProvider/GroundTruthEvaluator.cpp
    bh_src/Src/Modules/Modeling/GroundTruthProvider/GroundTruthProvider.cpp
    bh_src/Src/Modules/Modeling/FieldCoverageProvider/FieldCoverageProvider.cpp
    bh_src/Src/Modules/Modeling/FreePartOfOpponentGoalProvider/FreePartOfOpponentGoalProvider.cpp
    bh_src/Src/Modules/Modeling/ObstacleModelProvider/ObstacleCombinator.cpp
    bh_src/Src/Modules/Modeling/ObstacleModelProvider/USObstacleGridProvider.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SelfLocator.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/FieldModel.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SampleTemplateGenerator.cpp
    bh_src/Src/Modules/Modeling/RobotLocator/RobotLocatorUKF.cpp
    bh_src/Src/Modules/Modeling/RobotLocator/UKF.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/CenterCircleSensorModel.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/CornersSensorModel.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/FieldModel.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/GoalPostsSensorModel.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/LineSensorModel.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/PerceptValidityChecker.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SensorModels/SensorModel.h
    bh_src/Src/Modules/Modeling/RobotPoseValidator/RobotPoseValidator.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/SelfLocatorParameter.cpp
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/PoseCalculator/PoseCalculator.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/PoseCalculator/PoseCalculator2DBinning.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/PoseCalculator/PoseCalculatorBestParticle.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/PoseCalculator/PoseCalculatorKMeansClustering.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/PoseCalculator/PoseCalculatorOverallAverage.h
    bh_src/Src/Modules/Modeling/ParticleFilterSelfLocator/PoseCalculator/PoseCalculatorParticleHistory.h
    bh_src/Src/Modules/MotionControl/BLAME.cpp
    bh_src/Src/Modules/MotionControl/BlameData.cpp
    bh_src/Src/Modules/MotionControl/BIKEParameters.cpp
    bh_src/Src/Modules/MotionControl/HeadMotionEngine.cpp
    bh_src/Src/Modules/MotionControl/MotionCombinator.cpp
    bh_src/Src/Modules/MotionControl/MotionSelector.cpp
    bh_src/Src/Modules/MotionControl/SpecialActions.cpp
    bh_src/Src/Modules/MotionControl/WalkingEngine/WalkingEngine.cpp
    bh_src/Src/Modules/MotionControl/WalkingEngine/WalkingEngineKick.cpp
    bh_src/Src/Modules/Perception/RobotCameraMatrixProvider.cpp
    bh_src/Src/Modules/Perception/CameraMatrixProvider.cpp
    bh_src/Src/Modules/Perception/BodyContourProvider.cpp
    bh_src/Src/Modules/Perception/CoordinateSystemProvider.cpp
    bh_src/Src/Modules/Perception/GoalPerceptor.cpp
    bh_src/Src/Modules/Perception/Regionizer.cpp
    bh_src/Src/Modules/Perception/TeamMarkerPerceptor.cpp
    bh_src/Src/Modules/Perception/RegionAnalyzer.cpp
    bh_src/Src/Modules/Perception/RobotPerceptor.cpp
    bh_src/Src/Modules/Perception/BallPerceptor.cpp
    bh_src/Src/Modules/Perception/LinePerceptor.cpp
    bh_src/Src/Modules/Perception/PointExplorer.cpp
    bh_src/Src/Modules/Perception/Storage.h
    bh_src/Src/Modules/Sensing/FallDownStateDetector.cpp
    bh_src/Src/Modules/Sensing/GroundContactDetector.cpp
    bh_src/Src/Modules/Sensing/InertiaSensorCalibrator.cpp
    bh_src/Src/Modules/Sensing/InertiaSensorFilter.cpp
    bh_src/Src/Modules/Sensing/InertiaSensorInspector.cpp
    bh_src/Src/Modules/Sensing/JointFilter.cpp
    bh_src/Src/Modules/Sensing/RobotModelProvider.cpp
    bh_src/Src/Modules/Sensing/SensorFilter.cpp
    bh_src/Src/Modules/Sensing/TorsoMatrixProvider.cpp
    bh_src/Src/Platform/Camera.h
    bh_src/Src/Platform/SystemCall.h
    bh_src/Src/Platform/BHAssert.h
    bh_src/Src/Platform/linux/BHAssert.cpp
    bh_src/Src/Platform/linux/SystemCall.cpp
    bh_src/Src/Platform/linux/NaoCamera.h
    bh_src/Src/Platform/linux/NaoCamera.cpp
    bh_src/Src/Platform/File.h
    bh_src/Src/Platform/UdpComm.h
    bh_src/Src/Platform/Win32Linux/UdpComm.cpp
    bh_src/Src/Platform/Win32Linux/File.cpp
    bh_src/Src/Representations/BehaviorControl/BehaviorControlOutput.h
    bh_src/Src/Representations/Configuration/BehaviorConfiguration.h
    bh_src/Src/Representations/Configuration/CameraCalibration.h
    bh_src/Src/Representations/Configuration/CameraSettings.cpp
    bh_src/Src/Representations/Configuration/ColorTable64.cpp
    bh_src/Src/Representations/Configuration/DamageConfiguration.h
    bh_src/Src/Representations/Configuration/FieldDimensions.cpp
    bh_src/Src/Representations/Configuration/JointCalibration.h
    bh_src/Src/Representations/Configuration/MassCalibration.h
    bh_src/Src/Representations/Configuration/RobotDimensions.h
    bh_src/Src/Representations/Configuration/SensorCalibration.h
    bh_src/Src/Representations/Infrastructure/BoardInfo.h
    bh_src/Src/Representations/Infrastructure/FrameInfo.h
    bh_src/Src/Representations/Infrastructure/GameInfo.h
    bh_src/Src/Representations/Infrastructure/CameraInfo.cpp
    bh_src/Src/Representations/Infrastructure/Image.cpp
    bh_src/Src/Representations/Infrastructure/TeamInfo.cpp
    bh_src/Src/Representations/Infrastructure/TeamMateData.h
    bh_src/Src/Representations/Infrastructure/JointData.h
    bh_src/Src/Representations/Infrastructure/RoboCupGameControlData.h
    bh_src/Src/Representations/Infrastructure/SensorData.h
    bh_src/Src/Representations/Infrastructure/JointData.h
    bh_src/Src/Representations/Infrastructure/KeyStates.h
    bh_src/Src/Representations/Infrastructure/LEDRequest.h
    bh_src/Src/Representations/Infrastructure/RobotInfo.h
    bh_src/Src/Representations/Infrastructure/SensorData.h
    bh_src/Src/Representations/Infrastructure/TeamInfo.cpp
    bh_src/Src/Representations/Infrastructure/TeamMateData.h
    bh_src/Src/Representations/Infrastructure/SoundRequest.h
    bh_src/Src/Representations/Modeling/ArmContactModel.h
    bh_src/Src/Representations/Modeling/BallHypotheses.h
    bh_src/Src/Representations/Modeling/CombinedWorldModel.cpp
    bh_src/Src/Representations/Modeling/FallDownState.cpp
    bh_src/Src/Representations/Modeling/FieldCoverage.cpp
    bh_src/Src/Representations/Modeling/GlobalFieldCoverage.cpp
    bh_src/Src/Representations/Modeling/GroundTruthResult.h
    bh_src/Src/Representations/Modeling/RobotPose.cpp
    bh_src/Src/Representations/Modeling/RobotsModel.cpp
    bh_src/Src/Representations/Modeling/ObstacleModel.cpp
    bh_src/Src/Representations/Modeling/SSLVisionData.cpp
    bh_src/Src/Representations/Modeling/FreePartOfOpponentGoalModel.cpp
    bh_src/Src/Representations/MotionControl/BikeEngineOutput.h
    bh_src/Src/Representations/MotionControl/BikeRequest.cpp
    bh_src/Src/Representations/MotionControl/HeadAngleRequest.h
    bh_src/Src/Representations/MotionControl/HeadJointRequest.h
    bh_src/Src/Representations/MotionControl/MotionInfo.h
    bh_src/Src/Representations/MotionControl/MotionRequest.cpp
    bh_src/Src/Representations/MotionControl/MotionSelection.cpp
    bh_src/Src/Representations/MotionControl/OdometryData.h
    bh_src/Src/Representations/MotionControl/SpecialActionRequest.h
    bh_src/Src/Representations/MotionControl/SpecialActionsOutput.h
    bh_src/Src/Representations/MotionControl/WalkingEngineOutput.h
    bh_src/Src/Representations/MotionControl/WalkingEngineStandOutput.h
    bh_src/Src/Representations/Perception/BallPercept.cpp
    bh_src/Src/Representations/MotionControl/WalkRequest.h
    bh_src/Src/Representations/Perception/BallSpots.h
    bh_src/Src/Representations/Perception/CameraMatrix.cpp
    bh_src/Src/Representations/Perception/ColorClassImage.cpp
    bh_src/Src/Representations/Perception/BodyContour.cpp
    bh_src/Src/Representations/Perception/ImageCoordinateSystem.cpp
    bh_src/Src/Representations/Perception/ImageInfo.h
    bh_src/Src/Representations/Perception/LinePercept.cpp
    bh_src/Src/Representations/Perception/LineSpots.h
    bh_src/Src/Representations/Perception/GoalPercept.cpp
    bh_src/Src/Representations/Perception/RegionPercept.cpp
    bh_src/Src/Representations/Perception/RobotPercept.h
    bh_src/Src/Representations/Perception/TeamMarkerSpots.cpp
    bh_src/Src/Representations/Sensing/GroundContactState.h
    bh_src/Src/Representations/Sensing/InertiaSensorData.h
    bh_src/Src/Representations/Sensing/OrientationData.h
    bh_src/Src/Representations/Sensing/RobotModel.cpp
    bh_src/Src/Representations/Sensing/TorsoMatrix.h
    bh_src/Src/Tools/Boundary.h
    bh_src/Src/Tools/Enum.cpp
    bh_src/Src/Tools/Global.cpp
    bh_src/Src/Tools/ForwardKinematic.h
    bh_src/Src/Tools/InverseKinematic.h
    bh_src/Src/Tools/Range.h
    bh_src/Src/Tools/Sha1.cpp
    bh_src/Src/Tools/RingBuffer.h
    bh_src/Src/Tools/RingBufferWithSum.h
    bh_src/Src/Tools/SampleSet.h
    #bh_src/Src/Tools/Settings.cpp
    #bh_src/Src/Tools/Settings.h
    bh_src/Src/Tools/Team.h
    bh_src/Src/Tools/Configuration/ConfigMap.cpp
    bh_src/Src/Tools/Configuration/ConfigMapParser.cpp
    bh_src/Src/Tools/Debugging/Debugging.h
    bh_src/Src/Tools/Debugging/DebugRequest.h
    bh_src/Src/Tools/Debugging/Asserts.h
    bh_src/Src/Tools/ImageProcessing/BresenhamLineScan.cpp
    bh_src/Src/Tools/ImageProcessing/ColorModelConversions.h
    bh_src/Src/Tools/Math/Common.h
    bh_src/Src/Tools/Math/Covariance.h
    bh_src/Src/Tools/Math/GaussianTable.h
    bh_src/Src/Tools/Math/GaussianDistribution3D.cpp
    bh_src/Src/Tools/Math/Probabilistics.h
    bh_src/Src/Tools/Math/Geometry.cpp   
    bh_src/Src/Tools/Math/Kalman.h
    bh_src/Src/Tools/Math/Matrix.cpp
    bh_src/Src/Tools/Math/Matrix2x2.h
    bh_src/Src/Tools/Math/Matrix3x3.h
    bh_src/Src/Tools/Math/MVTools.cpp
    bh_src/Src/Tools/Math/Pose2D.cpp
    bh_src/Src/Tools/Math/Pose3D.h
    bh_src/Src/Tools/Math/Random.h
    bh_src/Src/Tools/Math/RotationMatrix.cpp
    bh_src/Src/Tools/Math/Vector.h
    bh_src/Src/Tools/Math/Vector2.h
    bh_src/Src/Tools/Math/Vector3.h
    bh_src/Src/Tools/Math/YaMatrix.h
    bh_src/Src/Tools/Optimization/ParticleSwarm.cpp
    bh_src/Src/Tools/ProcessFramework/GameHandler.cpp
    bh_src/Src/Tools/Streams/InOut.cpp
    bh_src/Src/Tools/Streams/InStreams.cpp
    bh_src/Src/Tools/Streams/OutStreams.cpp
    bh_src/Src/Tools/Streams/Streamable.cpp
    bh_src/Src/Tools/Streams/StreamHandler.cpp
    bh_src/Src/Tools/Xabsl/B-Human/BHXabslEngineExecutor.cpp
    bh_src/Src/Tools/Xabsl/B-Human/BHXabslMessage.h
    bh_src/Src/Tools/Xabsl/B-Human/BHXabslTools.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslAction.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslAgent.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslArray.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslBasicBehavior.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslBehavior.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslBooleanExpression.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslCoopState.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslDecimalExpression.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslEngine.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslEnumeratedExpression.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslOption.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslParameters.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslState.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslStatement.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslSymbolInputValue.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslSymbolOutputValue.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslSymbols.cpp
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslSymbolValue.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslTeamMessage.h
    bh_src/Src/Tools/Xabsl/XabslEngine/XabslTools.cpp

    bh_src/Util/I2C/include/linux/i2c-dev.h
)

#  DLF <---
set( srcs_uchNaoROS
    exampleNode.cpp
    motionNode.cpp
)
# DLF --->


#NO BORRAR, POSIBLEMENTE LO USAREMOS MÁS ADELANTE.
#set( Boost_USE_STATIC_LIBS    ON )
#set( Boost_USE_MULTITHREADED  ON )
#set( Boost_USE_STATIC_RUNTIME OFF)
#find_package( Boost COMPONENTS thread date_time system )

ADD_LIBRARY(boost_thread STATIC IMPORTED)
SET_TARGET_PROPERTIES(boost_thread PROPERTIES IMPORTED_LOCATION /home/nao/naoqi-sdk-1.12.5-linux32/lib/libboost_thread-mt.a)

ADD_LIBRARY(boost_data STATIC IMPORTED)
SET_TARGET_PROPERTIES(boost_data PROPERTIES IMPORTED_LOCATION /home/nao/naoqi-sdk-1.12.5-linux32/lib/libboost_date_time-mt.a)

ADD_LIBRARY(boost_system STATIC IMPORTED)
SET_TARGET_PROPERTIES(boost_system PROPERTIES IMPORTED_LOCATION /home/nao/naoqi-sdk-1.12.5-linux32/lib/libboost_system-mt.a)

#find_package( Threads REQUIRED )

#include_directories(/usr/include)
include_directories(/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include)
include_directories(bh_src/Src)

include_directories(/home/nao/ros-nao/ros/fuerte/include) #//DLF
include_directories(/mnt/hgfs/UChLib/UChRoboticsTeam/naoros2/msg_gen/cpp/include) #//DLF

#include_directories(/mnt/hgfs/UChLib/BHActuation)
#find_library(alcommon NAMES alcommon PATHS /home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/lib)

#IF (alcommon)
#message(alcommonTrue)
#ENDIF (alcommon)

set(NAO_PATH /home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/lib)
set(ALCOMMON_LIBRARIES "${NAO_PATH}/libalcommon.so" CACHE STRING "" FORCE)
set(ALCOMMON_INCLUDE_DIR "${NAO_PATH}/../include" CACHE STRING "" FORCE)

set(ALCOMMON_DEFINITIONS "" CACHE STRING "" FORCE)
add_definitions(-DTARGET_ROBOT -DLINUX -DRELEASE)

#include(FindPackageHandleStandardArgs)
#FIND_PACKAGE_HANDLE_STANDARD_ARGS(ALCOMMON DEFAULT_MSG
#  ALCOMMON_LIBRARIES
#  ALCOMMON_INCLUDE_DIR
#)

MARK_AS_ADVANCED(
  ALCOMMON_DEFINITIONS
  ALCOMMON_INCLUDE_DIR
  ALCOMMON_LIBRARIES
)

include_directories(  ${ALCOMMON_INCLUDE_DIR})

add_library( uchilerobotcomm SHARED ${srcs_uchrobotcomm})
target_link_libraries( uchilerobotcomm boost_thread boost_data boost_system pthread rt ${ALCOMMON_LIBRARIES})

add_executable( process ${srcs_uchprocess})
target_link_libraries( process boost_thread boost_data boost_system pthread rt)

add_executable( nodes ${srcs_uchNaoROS})

