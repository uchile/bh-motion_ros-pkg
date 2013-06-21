cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()
rosbuild_add_boost_directories ()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ************************************
# PLEASE MODIFY ACCORDING YOU CTC PATH
set(NAO_PATH /home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/lib)
include(/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/cross-config.cmake)

# ************************************

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


set( srcs_uchrobotcomm
    src/uch_src/UChRobotComm/uchrobotcomm.h
    src/uch_src/UChRobotComm/uchrobotcomm.cpp
    src/uch_src/UChRobotComm/UChRobotData.h
)

aux_source_directory(src/bh_src/Src/Platform/Win32Linux Win32Linux)

message(${Win32Linux})

set( srcs_motion
    src/uch_src/UChProcess/process.h
    src/uch_src/UChProcess/process.cpp
    src/uch_src/UChRobotComm/UChRobotData.h
    src/uch_src/UChProcess/UChBlackBoard.h
    src/uch_src/UChProcess/UChMotionExecutor.h
    src/uch_src/UChProcess/UChMotionExecutor.cpp

    src/bh_src/Src/Modules/Infrastructure/NaoProvider.h
    src/bh_src/Src/Modules/Infrastructure/NaoProvider.cpp

    src/bh_src/Src/Modules/MotionControl/BLAME.cpp
    src/bh_src/Src/Modules/MotionControl/BlameData.cpp
    src/bh_src/Src/Modules/MotionControl/BIKEParameters.cpp
    src/bh_src/Src/Modules/MotionControl/HeadMotionEngine.cpp
    src/bh_src/Src/Modules/MotionControl/MotionCombinator.cpp
    src/bh_src/Src/Representations/MotionControl/MotionRequest.cpp
    src/bh_src/Src/Representations/MotionControl/MotionSelection.cpp
    src/bh_src/Src/Modules/MotionControl/MotionSelector.cpp
    src/bh_src/Src/Modules/MotionControl/SpecialActions.cpp
    src/bh_src/Src/Modules/MotionControl/WalkingEngine/WalkingEngine.cpp
    src/bh_src/Src/Modules/MotionControl/WalkingEngine/WalkingEngineKick.cpp
  
    src/bh_src/Src/Modules/Sensing/FallDownStateDetector.cpp
    src/bh_src/Src/Modules/Sensing/GroundContactDetector.cpp
    src/bh_src/Src/Modules/Sensing/InertiaSensorCalibrator.cpp
    src/bh_src/Src/Modules/Sensing/InertiaSensorFilter.cpp
    src/bh_src/Src/Modules/Sensing/InertiaSensorInspector.cpp
    src/bh_src/Src/Modules/Sensing/JointFilter.cpp
    src/bh_src/Src/Modules/Sensing/RobotModelProvider.cpp
    src/bh_src/Src/Modules/Sensing/SensorFilter.cpp
    src/bh_src/Src/Modules/Sensing/TorsoMatrixProvider.cpp

)

set( srcs_motion_cognition   
    src/bh_src/Src/Platform/Camera.h
    src/bh_src/Src/Platform/SystemCall.h
    src/bh_src/Src/Platform/BHAssert.h
    src/bh_src/Src/Platform/linux/BHAssert.cpp
    src/bh_src/Src/Platform/linux/SystemCall.cpp
    src/bh_src/Src/Platform/linux/NaoCamera.h
    src/bh_src/Src/Platform/linux/NaoCamera.cpp
    src/bh_src/Src/Platform/File.h
    src/bh_src/Src/Platform/UdpComm.h
    src/bh_src/Src/Platform/Win32Linux/UdpComm.cpp
    src/bh_src/Src/Platform/Win32Linux/File.cpp
   src/bh_src/Src/Representations/BehaviorControl/BehaviorControlOutput.h
   src/bh_src/Src/Representations/Configuration/BehaviorConfiguration.h
   src/bh_src/Src/Representations/Configuration/CameraCalibration.h
   src/bh_src/Src/Representations/Configuration/CameraSettings.cpp
   src/bh_src/Src/Representations/Configuration/ColorTable64.cpp
   src/bh_src/Src/Representations/Configuration/DamageConfiguration.h
   src/bh_src/Src/Representations/Configuration/FieldDimensions.cpp
   src/bh_src/Src/Representations/Configuration/JointCalibration.h
   src/bh_src/Src/Representations/Configuration/MassCalibration.h
   src/bh_src/Src/Representations/Configuration/RobotDimensions.h
   src/bh_src/Src/Representations/Configuration/SensorCalibration.h
   src/bh_src/Src/Representations/Infrastructure/BoardInfo.h
   src/bh_src/Src/Representations/Infrastructure/FrameInfo.h
   src/bh_src/Src/Representations/Infrastructure/GameInfo.h
   src/bh_src/Src/Representations/Infrastructure/CameraInfo.cpp
   src/bh_src/Src/Representations/Infrastructure/Image.cpp
   src/bh_src/Src/Representations/Infrastructure/TeamInfo.cpp
   src/bh_src/Src/Representations/Infrastructure/TeamMateData.h
   src/bh_src/Src/Representations/Infrastructure/JointData.h
   src/bh_src/Src/Representations/Infrastructure/RoboCupGameControlData.h
   src/bh_src/Src/Representations/Infrastructure/SensorData.h
   src/bh_src/Src/Representations/Infrastructure/JointData.h
   src/bh_src/Src/Representations/Infrastructure/KeyStates.h
   src/bh_src/Src/Representations/Infrastructure/LEDRequest.h
   src/bh_src/Src/Representations/Infrastructure/RobotInfo.h
   src/bh_src/Src/Representations/Infrastructure/SensorData.h
   src/bh_src/Src/Representations/Infrastructure/TeamInfo.cpp
   src/bh_src/Src/Representations/Infrastructure/TeamMateData.h
   src/bh_src/Src/Representations/Infrastructure/SoundRequest.h
   src/bh_src/Src/Representations/Modeling/ArmContactModel.h
   src/bh_src/Src/Representations/Modeling/BallHypotheses.h
   src/bh_src/Src/Representations/Modeling/CombinedWorldModel.cpp
   src/bh_src/Src/Representations/Modeling/FallDownState.cpp
   src/bh_src/Src/Representations/Modeling/FieldCoverage.cpp
   src/bh_src/Src/Representations/Modeling/GlobalFieldCoverage.cpp
   src/bh_src/Src/Representations/Modeling/GroundTruthResult.h
   src/bh_src/Src/Representations/Modeling/RobotPose.cpp
   src/bh_src/Src/Representations/Modeling/RobotsModel.cpp
   src/bh_src/Src/Representations/Modeling/ObstacleModel.cpp
   src/bh_src/Src/Representations/Modeling/SSLVisionData.cpp
   src/bh_src/Src/Representations/Modeling/FreePartOfOpponentGoalModel.cpp
   src/bh_src/Src/Representations/MotionControl/BikeEngineOutput.h
   src/bh_src/Src/Representations/MotionControl/BikeRequest.cpp
   src/bh_src/Src/Representations/MotionControl/HeadAngleRequest.h
   src/bh_src/Src/Representations/MotionControl/HeadJointRequest.h
   src/bh_src/Src/Representations/MotionControl/MotionInfo.h
   src/bh_src/Src/Representations/MotionControl/MotionRequest.cpp
   src/bh_src/Src/Representations/MotionControl/MotionSelection.cpp
   src/bh_src/Src/Representations/MotionControl/OdometryData.h
   src/bh_src/Src/Representations/MotionControl/SpecialActionRequest.h
   src/bh_src/Src/Representations/MotionControl/SpecialActionsOutput.h
   src/bh_src/Src/Representations/MotionControl/WalkingEngineOutput.h
   src/bh_src/Src/Representations/MotionControl/WalkingEngineStandOutput.h
   src/bh_src/Src/Representations/Perception/BallPercept.cpp
   src/bh_src/Src/Representations/MotionControl/WalkRequest.h
   src/bh_src/Src/Representations/Perception/BallSpots.h
   src/bh_src/Src/Representations/Perception/CameraMatrix.cpp
   src/bh_src/Src/Representations/Perception/ColorClassImage.cpp
   src/bh_src/Src/Representations/Perception/BodyContour.cpp
   src/bh_src/Src/Representations/Perception/ImageCoordinateSystem.cpp
   src/bh_src/Src/Representations/Perception/ImageInfo.h
   src/bh_src/Src/Representations/Perception/LinePercept.cpp
   src/bh_src/Src/Representations/Perception/LineSpots.h
   src/bh_src/Src/Representations/Perception/GoalPercept.cpp
   src/bh_src/Src/Representations/Perception/RegionPercept.cpp
   src/bh_src/Src/Representations/Perception/RobotPercept.h
   src/bh_src/Src/Representations/Perception/TeamMarkerSpots.cpp
   src/bh_src/Src/Representations/Sensing/GroundContactState.h
   src/bh_src/Src/Representations/Sensing/InertiaSensorData.h
   src/bh_src/Src/Representations/Sensing/OrientationData.h
   src/bh_src/Src/Representations/Sensing/RobotModel.cpp
   src/bh_src/Src/Representations/Sensing/TorsoMatrix.h
    src/bh_src/Src/Tools/Boundary.h
    src/bh_src/Src/Tools/Enum.cpp
    src/bh_src/Src/Tools/Global.cpp
    src/bh_src/Src/Tools/ForwardKinematic.h
    src/bh_src/Src/Tools/InverseKinematic.h
    src/bh_src/Src/Tools/Range.h
    src/bh_src/Src/Tools/Sha1.cpp
    src/bh_src/Src/Tools/RingBuffer.h
    src/bh_src/Src/Tools/RingBufferWithSum.h
    src/bh_src/Src/Tools/SampleSet.h
    #src/bh_src/Src/Tools/Settings.cpp
    #src/bh_src/Src/Tools/Settings.h
    src/bh_src/Src/Tools/Team.h
    src/bh_src/Src/Tools/Configuration/ConfigMap.cpp
    src/bh_src/Src/Tools/Configuration/ConfigMapParser.cpp
    src/bh_src/Src/Tools/Debugging/Debugging.h
    src/bh_src/Src/Tools/Debugging/DebugRequest.h
    src/bh_src/Src/Tools/Debugging/Asserts.h
    src/bh_src/Src/Tools/ImageProcessing/BresenhamLineScan.cpp
    src/bh_src/Src/Tools/ImageProcessing/ColorModelConversions.h
    src/bh_src/Src/Tools/Math/Common.h
    src/bh_src/Src/Tools/Math/Covariance.h
    src/bh_src/Src/Tools/Math/GaussianTable.h
    src/bh_src/Src/Tools/Math/GaussianDistribution3D.cpp
    src/bh_src/Src/Tools/Math/Probabilistics.h
    src/bh_src/Src/Tools/Math/Geometry.cpp   
    src/bh_src/Src/Tools/Math/Kalman.h
    src/bh_src/Src/Tools/Math/Matrix.cpp
    src/bh_src/Src/Tools/Math/Matrix2x2.h
    src/bh_src/Src/Tools/Math/Matrix3x3.h
    src/bh_src/Src/Tools/Math/MVTools.cpp
    src/bh_src/Src/Tools/Math/Pose2D.cpp
    src/bh_src/Src/Tools/Math/Pose3D.h
    src/bh_src/Src/Tools/Math/Random.h
    src/bh_src/Src/Tools/Math/RotationMatrix.cpp
    src/bh_src/Src/Tools/Math/Vector.h
    src/bh_src/Src/Tools/Math/Vector2.h
    src/bh_src/Src/Tools/Math/Vector3.h
    src/bh_src/Src/Tools/Math/YaMatrix.h
    src/bh_src/Src/Tools/Optimization/ParticleSwarm.cpp
    src/bh_src/Src/Tools/ProcessFramework/GameHandler.cpp
    src/bh_src/Src/Tools/Streams/InOut.cpp
    src/bh_src/Src/Tools/Streams/InStreams.cpp
    src/bh_src/Src/Tools/Streams/OutStreams.cpp
    src/bh_src/Src/Tools/Streams/Streamable.cpp
    src/bh_src/Src/Tools/Streams/StreamHandler.cpp
)


#find_package( Threads REQUIRED )

#include_directories(/usr/include)


include_directories(src/bh_src/Src)
include_directories(${NAO_PATH}/../include)



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


rosbuild_add_executable(bh_motion src/motionNode.cpp ${srcs_motion} ${srcs_motion_cognition} )
rosbuild_link_boost( bh_motion thread date_time system)  
target_link_libraries( bh_motion pthread rt)      

rosbuild_add_executable(exampleNode src/exampleNode.cpp)

#add_library( uchilerobotcomm SHARED ${srcs_uchrobotcomm})
#target_link_libraries( uchilerobotcomm boost_thread boost_data boost_system pthread rt ${ALCOMMON_LIBRARIES})

rosbuild_add_library( uchilerobotcomm ${srcs_uchrobotcomm})
rosbuild_link_boost( uchilerobotcomm thread date_time system)
target_link_libraries( uchilerobotcomm pthread rt ${ALCOMMON_LIBRARIES})



