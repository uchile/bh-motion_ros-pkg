/**
* @file CameraMatrixProvider.h
* This file declares a class to calculate the position of the camera for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"


/*
MODULE(CameraMatrixProvider)
  REQUIRES(FrameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(CameraCalibration)
  REQUIRES(RobotDimensions)
  REQUIRES(FilteredJointData)
  REQUIRES(FilteredJointDataPrev)
  REQUIRES(MotionInfo)
  REQUIRES(FallDownState)
  REQUIRES(TorsoMatrix)
  REQUIRES(RobotCameraMatrix)
  REQUIRES(RobotCameraMatrixOther)
  REQUIRES(RobotCameraMatrixPrev)
  PROVIDES_WITH_MODIFY(CameraMatrixOther)
  PROVIDES(CameraMatrixPrev)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrix);
  USES(CameraMatrix)
  USES(FieldDimensions) // for debug drawing
  USES(RobotPose) // for debug drawing
  USES(CameraInfo) // for debug drawing
END_MODULE
*/

class CameraMatrixProvider //: public CameraMatrixProviderBase
{
public:
  void update(CameraMatrixOther& cameraMatrixOther,
              const TorsoMatrix& theTorsoMatrix,
              const RobotCameraMatrixOther& theRobotCameraMatrixOther,
              const CameraCalibration& theCameraCalibration,
              const MotionInfo& theMotionInfo,
              const FallDownState& theFallDownState,
              const FrameInfo& theFrameInfo,
              const FilteredJointData& theFilteredJointData,
              const RobotInfo& theRobotInfo);

  void update(CameraMatrixPrev& cameraMatrixPrev,
              const TorsoMatrix& theTorsoMatrix,
              const RobotCameraMatrixPrev& theRobotCameraMatrixPrev,
              const CameraCalibration& theCameraCalibration,
              const FrameInfo& theFrameInfo,
              const FilteredJointDataPrev theFilteredJointDataPrev);

  void update(CameraMatrix& cameraMatrix,
              const TorsoMatrix& theTorsoMatrix,
              const RobotCameraMatrix& theRobotCameraMatrix,
              const CameraCalibration& theCameraCalibration,
              const MotionInfo& theMotionInfo,
              const FallDownState& theFallDownState,
              const FrameInfo& theFrameInfo,
              const FilteredJointData& theFilteredJointData,
              const RobotInfo& theRobotInfo);

private:
  void camera2image(const Vector3<>& camera, Vector2<>& image, const CameraInfo& theCameraInfo) const;
  bool intersectLineWithCullPlane(const Vector3<>& lineBase, const Vector3<>& lineDir,
                                  Vector3<>& point) const;
  void drawFieldLines(const CameraMatrix& cameraMatrix, const RobotPose& theRobotPose, const FieldDimensions& theFieldDimensions, const CameraInfo& theCameraInfo) const;
};
