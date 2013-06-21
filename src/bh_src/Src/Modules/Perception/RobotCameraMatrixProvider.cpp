/**
* @file RobotCameraMatrixProvider.cpp
* This file implements a class to calculate the position of the camera relative to the body for the Nao.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander H�rtl</a>
* @author Colin Graf
*/

#include "RobotCameraMatrixProvider.h"

//MAKE_MODULE(RobotCameraMatrixProvider, Perception);

void RobotCameraMatrixProvider::update(RobotCameraMatrixOther& robotCameraMatrixOther, const RobotDimensions& theRobotDimensions, const FilteredJointData& theFilteredJointData, const CameraCalibration& theCameraCalibration, const ImageInfo& theImageInfo)
{
  robotCameraMatrixOther.computeRobotCameraMatrix(theRobotDimensions, theFilteredJointData.angles[JointData::HeadYaw], theFilteredJointData.angles[JointData::HeadPitch], theCameraCalibration, theImageInfo.fromLowerCamera());
}

void RobotCameraMatrixProvider::update(RobotCameraMatrixPrev& robotCameraMatrixPrev, const RobotDimensions& theRobotDimensions, const FilteredJointData& theFilteredJointDataPrev, const CameraCalibration& theCameraCalibration, const ImageInfo& theImageInfo)
{
  robotCameraMatrixPrev.computeRobotCameraMatrix(theRobotDimensions, theFilteredJointDataPrev.angles[JointData::HeadYaw], theFilteredJointDataPrev.angles[JointData::HeadPitch], theCameraCalibration, !theImageInfo.fromLowerPrevCamera());
}

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix, const RobotDimensions& theRobotDimensions, const FilteredJointData& theFilteredJointData, const CameraCalibration& theCameraCalibration, const ImageInfo& theImageInfo)
{
  robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions, theFilteredJointData.angles[JointData::HeadYaw], theFilteredJointData.angles[JointData::HeadPitch], theCameraCalibration, !theImageInfo.fromLowerCamera());
}
