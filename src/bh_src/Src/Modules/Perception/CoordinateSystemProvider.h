/**
* @file CoordinateSystemProvider.h
* This file declares a module that provides coordinate systems.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ImageInfo.h"
//#include "Tools/Debugging/DebugImages.h"

/*
MODULE(CoordinateSystemProvider)
  REQUIRES(Image) // for debugging only
  REQUIRES(FrameInfo)
  REQUIRES(FilteredJointData) // for timeStamp only
  REQUIRES(FilteredJointDataPrev) // for timeStamp only
  REQUIRES(CameraInfo)
  REQUIRES(RobotDimensions)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraMatrixPrev)
  REQUIRES(CameraMatrixOther)
  REQUIRES(ImageInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageCoordinateSystem);
END_MODULE
*/

class CoordinateSystemProvider// : public CoordinateSystemProviderBase
{
private:
  void init(const CameraInfo& theCameraInfo);

public:
  void update(ImageCoordinateSystem& imageCoordinateSystem,
              const CameraMatrix& theCameraMatrix,
              const CameraInfo& theCameraInfo,
              const ImageInfo& theImageInfo,
              const CameraMatrixOther& theCameraMatrixOther,
              const CameraMatrixPrev& theCameraMatrixPrev,
              const RobotDimensions& theRobotDimensions,
              const FilteredJointData& theFilteredJointData,
              const FilteredJointDataPrev& theFilteredJointDataPrev,
              const FrameInfo& theFrameInfo,
              const Image& theImage);

private:
  /**
  * The method calculates the scaling factors for the distored image.
  * @param a The constant part of the equation for motion distortion will be returned here.
  * @param b The linear part of the equation for motion distortion will be returned here.
  */
  void calcScaleFactors(float& a, float& b,
                        const RobotDimensions& theRobotDimensions,
                        const FilteredJointData theFilteredJointData,
                        const FilteredJointDataPrev theFilteredJointDataPrev,
                        const FrameInfo& theFrameInfo,
                        const Image& theImage) const;

  //DECLARE_DEBUG_IMAGE(corrected);
  //DECLARE_DEBUG_IMAGE(horizonAligned);
};
