/**
* @file CameraControlEngine.h
* To execute the current HeadMotionRequest the CameraControlEngine
* chooses one of the two cameras of the Nao and computes the appropriate
* angles for the head joints to execute the current HeadMotionRequest.
* @author Felix Wenk
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Perception/BodyContour.h"

/*MODULE(CameraControlEngine)
  REQUIRES(BodyContour)
  REQUIRES(FrameInfo)
  REQUIRES(RobotDimensions)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraMatrixOther)
  REQUIRES(CameraInfo)
  REQUIRES(FilteredJointData)
  REQUIRES(ImageInfo)
  REQUIRES(HeadJointRequest)
  REQUIRES(HeadMotionRequest)
  REQUIRES(HeadAngleRequest)
  REQUIRES(RobotCameraMatrix)
  REQUIRES(RobotCameraMatrixOther)
  REQUIRES(RobotPose)
  REQUIRES(TorsoMatrix)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageRequest)
  PROVIDES_WITH_MODIFY(HeadAngleRequest)
END_MODULE*/

class CameraControlEngine //: public CameraControlEngineBase
{
public:
  CameraControlEngine();

private:
  class Parameters : public Streamable
  {
  public:
    Parameters()
      : disableUpperCamera(false),
        disableLowerCamera(false),
        defaultHeadSpeed(3.4f)
    {}

    bool disableUpperCamera; /** true to disable. */
    bool disableLowerCamera; /** true to disable. */
    float defaultHeadSpeed;
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(disableUpperCamera);
      STREAM(disableLowerCamera);
      STREAM(defaultHeadSpeed);
      STREAM_REGISTER_FINISH();
    }
  };

  Parameters p;
  bool requestedLowerCamera;
  bool lastFrameLowerClipped;
  float minimumTiltAngle[2][120]; /* Maps camera selection (0: lower Camera) and pan angle [degree] to minimum reasonable tilt angle. [radians] */
  float minimumDistances[2][120]; /* Maps camera selection (0: lower Camera) and pan angle [degree] to minimum distance between the robot and a ground target. */

  void calculateYImageCoordinate(const Vector3<>& hip2Target,
                                 const Pose3D& transformMatrix, const Pose3D& transformMatrixOther,
                                 float& yImage, float& yImageOther
                                 , const FilteredJointData& theFilteredJointData
                                 , const CameraInfo& theCameraInfo);
  float calculateTiltAngles(const Vector3<>& hip2Target, bool lowerCamera, const RobotDimensions& theRobotDimensions);
  float calculateClippedAngle(const float yClipped, bool lowerCamera
                              , const CameraInfo& theCameraInfo
                              , const RobotCameraMatrix& theRobotCameraMatrix
                              , const RobotDimensions& theRobotDimensions);
  void recordMinimumTiltAngles(bool isImageFromLowerCamera
                               , const CameraInfo& theCameraInfo
                               , const BodyContour& theBodyContour
                               , const FilteredJointData& theFilteredJointData
                               , const RobotCameraMatrix& theRobotCameraMatrix
                               , const RobotDimensions& theRobotDimensions);
  inline bool cameraSwitchIsUseful(float yImageOther, const CameraInfo& theCameraInfo, const ImageInfo& theImageInfo);
public:
  void update(HeadAngleRequest& headAngleRequest
              , const ImageInfo& theImageInfo
              , const HeadMotionRequest& theHeadMotionRequest
              , const FilteredJointData& theFilteredJointData
              , const CameraInfo& theCameraInfo
              , const TorsoMatrix& theTorsoMatrix
              , const RobotDimensions& theRobotDimensions
              , const RobotCameraMatrix& theRobotCameraMatrix
              , const RobotCameraMatrixOther& theRobotCameraMatrixOther);
  void update(ImageRequest& imageRequest);
};
