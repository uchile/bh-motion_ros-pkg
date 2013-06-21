/**
* @file BodyContourProvider.h
* This file implements a module that provides the contour of the robot's body in the image.
* The contour can be used to exclude the robot's body from image processing.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "BodyContourProvider.h"
//#include "Tools/Debugging/DebugDrawings.h"
//#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Configuration/ConfigMap.h"

#include "Tools/Math/Vector3.h"

BodyContourProvider::BodyContourProvider()
{
  /*
  InConfigMap stream("/home/nao/.config/naoqi/Data/Config/bodyContour.cfg");
  ASSERT(stream.exists());
  stream >> parameters;
  */
  std::vector<Vector3<> > pm;
  pm.resize(16);
  pm[0] = Vector3<>(-60,17,160);
  pm[1] = Vector3<>(-53,45,160);
  pm[2] = Vector3<>(-35,58,180);
  pm[3] = Vector3<>(-32,75,195);
  pm[4] = Vector3<>(32,75,195);
  pm[5] = Vector3<>(35,58,180);
  pm[6] = Vector3<>(53,45,160);
  pm[7] = Vector3<>(60,17,160);
  pm[8] = Vector3<>(60,-17,160);
  pm[9] = Vector3<>(53,-45,160);
  pm[10] = Vector3<>(35,-58,180);
  pm[11] = Vector3<>(32,-75,195);
  pm[12] = Vector3<>(-32,-75,195);
  pm[13] = Vector3<>(-35,-58,180);
  pm[14] = Vector3<>(-53,-45,160);
  pm[15] = Vector3<>(-60,-17,160);
  parameters.torso = pm;


  pm.resize(10);
  pm[0] = Vector3<>(15,-5,-60);
  pm[1] = Vector3<>(10,5,-60);
  pm[2] = Vector3<>(-25,5,-40);
  pm[3] = Vector3<>(-30,25,-30);
  pm[4] = Vector3<>(-40,25,-10);
  pm[5] = Vector3<>(-45,20,5);
  pm[6] = Vector3<>(-35,10,30);
  pm[7] = Vector3<>(-10,-5,42);
  pm[8] = Vector3<>(0,-22,47);
  pm[9] = Vector3<>(120,-22,47);
  parameters.upperArm = pm;

  pm.resize(3);
  pm[0] = Vector3<>(-20,30,25);
  pm[1] = Vector3<>(120,30,45);
  pm[2] = Vector3<>(120,-40,40);
  parameters.lowerArm = pm;

  pm.resize(6);
  pm[0] = Vector3<>(50,10,0);
  pm[1] = Vector3<>(50,50,-115);
  pm[2] = Vector3<>(-60,45,-160);
  pm[3] = Vector3<>(-60,-30,-160);
  pm[4] = Vector3<>(50,-40,-115);
  pm[5] = Vector3<>(50,-30,0);
  parameters.upperLeg1 = pm;

  pm.resize(2);
  pm[0] = Vector3<>(50,50,-115);
  pm[1] = Vector3<>(50,-40,-115);
  parameters.upperLeg2 = pm;

  pm.resize(6);
  pm[0] = Vector3<>(0,50,-35);
  pm[1] = Vector3<>(70,50,-35);
  pm[2] = Vector3<>(115,25,-35);
  pm[3] = Vector3<>(115,-25,-35);
  pm[4] = Vector3<>(70,-50,-35);
  pm[5] = Vector3<>(0,-50,-35);
  parameters.foot = pm;

}

void BodyContourProvider::update(BodyContour& bodyContour, const CameraInfo& theCameraInfo, const RobotModel& theRobotModel, const RobotCameraMatrix& theRobotCameraMatrix, const ImageCoordinateSystem& theImageCoordinateSystem)
{
  //DECLARE_DEBUG_DRAWING3D("module:BodyContourProvider:contour", "origin");

  //MODIFY("parameters:BodyContourProvider", parameters);

  bodyContour.cameraResolution.x = theCameraInfo.resolutionWidth;
  bodyContour.cameraResolution.y = theCameraInfo.resolutionHeight;
  bodyContour.lines.clear();

  add(Pose3D(), parameters.torso, 1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::bicepsLeft], parameters.upperArm, 1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::bicepsRight], parameters.upperArm, -1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::foreArmLeft], parameters.lowerArm, 1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::foreArmRight], parameters.lowerArm, -1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::thighLeft], parameters.upperLeg1, 1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::thighRight], parameters.upperLeg1, -1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::thighLeft], parameters.upperLeg2, 1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::thighRight], parameters.upperLeg2, -1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::footLeft], parameters.foot, 1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
  add(theRobotModel.limbs[MassCalibration::footRight], parameters.foot, -1, bodyContour, theRobotCameraMatrix, theCameraInfo, theImageCoordinateSystem);
}

void BodyContourProvider::add(const Pose3D& origin, const std::vector<Vector3<> >& c, float sign,
                              BodyContour& bodyContour,
                              const RobotCameraMatrix& theRobotCameraMatrix,
                              const CameraInfo& theCameraInfo,
                              const ImageCoordinateSystem& theImageCoordinateSystem)
{
  Vector2<int> q1,
               q2;
  Vector3<> p1 = origin * Vector3<>(c[0].x, c[0].y * sign, c[0].z);
  bool valid1 = Geometry::calculatePointInImage(p1, theRobotCameraMatrix, theCameraInfo, q1);
  if(valid1)
  {
    Vector2<> v = theImageCoordinateSystem.fromCorrectedApprox(q1);
    q1 = Vector2<int>(int(floor(v.x)), int(floor(v.y)));
  }

  for(unsigned i = 1; i < c.size(); ++i)
  {
    Vector3<> p2 = origin * Vector3<>(c[i].x, c[i].y * sign, c[i].z);
    bool valid2 = Geometry::calculatePointInImage(p2, theRobotCameraMatrix, theCameraInfo, q2);
    if(valid2)
    {
      Vector2<> v = theImageCoordinateSystem.fromCorrectedApprox(q2);
      q2 = Vector2<int>(int(floor(v.x)), int(floor(v.y)));
    }

    if(valid1 && valid2 &&
       (q1.y < theCameraInfo.resolutionHeight || q2.y < theCameraInfo.resolutionHeight) &&
       (q1.x >= 0 || q2.x >= 0) &&
       (q1.x < theCameraInfo.resolutionWidth || q2.x < theCameraInfo.resolutionWidth))
      bodyContour.lines.push_back(BodyContour::Line(q1, q2));

    q1 = q2;
    valid1 = valid2;

    //COMPLEX_DRAWING3D("module:BodyContourProvider:contour",
    //{
      //LINE3D("module:BodyContourProvider:contour", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, 1, ColorRGBA(255, 0, 0));
    //  p1 = p2;
    //});
  }
}

//MAKE_MODULE(BodyContourProvider, Perception)
