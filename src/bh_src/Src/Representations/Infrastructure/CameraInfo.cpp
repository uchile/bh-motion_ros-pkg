/**
* @file CameraInfo.cpp
* Implementation of class CameraInfo
*/

#include "CameraInfo.h"

CameraInfo::CameraInfo()
{
  resolutionWidth  = cameraResolutionWidth;
  resolutionHeight = cameraResolutionHeight;

  openingAngleWidth   = 0.809833; // 45.08°- 0.78674f
  openingAngleHeight  = 0.607375; // 34.58°- 0.60349f

  focalLength = 272.0f;     // 385.54f;
  opticalCenter.x = 160.0f; // unchecked
  opticalCenter.y = 120.0f; // unchecked

  calcAdditionalConstants();
}

void CameraInfo::calcAdditionalConstants()
{
  focalLenPow2 = focalLength * focalLength;
  focalLenPow4 = focalLenPow2 * focalLenPow2;
  focalLengthInv = 1.0f / focalLength;
}
