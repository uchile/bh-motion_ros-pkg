/**
* @file BallPerceptor.cpp
* This file declares a module that provides a ball percept without using color tables.
* The ball center / radius calculation algorithm is based on the BallSpecialist in GT2005.
* @author Colin Graf
*/

#include "BallPerceptor.h"
//#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Matrix.h"
//#include "Tools/Settings.h"
#include "Platform/File.h"
#include <iostream>

//MAKE_MODULE(BallPerceptor, Perception)

void BallPerceptor::init(const FieldDimensions& theFieldDimensions)
{
  sqrMaxBallDistance = float(Vector2<int>(theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOwnFieldBorder,
                                          theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosRightFieldBorder).squareAbs());

  /*
  cout << "theFieldDimensions.xPosOpponentFieldBorder: " << theFieldDimensions.xPosOpponentFieldBorder << endl;
  cout << "theFieldDimensions.xPosOwnFieldBorder:      " << theFieldDimensions.xPosOwnFieldBorder      << endl;
  cout << "theFieldDimensions.yPosLeftFieldBorder:     " << theFieldDimensions.yPosLeftFieldBorder     << endl;
  cout << "theFieldDimensions.yPosRightFieldBorder:    " << theFieldDimensions.yPosRightFieldBorder    << endl;
 */

  ConfigMap params;
  params["clippingApproxRadiusScale"] << 2.f;
  params["clippingApproxRadiusPixelBonus"] << 2.5f;

  params["scanMaxColorDistance"] << (unsigned int) 30;
  params["scanPixelTolerance"] << (unsigned int) 2;

  params["refineMaxPixelCount"] << (unsigned int) 2;
  params["refineMaxColorDistance"] << (unsigned int) 40;

  params["checkMaxRadiusDifference"] << 1.6f;
  params["checkMinRadiusDifference"] << 0.9f;
  params["checkMinRadiusPixelBonus"] << 6.f;

  params["checkOutlineRadiusScale"] << 1.1f;
  params["checkOutlineRadiusPixelBonus"] << 2.f;

  //params.read(Global::getSettings().expandRobotLocationFilename("ballPerceptor.cfg"));
  params.read("/home/nao/.config/naoqi/Data/Config/Locations/Default/Robots/Nao/ballPerceptor.cfg");

  params["clippingApproxRadiusScale"] >> p.clippingApproxRadiusScale;
  params["clippingApproxRadiusPixelBonus"] >> p.clippingApproxRadiusPixelBonus;

  params["scanMaxColorDistance"] >> p.scanMaxColorDistance;
  params["scanPixelTolerance"] >> p.scanPixelTolerance;

  params["refineMaxPixelCount"] >> p.refineMaxPixelCount;
  params["refineMaxColorDistance"] >> p.refineMaxColorDistance;

  params["checkMaxRadiusDifference"] >> p.checkMaxRadiusDifference;
  params["checkMinRadiusDifference"] >> p.checkMinRadiusDifference;
  params["checkMinRadiusPixelBonus"] >> p.checkMinRadiusPixelBonus;

  params["checkOutlineRadiusScale"] >> p.checkOutlineRadiusScale;
  params["checkOutlineRadiusPixelBonus"] >> p.checkOutlineRadiusPixelBonus;
}

void BallPerceptor::update(BallPercept& ballPercept,
                           const BallSpots& theBallSpots,
                           const ImageCoordinateSystem& theImageCoordinateSystem,
                           const CameraInfo& theCameraInfo,
                           const CameraMatrix& theCameraMatrix,
                           const FieldDimensions& theFieldDimensions,
                           const Image& theImage)
{
  //MODIFY("module:BallPerceptor:parameters", p);

    /*
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:field", "drawingOnField");
  DECLARE_PLOT("module:BallPerceptor:angle");
  */

    init(theFieldDimensions);

    //std::cout << "BallPerceptor::update" << std::endl;

  for(std::vector<BallSpot>::const_iterator it = theBallSpots.ballSpots.begin(), end = theBallSpots.ballSpots.end(); it != end; ++it)
  {
    const BallSpot& ballSpot = *it;

    // TODO: prefer large spots

    if(!checkBallSpot(ballSpot, theImageCoordinateSystem, theCameraInfo, theCameraMatrix, theFieldDimensions)) // step 1
    {
        std::cout << "checkBallSpot" << std::endl;
        continue;
    }
    if(!searchBallPoints(ballSpot, theImage)) // step 2
    {
        std::cout << "searchBallPoints" << std::endl;
        continue;
    }
    if(!checkBallPoints()) // step 3
    {
        std::cout << "checkBallPoints" << std::endl;
        continue;
    }
    if(!calculateBallInImage()) // step 4
    {
        std::cout << "calculateBallInImage" << std::endl;
        continue;
    }
    if(!checkBallInImage(theImage)) // step 5
    {
        std::cout << "checkBallInImage" << std::endl;
        continue;
    }
    if(!calculateBallOnField(theImageCoordinateSystem, theCameraInfo, theFieldDimensions, theCameraMatrix)) // step 6
    {
        std::cout << "calculateBallOnField" << std::endl;
        continue;
    }
    if(!checkBallOnField()) // step 7
    {
        std::cout << "checkBallOnField" << std::endl;
        continue;
    }

    ballPercept.positionInImage = center;
    ballPercept.radiusInImage = radius;
    ballPercept.ballWasSeen = true;
    ballPercept.relativePositionOnField = Vector2<>(usedCenterOnField.x, usedCenterOnField.y);

    //PLOT("module:BallPerceptor:angle", toDegrees(ballPercept.relativePositionOnField.angle()));

    return;
  }

  ballPercept.ballWasSeen = false;
}

bool BallPerceptor::checkBallSpot(const BallSpot& ballSpot, const ImageCoordinateSystem& theImageCoordinateSystem,
                                  const CameraInfo& theCameraInfo,
                                  const CameraMatrix& theCameraMatrix,
                                  const FieldDimensions& theFieldDimensions)
{
  // calculate an approximation of the radius based on bearing distance of the ball spot
  const Vector2<int>& spot = ballSpot.position;
  Vector2<> correctedStart = theImageCoordinateSystem.toCorrected(spot);
  Vector3<> cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedStart.x, theCameraInfo.opticalCenter.y - correctedStart.y);
  Vector3<> unscaledField = theCameraMatrix.rotation * cameraToStart;
  if(unscaledField.z >= 0.f)
  {
      //cout << "unscaledField.z: "<< unscaledField.z << endl;
      return false; // above horizon
  }
  const float scaleFactor = (theCameraMatrix.translation.z - theFieldDimensions.ballRadius) / unscaledField.z;

  /*
  cout << "theFieldDimensions.ballRadius: " << theFieldDimensions.ballRadius << endl;
  cout << "theCameraMatrix.translation.z: " << theCameraMatrix.translation.z << endl;
  cout << "unscaledField.x:               " << unscaledField.x << endl;
  cout << "unscaledField.z:               " << unscaledField.z << endl;
  cout << "unscaledField.y:               " << unscaledField.y << endl;
  cout << "scaleFactor:                   " << scaleFactor << endl;
  */

  cameraToStart *= scaleFactor;
  unscaledField *= scaleFactor;
  if(Vector2<>(unscaledField.x, unscaledField.y).squareAbs() > sqrMaxBallDistance)
  {
      /*
      cout << "Vector2<>(unscaledField.x, unscaledField.y).squareAbs(): "<< Vector2<>(unscaledField.x, unscaledField.y).squareAbs() << endl;
      cout << "sqrMaxBallDistance: " << sqrMaxBallDistance << endl;
      cout << "too far away" << endl;
      */
      return false; // too far away
  }
  cameraToStart.y += cameraToStart.y > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
  cameraToStart /= scaleFactor;
  approxRadius1 = abs(theImageCoordinateSystem.fromCorrectedApprox(Vector2<int>(int(theCameraInfo.opticalCenter.x - cameraToStart.y), int(theCameraInfo.opticalCenter.y - cameraToStart.z))).x - spot.x);

  return true;
}

bool BallPerceptor::searchBallPoints(const BallSpot& ballSpot,
                                     const Image& theImage)
{
  Vector2<int> start = ballSpot.position;
  const float approxDiameter = approxRadius1 * p.clippingApproxRadiusScale + p.clippingApproxRadiusPixelBonus;
  int halfApproxRadius = int(approxRadius1 * 0.5f);

  //CROSS("module:BallPerceptor:image", start.x, start.y, 2, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0, 0x6a));
  //CIRCLE("module:BallPerceptor:image", start.x, start.y, approxDiameter, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0, 0x6a), Drawings::bs_null, ColorRGBA());

  // try to improve the start point
  int maxColorDistance = p.scanMaxColorDistance;
  int resolutionWidth = theImage.cameraInfo.resolutionWidth;
  int resolutionHeight = theImage.cameraInfo.resolutionHeight;
  startPixel = theImage.image[start.y][start.x];
  Vector2<int> preScanPoints[4] =
  {
    Vector2<int>(start.x + halfApproxRadius, start.y + halfApproxRadius),
    Vector2<int>(start.x - halfApproxRadius, start.y - halfApproxRadius),
    Vector2<int>(start.x + halfApproxRadius, start.y - halfApproxRadius),
    Vector2<int>(start.x - halfApproxRadius, start.y + halfApproxRadius)
  };
  bool preScanResults[4];
  for(int i = 0; i < 4; ++i)
  {
    if(preScanPoints[i].x < 0 || preScanPoints[i].x >= resolutionWidth ||
       preScanPoints[i].y < 0 || preScanPoints[i].y >= resolutionHeight)
    {
      i -= i % 2;
      preScanResults[i++] = false;
      preScanResults[i] = false;
    }
    else
    {
      const Image::Pixel& pixel = theImage.image[preScanPoints[i].y][preScanPoints[i].x];
      preScanResults[i] = abs(startPixel.cb - pixel.cb) + abs(startPixel.cr - pixel.cr) < maxColorDistance;
    }
  }
  if(preScanResults[0] != preScanResults[1] && preScanResults[2] != preScanResults[3])
  {
    start = Vector2<int>();
    if(preScanResults[0])
      start += preScanPoints[0];
    else
      start += preScanPoints[1];
    if(preScanResults[2])
      start += preScanPoints[2];
    else
      start += preScanPoints[3];
    start /= 2;
  }
  else if(preScanResults[0] != preScanResults[1])
    start = preScanResults[0] ? preScanPoints[0] : preScanPoints[1];
  else if(preScanResults[2] != preScanResults[3])
    start = preScanResults[2] ? preScanPoints[2] : preScanPoints[3];


  //CROSS("module:BallPerceptor:image", start.x, start.y, 2, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
  //CIRCLE("module:BallPerceptor:image", start.x, start.y, approxDiameter, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0), Drawings::bs_null, ColorRGBA());

  // prepare scans
  totalPixelCount = 0;
  totalCbSum = 0;
  totalCrSum = 0;

  // vertical scan
  if(!searchBallPoint(start, startPixel, Vector2<int>(0, 1), approxDiameter, ballPoints[0], theImage) ||
     !searchBallPoint(start, startPixel, Vector2<int>(0, -1), approxDiameter, ballPoints[4], theImage))
    return false;
  if(ballPoints[0].atBorder && ballPoints[4].atBorder)
    return false; // too large
  else if(ballPoints[0].atBorder)
  {
    start.y = ballPoints[4].point.y + int(approxRadius1);
    if(start.y > ballPoints[0].point.y - 1)
      start.y = ballPoints[0].point.y - 1;
  }
  else if(ballPoints[4].atBorder)
  {
    start.y = ballPoints[0].point.y - int(approxRadius1);
    if(start.y < ballPoints[4].point.y + 1)
      start.y = ballPoints[4].point.y + 1;
  }
  else
    start.y = (ballPoints[0].point.y + ballPoints[4].point.y) / 2;

  // horizontal scan
  if(!searchBallPoint(start, startPixel, Vector2<int>(1, 0), approxDiameter, ballPoints[2], theImage) ||
     !searchBallPoint(start, startPixel, Vector2<int>(-1, 0), approxDiameter, ballPoints[6], theImage))
    return false;
  if(ballPoints[2].atBorder && ballPoints[6].atBorder)
    return false; // too large
  else if(ballPoints[2].atBorder)
  {
    start.x = ballPoints[6].point.x + int(approxRadius1);
    if(start.x > ballPoints[2].point.x - 1)
      start.x = ballPoints[2].point.x - 1;
  }
  else if(ballPoints[6].atBorder)
  {
    start.x = ballPoints[2].point.x - int(approxRadius1);
    if(start.x < ballPoints[6].point.x + 1)
      start.x = ballPoints[6].point.x + 1;
  }
  else
    start.x = (ballPoints[2].point.x + ballPoints[6].point.x) / 2;
  approxCenter2 = start;

  // maybe repeat vertical and horizontal scans
  int skipArea = std::min(ballPoints[0].point.y - ballPoints[4].point.y, ballPoints[2].point.x - ballPoints[6].point.x) / 4;
  float maxLength = approxDiameter - skipArea;
  if(abs(start.x - ballPoints[0].start.x) > halfApproxRadius)
  {
    if(!searchBallPoint(start + Vector2<int>(0, skipArea), startPixel, Vector2<int>(0, 1), maxLength, ballPoints[0], theImage) ||
       !searchBallPoint(start + Vector2<int>(0, -skipArea), startPixel, Vector2<int>(0, -1), maxLength, ballPoints[4], theImage))
      return false;
  }
  if(abs(start.y - ballPoints[2].start.y) > halfApproxRadius)
  {
    if(!searchBallPoint(start + Vector2<int>(skipArea, 0), startPixel, Vector2<int>(1, 0), maxLength, ballPoints[2], theImage) ||
       !searchBallPoint(start + Vector2<int>(-skipArea, 0), startPixel, Vector2<int>(-1, 0), maxLength, ballPoints[6], theImage))
      return false;
  }

  // diagonal scans
  skipArea = std::min(ballPoints[0].point.y - ballPoints[4].point.y, ballPoints[2].point.x - ballPoints[6].point.x) / 4;
  maxLength = approxDiameter - 1.41421356f * skipArea;
  if(!searchBallPoint(start + Vector2<int>(skipArea, skipArea), startPixel, Vector2<int>(1, 1), maxLength, ballPoints[1], theImage) ||
     !searchBallPoint(start + Vector2<int>(-skipArea, -skipArea), startPixel, Vector2<int>(-1, -1), maxLength, ballPoints[5], theImage) ||
     !searchBallPoint(start + Vector2<int>(skipArea, -skipArea), startPixel, Vector2<int>(1, -1), maxLength, ballPoints[3], theImage) ||
     !searchBallPoint(start + Vector2<int>(-skipArea, skipArea), startPixel, Vector2<int>(-1, 1), maxLength, ballPoints[7], theImage))
    return false;

  // improve ball points
  if(totalPixelCount == 0)
    return false;
  int cbAvg = totalCbSum / totalPixelCount;
  int crAvg = totalCrSum / totalPixelCount;
  int refineMaxPixelCount = p.refineMaxPixelCount;
  int refineMaxColorDistance = p.refineMaxColorDistance;
  for(unsigned j = 0; j < sizeof(ballPoints) / sizeof(*ballPoints); ++j)
  {
    BallPoint& ballPoint = ballPoints[j];
    const Vector2<int>& step = ballPoint.step;
    Vector2<int> pos = ballPoint.point;
    int i = 0;
    for(; i < refineMaxPixelCount; ++i)
    {
      pos += step;
      if(pos.x < 0 || pos.x >= resolutionWidth ||
         pos.y < 0 || pos.y >= resolutionHeight)
        break;
      if(abs(theImage.image[pos.y][pos.x].cb - cbAvg) + abs(theImage.image[pos.y][pos.x].cr - crAvg) > refineMaxColorDistance)
        break;
    }
    if(i)
      ballPoint.point = pos - step;
    ballPoint.pointf = Vector2<float>(float(ballPoint.point.x), float(ballPoint.point.y));
  }


  return true;
}

bool BallPerceptor::searchBallPoint(const Vector2<int>& start, Image::Pixel startPixel, const Vector2<int>& step, float maxLength, BallPoint& ballPoint,
                                    const Image& theImage)
{
  ASSERT(step.x == 0 || step.x == 1 || step.x == -1);
  ASSERT(step.y == 0 || step.y == 1 || step.y == -1);

  Vector2<int> pos = start;
  Vector2<int> lastValidPos = pos;
  unsigned int overtime = 0;
  int cbSum = startPixel.cb;
  int crSum = startPixel.cr;
  int pixelCount = 1;
  int cb, cr;

  ballPoint.step = step;
  ballPoint.atBorder = false;
  ballPoint.start = start;

  int maxColorDistance = p.scanMaxColorDistance;
  unsigned int maxOvertime = p.scanPixelTolerance;
  int resolutionWidth = theImage.cameraInfo.resolutionWidth;
  int resolutionHeight = theImage.cameraInfo.resolutionHeight;
  int stepAbs1024 = (step.x == 0 || step.y == 0) ? 1024 : 1448; // 1448 = sqrt(2) * 1024
  int maxAbs1024 = int(maxLength * 1024.f); // + 1024;
  int length1024 = 0;
  for(;;)
  {
    pos += step;
    length1024 += stepAbs1024;

    if(length1024 > maxAbs1024)
    {
      if(overtime > 0)
        break;
      //LINE("module:BallPerceptor:image", start.x, start.y, pos.x, pos.y, 1, Drawings::ps_solid, ColorRGBA(0, 0, 0xff, 0x70));
      return false;
    }

    if(pos.x < 0 || pos.x >= resolutionWidth ||
       pos.y < 0 || pos.y >= resolutionHeight)
    {
      ballPoint.atBorder = true;
      break;
    }

    cb = theImage.image[pos.y][pos.x].cb;
    cr = theImage.image[pos.y][pos.x].cr;
    if(abs(cb - cbSum / pixelCount) + abs(cr - crSum / pixelCount) > maxColorDistance)
    {
      if(overtime == 0)
        lastValidPos = pos - step;
      ++overtime;
      if(overtime <= maxOvertime)
        continue;
      break;
    }
    cbSum += cb;
    crSum += cr;
    ++pixelCount;
    overtime = 0;
  }

  ballPoint.point = lastValidPos;

  if(pixelCount > 0)
  {
    --pixelCount;
    cbSum -= startPixel.cb;
    crSum -= startPixel.cr;
    totalPixelCount += pixelCount;
    totalCbSum += cbSum;
    totalCrSum += crSum;
  }

  //LINE("module:BallPerceptor:image", ballPoint.start.x, ballPoint.start.y, ballPoint.point.x, ballPoint.point.y, 1, Drawings::ps_solid, ColorRGBA(0, 0, 0xff));

  return true;
}

bool BallPerceptor::checkBallPoints()
{
  // find "valid" ball points
  validBallPoints = 0;
  static const int countOfBallPoints = sizeof(ballPoints) / sizeof(*ballPoints);
  for(int i = 0; i < countOfBallPoints; ++i)
  {
    BallPoint& ballPoint(ballPoints[i]);
    ballPoint.isValid = !ballPoint.atBorder && ballPoint.point != ballPoint.start;
    if(ballPoint.isValid)
    {
      ++validBallPoints;
      //DOT("module:BallPerceptor:image", ballPoint.point.x, ballPoint.point.y, ColorRGBA(0, 0xff, 0, 0x70), ColorRGBA(0, 0xff, 0, 0x70));
    }
  }
  if(validBallPoints < 3)
    return false;

  // find duplicated ball points (may occur in small balls)
  for(int i = 0; i < countOfBallPoints; ++i)
  {
    BallPoint& ballPoint(ballPoints[i]);
    if(ballPoint.isValid)
    {
      const BallPoint& nextBallPoint(ballPoints[(i + 1) % countOfBallPoints]);
      if(nextBallPoint.isValid && ballPoint.point == nextBallPoint.point)
      {
        ballPoint.isValid = false;
        --validBallPoints;
      }
    }
  }
  if(validBallPoints < 3)
    return false;

  // drop mismatching ball points
  while(validBallPoints > 3)
  {
    Vector2<> preCenter;
    float preRadius;
    if(!getBallFromBallPoints(preCenter, preRadius))
      return false;

    float minDist = 0;
    BallPoint* minDistBallPoint = 0;
    for(int i = 0; i < countOfBallPoints; ++i)
    {
      BallPoint& ballPoint(ballPoints[i]);
      if(ballPoint.isValid)
      {
        float dist = (ballPoint.pointf - preCenter).squareAbs();
        if(!minDistBallPoint || dist < minDist)
        {
          minDist = dist;
          minDistBallPoint = &ballPoint;
        }
      }
    }
    minDistBallPoint->isValid = false;
    --validBallPoints;

    if((preRadius - (sqrt(minDist) + 2.f)) / preRadius < 0.1f)
      break;
  }

  /*
  COMPLEX_DRAWING("module:BallPerceptor:image",
  {
    for(BallPoint* ballPoint = ballPoints, * end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
      if(ballPoint->isValid)
      {
        DOT("module:BallPerceptor:image", ballPoint->point.x, ballPoint->point.y, ColorRGBA(0, 0xff, 0), ColorRGBA(0, 0, 0));
      }
  });
  */

  return true;
}


bool BallPerceptor::getBallFromBallPoints(Vector2<>& center, float& radius) const
{
  int Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;

  for(const BallPoint* ballPoint = ballPoints, * end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
    if(ballPoint->isValid)
    {
      int x = ballPoint->point.x;
      int y = ballPoint->point.y;
      int xx = x * x;
      int yy = y * y;
      int z = xx + yy;
      Mx += x;
      My += y;
      Mxx += xx;
      Myy += yy;
      Mxy += x * y;
      Mz += z;
      Mxz += x * z;
      Myz += y * z;
    }

  // Construct and solve matrix
  // Result will be center and radius of ball in theImage.
  Matrix<3, 3> M(
    Vector<3>(static_cast<float>(Mxx), static_cast<float>(Mxy), static_cast<float>(Mx)),
    Vector<3>(static_cast<float>(Mxy), static_cast<float>(Myy), static_cast<float>(My)),
    Vector<3>(static_cast<float>(Mx), static_cast<float>(My), static_cast<float>(validBallPoints)));
  Vector<3> v(static_cast<float>(-Mxz), static_cast<float>(-Myz), static_cast<float>(-Mz));

  Vector<3> BCD;
  if(!M.solve(v, BCD))
    return false;

  center.x = BCD[0] * -0.5f;
  center.y = BCD[1] * -0.5f;
  float radicand = BCD[0] * BCD[0] / 4.0f + BCD[1] * BCD[1] / 4.0f - BCD[2];
  if(radicand <= 0.0f)
    return false;
  radius = sqrt(radicand);
  return true;
}

bool BallPerceptor::calculateBallInImage()
{
  if(!getBallFromBallPoints(center, radius))
    return false;
  return true;
}

bool BallPerceptor::checkBallInImage(const Image& theImage)
{
  // check if the ball covers approxCenter2
  if((Vector2<int>(int(center.x + 0.5), int(center.y + 0.5)) - approxCenter2).squareAbs() > sqr(radius))
    return false;

  // check ball radius
  if(radius > approxRadius1 * p.checkMaxRadiusDifference || radius + p.checkMinRadiusPixelBonus < approxRadius1 * p.checkMinRadiusDifference)
    return false;

  //
  float noBallRadius = radius * p.checkOutlineRadiusScale + p.checkOutlineRadiusPixelBonus;
  //CIRCLE("module:BallPerceptor:image", center.x, center.y, noBallRadius, 1, Drawings::ps_solid, ColorRGBA(0xff, 0xff, 0xff), Drawings::bs_null, ColorRGBA());
  Vector2<int> center32(int(center.x * 32.f), int(center.y * 32.f));
  int noBallRadius32 = int(noBallRadius * 32.f);
  int noBallDRadius32 = int(noBallRadius * 32.f / 1.41421356f);
  Vector2<int> noBallPoints32[8 + 4 * 4] =
  {
    Vector2<int>(center32.x + noBallRadius32, center32.y),
    Vector2<int>(center32.x - noBallRadius32, center32.y),
    Vector2<int>(center32.x, center32.y + noBallRadius32),
    Vector2<int>(center32.x, center32.y - noBallRadius32),
    Vector2<int>(center32.x + noBallDRadius32, center32.y + noBallDRadius32),
    Vector2<int>(center32.x - noBallDRadius32, center32.y - noBallDRadius32),
    Vector2<int>(center32.x + noBallDRadius32, center32.y - noBallDRadius32),
    Vector2<int>(center32.x - noBallDRadius32, center32.y + noBallDRadius32),
  };
  int noBallPointCount = 8;
  int resolutionWidth = theImage.cameraInfo.resolutionWidth;
  int resolutionHeight = theImage.cameraInfo.resolutionHeight;
  int borderDists[2] = {0, 3};
  if(center32.x + noBallRadius32 >= resolutionWidth * 32)
    for(int i = 0; i < 2; ++i)
    {
      int x = resolutionWidth - 1 - borderDists[i];
      float d = sqrt(float(sqr(noBallRadius) - sqr(x - center.x)));
      noBallPoints32[noBallPointCount++] = Vector2<int>(x * 32, int((center.y + d) * 32.f));
      noBallPoints32[noBallPointCount++] = Vector2<int>(x * 32, int((center.y - d) * 32.f));
    }
  if(center32.y + noBallRadius32 >= resolutionHeight * 32)
    for(int i = 0; i < 2; ++i)
    {
      int y = resolutionHeight - 1 - borderDists[i];
      float d = sqrt(float(sqr(noBallRadius) - sqr(y - center.y)));
      noBallPoints32[noBallPointCount++] = Vector2<int>(int((center.x + d) * 32.f), y * 32);
      noBallPoints32[noBallPointCount++] = Vector2<int>(int((center.x - d) * 32.f), y * 32);
    }
  if(center32.x - noBallRadius32 < 0)
    for(int i = 0; i < 2; ++i)
    {
      int x = borderDists[i];
      float d = sqrt(float(sqr(noBallRadius) - sqr(x - center.x)));
      noBallPoints32[noBallPointCount++] = Vector2<int>(x * 32, int((center.y + d) * 32.f));
      noBallPoints32[noBallPointCount++] = Vector2<int>(x * 32, int((center.y - d) * 32.f));
    }
  if(center32.y - noBallRadius32 < 0)
    for(int i = 0; i < 2; ++i)
    {
      int y = borderDists[i];
      float d = sqrt(float(sqr(noBallRadius) - sqr(y - center.y)));
      noBallPoints32[noBallPointCount++] = Vector2<int>(int((center.x + d) * 32.f), y * 32);
      noBallPoints32[noBallPointCount++] = Vector2<int>(int((center.x - d) * 32.f), y * 32);
    }
  int maxColorDistance = p.scanMaxColorDistance;
  //int cbStart = startPixel.cb, crStart = startPixel.cr;
  int cbStart = totalCbSum / totalPixelCount, crStart = totalCrSum / totalPixelCount;
  for(int i = 0; i < noBallPointCount; ++i)
  {
    Vector2<int> pos(noBallPoints32[i] / 32);
    if(pos.x < 0 || pos.x >= resolutionWidth ||
       pos.y < 0 || pos.y >= resolutionHeight)
      continue;
    //DOT("module:BallPerceptor:image", pos.x, pos.y, ColorRGBA(0xff, 0xff, 0xff), ColorRGBA(0, 0, 0));

    if(abs(theImage.image[pos.y][pos.x].cb - cbStart) + abs(theImage.image[pos.y][pos.x].cr - crStart) <= maxColorDistance)
      return false;
  }

  return true;
}

bool BallPerceptor::calculateBallOnField(const ImageCoordinateSystem& theImageCoordinateSystem,
                                         const CameraInfo& theCameraInfo,
                                         const FieldDimensions& theFieldDimensions,
                                         const CameraMatrix& theCameraMatrix)
{
  const Vector2<> correctedCenter = theImageCoordinateSystem.toCorrected(center);
  Vector3<> cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedCenter.x, theCameraInfo.opticalCenter.y - correctedCenter.y);
  cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / radius);
  Vector3<> rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
  sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
  bearingBasedCenterOnField =  theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z - theFieldDimensions.ballRadius) / rotatedCameraToBall.z);

  //CIRCLE("module:BallPerceptor:field", sizeBasedCenterOnField.x, sizeBasedCenterOnField.y, theFieldDimensions.ballRadius, 1, Drawings::ps_solid, ColorRGBA(0, 0, 0xff), Drawings::bs_null, ColorRGBA());
  //CIRCLE("module:BallPerceptor:field", bearingBasedCenterOnField.x, bearingBasedCenterOnField.y, theFieldDimensions.ballRadius, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0), Drawings::bs_null, ColorRGBA());

  usedCenterOnField = rotatedCameraToBall.z < 0 ? bearingBasedCenterOnField : sizeBasedCenterOnField;

  return true;
}

bool BallPerceptor::checkBallOnField()
{
  // check difference between both transformations?

  return true;
}

