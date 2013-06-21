/**
* @file GoalPerceptor.h
* @author jeff
* @author moe
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Configuration/ColorTable64.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Storage.h"
//#include "Tools/Debugging/DebugImages.h"

#ifndef MAX_GOAL_SPOTS
#define MAX_GOAL_SPOTS 8
#endif

/*
MODULE(GoalPerceptor)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(Image)
  REQUIRES(ColorTable64)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(OwnTeamInfo)
  PROVIDES_WITH_DRAW(GoalPercept)
END_MODULE
*/

/**
 * @class GoalPerceptor
 */
class GoalPerceptor //: public GoalPerceptorBase
{
private:
  class Parameters : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(stepSize);
      STREAM(maxSkip);
      STREAM(minSegmentLength);
      STREAM(minYRunDiff);
      STREAM(minPostPixelHeight);
      STREAM(widthStepSize);
      STREAM(maxWidthErrorRatio);
      STREAM(minWidthErrorRatio);
      STREAM(maxWrongSizeRatio);
      STREAM(maxDistanceError);
      STREAM(maxHeadCloserThanFoot);
      STREAM(minCenterDistance);
      STREAM(maxGreenScan);
      STREAM(minGreenBelow);
      STREAM(greenScanOffset);
      STREAM_REGISTER_FINISH();
    }

  public:
    Parameters() :
      stepSize(2),
      maxSkip(4),
      minSegmentLength(2),
      minYRunDiff(4),
      minPostPixelHeight(40),
      minHeadVisibleOffset(5),
      widthStepSize(10),
      maxGreenScan(10),
      minGreenBelow(5),
      greenScanOffset(2),
      maxWidthErrorRatio(1.5f),
      minWidthErrorRatio(0.4f),
      maxWrongSizeRatio(0.25f),
      maxDistanceError(0.5f),
      maxHeadCloserThanFoot(500),
      minCenterDistance(20)
    {}
    int stepSize,
        maxSkip,
        minSegmentLength,
        minYRunDiff,
        minPostPixelHeight,
        minHeadVisibleOffset,
        widthStepSize,
        maxGreenScan,
        minGreenBelow,
        greenScanOffset;
    float maxWidthErrorRatio,
          minWidthErrorRatio,
          maxWrongSizeRatio,
          maxDistanceError,
          maxHeadCloserThanFoot,
          minCenterDistance;
  };

  class Spot
  {
  public:
    ColorClasses::Color color;
    Vector2<> head, foot;
    Vector2<> center;
    int avrgWidth;
    bool headVisible,
         footVisible,
         dead;
    Vector2<> onField;
    enum
    {
      unknown,
      left,
      right
    } side;
  };

  Parameters parameters; /**< The parameters of this module. */
  GoalPercept* percept;
  Storage<Spot, MAX_GOAL_SPOTS> spots;

  public:
  void update(GoalPercept& percept,
              const CameraMatrix& theCameraMatrix,
              const ImageCoordinateSystem& theImageCoordinateSystem,
              const CameraInfo& theCameraInfo,
              const Image& theImage,
              const ColorTable64& theColorTable64,
              const FieldDimensions& theFieldDimensions,
              const OwnTeamInfo& theOwnTeamInfo,
              const FrameInfo& theFrameInfo);

  private:
  void findSpots(const ImageCoordinateSystem& theImageCoordinateSystem,
                 const CameraInfo& theCameraInfo,
                 const Image& theImage,
                 const ColorTable64& theColorTable64);
  void checkSpotsWidth(const Image& theImage,
                       const ColorTable64& theColorTable64,
                       const CameraInfo& theCameraInfo,
                       const ImageCoordinateSystem& theImageCoordinateSystem,
                       const CameraMatrix& theCameraMatrix,
                       const FieldDimensions& theFieldDimensions);
  void checkFieldCoordinates(const ImageCoordinateSystem& theImageCoordinateSystem,
                             const CameraInfo& theCameraInfo,
                             const CameraMatrix& theCameraMatrix,
                             const Image& theImage,
                             const FieldDimensions& theFieldDimensions);
  void checkMinimumDistance();
  void resetPercept(const OwnTeamInfo& theOwnTeamInfo);
  void createPercept(const OwnTeamInfo& theOwnTeamInfo,
                     const FrameInfo& theFrameInfo);
  void checkGreenBelow(const CameraInfo& theCameraInfo, const Image& theImage, const ColorTable64& theColorTable64);

  int findBlueOrYellowRight(int x, int y, int xEnd, const Image& theImage, const ColorTable64& theColorTable64);
  int findGreenDown(int x, int y, int yEnd);
  int runRight(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip, const Image& theImage, const ColorTable64& theColorTable64);
  int runLeft(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip, const Image& theImage, const ColorTable64& theColorTable64);
  int runDown(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip, const Image& theImage, const ColorTable64& theColorTable64);
  int runUp(int x, int y, ColorClasses::Color col, int yEnd, int maxSkip, const Image& theImage, const ColorTable64& theColorTable64);
  inline ColorClasses::Color imageColor(int x, int y, const Image& theImage, const ColorTable64& theColorTable64);
  int calculateExpectedPixelWidth(Vector2<int> posImg,
                                  const ImageCoordinateSystem& theImageCoordinateSystem,
                                  const CameraMatrix& theCameraMatrix,
                                  const Image& theImage,
                                  const FieldDimensions& theFieldDimensions);
  Vector2<> getPointOnField(const Vector2<int> &p,
                            const ImageCoordinateSystem& theImageCoordinateSystem,
                            const CameraMatrix& theCameraMatrix,
                            const CameraInfo& theCameraInfo);
							  
  void calculateOwnGoalPose();
  void calculateOpponentGoalPose();
};
