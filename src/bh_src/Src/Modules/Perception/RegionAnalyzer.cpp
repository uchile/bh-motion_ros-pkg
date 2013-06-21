/**
* @file RegionAnalyzer.cpp
* @author jeff
*/

#include "RegionAnalyzer.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"

#define TOTALE_GRUETZE 42345645.0f

RegionAnalyzer::RegionAnalyzer()
{
  /*
  InConfigMap stream("regionAnalyzer.cfg");
  ASSERT(stream.exists());
  stream >> parameters;
*/
    parameters.minLineSize                   = 10;
    parameters.minLineSegmentCount           = 2;
    parameters.maxLineNghbNoneSize           = 550;
    parameters.maxLineNghbNoneRatio          = 0.5;
    parameters.minLineNghbGreenAboveSize     = 30;
    parameters.minLineNghbGreenBelowSize     = 30;
    parameters.minLineNghbGreenSideSize      = 20;
    parameters.maxLineNghbGreySkip           = 2;
    parameters.minLineSingleSegmentSize      = 300;
    parameters.minEccentricityOfBallRegion   = -100;
    parameters.minRobotRegionSize            = 100;
    parameters.maxRobotRegionAlphaDiff       = 0.7853981639744828;
    parameters.minRobotWidthRatio            = 2;

}

void RegionAnalyzer::update(BallSpots& ballSpots,
                            const CameraMatrix& theCameraMatrix,
                            const RegionPercept& theRegionPercept)
{
  //MODIFY("parameters:RegionAnalyzer", parameters);
  /*
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:Line", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:robots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenBelow", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenAbove", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenLeft", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RegionAnalyzer:greenRight", "drawingOnImage");
  */

  ballSpots.ballSpots.clear();
  lineSpots.spots.clear();
  lineSpots.nonLineSpots.clear();

  if(theCameraMatrix.isValid)
    analyzeRegions(ballSpots, theRegionPercept);

}

int RegionAnalyzer::getGreenBelow(const RegionPercept::Region* region, const RegionPercept& theRegionPercept)
{
  int greenBelow = 0;
  for(vector<RegionPercept::Segment*>::const_iterator child = region->childs.begin(); child != region->childs.end(); child++)
  {
    if((*child) - theRegionPercept.segments == theRegionPercept.segmentsCounter - 1)
      continue;

    RegionPercept::Segment* next = (*child) + 1;
    const int childEndY = (*child)->y + (*child)->length;

    //find next green segment
    while(next - theRegionPercept.segments < theRegionPercept.segmentsCounter &&
          next->x == (*child)->x &&
          next->color != ColorClasses::green)
      next++;

    if(next - theRegionPercept.segments < theRegionPercept.segmentsCounter && next->x == (*child)->x)
    {
      //implizit durch die if-Bedingung und die while-Bedingung
      //if(next->color == ColorClasses::green)
      if(next->y - childEndY <= parameters.maxLineNghbGreySkip)
      {
        greenBelow += next->length;
        //LINE("module:RegionAnalyzer:greenBelow", next->x, next->y, next->x, next->y + next->length, 0, Drawings::ps_solid, ColorClasses::green);
      }
    }
  }
  ASSERT(greenBelow >= 0);
  return greenBelow;
}

int RegionAnalyzer::getGreenAbove(const RegionPercept::Region* region, const RegionPercept& theRegionPercept)
{
  int greenAbove = 0;
  for(vector<RegionPercept::Segment*>::const_iterator child = region->childs.begin(); child != region->childs.end(); child++)
  {
    if((*child) == theRegionPercept.segments)
      continue;

    RegionPercept::Segment* next = (*child) - 1;

    //find previous green segment
    while(next->x == (*child)->x &&
          next > theRegionPercept.segments &&
          next->color != ColorClasses::green)
      next--;

    ASSERT(next >= theRegionPercept.segments);

    if(next->color == ColorClasses::green && next->x == (*child)->x)
    {
      //if(next->color == ColorClasses::green)
      if((*child)->y - (next->y + next->length) <= parameters.maxLineNghbGreySkip)
      {
        greenAbove += next->length;
        //LINE("module:RegionAnalyzer:greenAbove", next->x, next->y, next->x, (*child)->y, 0, Drawings::ps_solid, ColorClasses::blue);
      }
    }
  }
  ASSERT(greenAbove >= 0);
  return greenAbove;
}

int RegionAnalyzer::getGreenRight(const RegionPercept::Region* region, const RegionPercept& theRegionPercept)
{
  const RegionPercept::Segment* nextColumnSegment;
  const int& gridStepSize = theRegionPercept.gridStepSize;
  int greenRight = 0;
  for(vector<RegionPercept::Segment*>::const_iterator seg_iter = region->childs.begin();
      seg_iter != region->childs.end();
      seg_iter++)
  {
    nextColumnSegment = *seg_iter + 1;
    //find first segment in next column (and skip that ones only used for extendend ball finding).
    while(nextColumnSegment < theRegionPercept.segments + theRegionPercept.segmentsCounter &&
          nextColumnSegment->x < (*seg_iter)->x + gridStepSize)
      nextColumnSegment++;

    if(nextColumnSegment >= theRegionPercept.segments + theRegionPercept.segmentsCounter ||
       nextColumnSegment->x > (*seg_iter)->x + gridStepSize)
      continue;

    ASSERT(nextColumnSegment->x == (*seg_iter)->x + gridStepSize);

    //find first segment which ends after seg_iter->y
    while(nextColumnSegment < theRegionPercept.segments + theRegionPercept.segmentsCounter &&
          nextColumnSegment->x == (*seg_iter)->x + gridStepSize &&
          nextColumnSegment->y + nextColumnSegment->length <= (*seg_iter)->y)
      nextColumnSegment++;

    if(nextColumnSegment >= theRegionPercept.segments + theRegionPercept.segmentsCounter ||
       nextColumnSegment->x > (*seg_iter)->x + gridStepSize)
      continue;

    //find first segment touching seg_iter and check from there on whether there are green segments
    while(nextColumnSegment->y < (*seg_iter)->y + (*seg_iter)->length &&
          nextColumnSegment->x == (*seg_iter)->x + gridStepSize &&
          nextColumnSegment < theRegionPercept.segments + theRegionPercept.segmentsCounter)
    {
      if(nextColumnSegment->color == ColorClasses::green)
      {
        const int start = max<int>(nextColumnSegment->y, (*seg_iter)->y);
        const int end = min<int>(nextColumnSegment->y + nextColumnSegment->length, (*seg_iter)->y + (*seg_iter)->length);
        ASSERT(end >= start);
        greenRight += end - start;
        //LINE("module:RegionAnalyzer:greenRight", nextColumnSegment->x, start, nextColumnSegment->x, end, 0, Drawings::ps_solid, ColorClasses::blue);
      }
      nextColumnSegment++;
    }
  }
  ASSERT(greenRight >= 0);
  return greenRight;
}


int RegionAnalyzer::getGreenLeft(const RegionPercept::Region* region, const RegionPercept& theRegionPercept)
{
  const RegionPercept::Segment* lastColumnSegment;
  const int& gridStepSize = theRegionPercept.gridStepSize;
  int greenLeft = 0;
  for(vector<RegionPercept::Segment*>::const_iterator seg_iter = region->childs.begin();
      seg_iter != region->childs.end();
      seg_iter++)
  {
    lastColumnSegment = *seg_iter - 1;
    //find first segment in previous column (and skip that ones only used for extendend ball finding).
    while(lastColumnSegment >= theRegionPercept.segments &&
          lastColumnSegment->x > (*seg_iter)->x - gridStepSize)
      lastColumnSegment--;

    if(lastColumnSegment < theRegionPercept.segments ||
       lastColumnSegment->x < (*seg_iter)->x - gridStepSize)
      continue;

    ASSERT(lastColumnSegment->x == (*seg_iter)->x - gridStepSize);

    //find first segment which begins before seg_iter->y + set_iter->length
    while(lastColumnSegment >= theRegionPercept.segments &&
          lastColumnSegment->x == (*seg_iter)->x - gridStepSize &&
          lastColumnSegment->y >= (*seg_iter)->y + (*seg_iter)->length)
      lastColumnSegment--;

    if(lastColumnSegment < theRegionPercept.segments ||
       lastColumnSegment->x < (*seg_iter)->x - gridStepSize)
      continue;

    //find first segment touching seg_iter and check from there on whether there are green segments
    while(lastColumnSegment >= theRegionPercept.segments &&
          lastColumnSegment->y + lastColumnSegment->length > (*seg_iter)->y &&
          lastColumnSegment->x == (*seg_iter)->x - gridStepSize)
    {
      if(lastColumnSegment->color == ColorClasses::green)
      {
        const int start = max<int>(lastColumnSegment->y, (*seg_iter)->y);
        const int end = min<int>(lastColumnSegment->y + lastColumnSegment->length, (*seg_iter)->y + (*seg_iter)->length);
        ASSERT(end >= start);
        greenLeft += end - start;
        //LINE("module:RegionAnalyzer:greenLeft", lastColumnSegment->x, start, lastColumnSegment->x, end, 0, Drawings::ps_solid, ColorClasses::blue);
      }
      lastColumnSegment--;
    }
  }
  ASSERT(greenLeft >= 0);
  return greenLeft;
}

bool RegionAnalyzer::isLine(const RegionPercept::Region* region, float& direction, LineSpots::LineSpot& spot, const RegionPercept& theRegionPercept)
{
  bool ret = true;
  if(region->size < parameters.minLineSize ||
     (region->childs.size() < (size_t)parameters.minLineSegmentCount &&
      region->size < parameters.minLineSingleSegmentSize))
  {
    /*
      COMPLEX_DRAWING("module:RegionAnalyzer:Line",
    {
      Vector2<int> c = region->getCenter();
      if(region->size < parameters.minLineSize)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x, c.y, 150, ColorClasses::black, region->size << "<s");
      }
      if(region->childs.size() < (size_t)parameters.minLineSegmentCount)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x, c.y - 5, 150, ColorClasses::black, (unsigned) region->childs.size() << "<c");
      }

    });
    */

    ret = false;
  }

  int neighborNoneSize = 0;
  for(vector<RegionPercept::Region*>::const_iterator neighbor = region->neighborRegions.begin(); neighbor != region->neighborRegions.end(); neighbor++)
  {
    switch((*neighbor)->color)
    {
    case ColorClasses::none:
      neighborNoneSize += (*neighbor)->size;
      break;
    default:
      break;
    }
  }

  if(neighborNoneSize > parameters.maxLineNghbNoneSize ||
     ((float)neighborNoneSize / region->size) > parameters.maxLineNghbNoneRatio)
  {
    /*
    COMPLEX_DRAWING("module:RegionAnalyzer:Line",
    {
      Vector2<int> c = region->getCenter();
      if(neighborNoneSize > parameters.maxLineNghbNoneSize)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorRGBA(255, 0, 0), neighborNoneSize);
        ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x, c.y - 5, 0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
      }
      if((float)neighborNoneSize / region->size > parameters.maxLineNghbNoneRatio)
      {
        DRAWTEXT("module:RegionAnalyzer:Line", c.x + 5, c.y, 150, ColorRGBA(255, 0, 0), neighborNoneSize / (float)region->size);
        ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x + 5, c.y, 0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
      }

    });
    */
    ret = false;
  }

  //calculate haupttraegheitsachsen and schwerpunkt from moments
  const int m00 = region->calcMoment00(),
            m10 = region->calcMoment10(),
            m01 = region->calcMoment01();
  const int x_s = m10 / m00,
            y_s = m01 / m00;
  const float m20 = (float) region->calcCMoment20(x_s),
              m02 = (float) region->calcCMoment02(y_s),
              m11 = (float) region->calcCMoment11(x_s, y_s);
  const float cm20 = m20 / m00,
              cm02 = m02 / m00;
  //CROSS("module:RegionAnalyzer:Line", x_s, y_s, 2, 2, Drawings::ps_solid, ColorClasses::black);
  spot.x_s = x_s;
  spot.y_s = y_s;
  float alpha = 0;
  if(cm20 != cm02)
  {
    //for an (yet) unknown reason we need to add an offset of 90 degrees
    alpha = .5f * atan2(-2 * ((float)m11), (float)(m02 - m20)) + (float)pi_2;
    spot.alpha = alpha;
  }
  else
  {
    spot.alpha = TOTALE_GRUETZE;
    ret = false;
  }

  bool verticalRegion = false;
  if(alpha > pi_2 - pi_4 && alpha < pi_2 + pi_4)
  {
    verticalRegion = true;
    //CROSS("module:RegionAnalyzer:Line", x_s, y_s, 2, 2, Drawings::ps_solid, ColorClasses::green);
  }

  if(verticalRegion)
  {
    int greenLeftSize = getGreenLeft(region, theRegionPercept);
    int greenRightSize = getGreenRight(region, theRegionPercept);

    if(greenLeftSize < parameters.minLineNghbGreenSideSize || greenRightSize < parameters.minLineNghbGreenSideSize)
    {
      /*
      COMPLEX_DRAWING("module:RegionAnalyzer:Line",
      {
        Vector2<int> c = region->getCenter();
        if(greenLeftSize < parameters.minLineNghbGreenSideSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x - 5, c.y, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x - 3, c.y + 5, 150, ColorClasses::black, greenLeftSize);
        }
        if(greenRightSize < parameters.minLineNghbGreenSideSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x + 5, c.y, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorClasses::black, greenRightSize);
        }
      });
      */
      ret = false;
    }
  }
  else
  {
    int greenAboveSize = getGreenAbove(region, theRegionPercept);
    int greenBelowSize = getGreenBelow(region, theRegionPercept);

    if(greenAboveSize < parameters.minLineNghbGreenAboveSize ||
       greenBelowSize < parameters.minLineNghbGreenBelowSize)
    {
        /*
      COMPLEX_DRAWING("module:RegionAnalyzer:Line",
      {
        Vector2<int> c = region->getCenter();
        if(greenAboveSize < parameters.minLineNghbGreenAboveSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y, c.x, c.y - 5, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorClasses::green, greenAboveSize);
        }
        if(greenBelowSize < parameters.minLineNghbGreenBelowSize)
        {
          ARROW("module:RegionAnalyzer:Line", c.x, c.y - 5, c.x, c.y, 0, Drawings::ps_solid, ColorClasses::green);
          DRAWTEXT("module:RegionAnalyzer:Line", c.x + 3, c.y - 5, 150, ColorClasses::green, greenBelowSize);
        }
      });
      */
      ret = false;
    }

  }

  //cut alpha_len and alpha_len2 to the maximum distance a point has to
  //the haupttraegheitsachse
  const Vector2<> achse1(cos(alpha), sin(alpha)),
        achse2(cos(alpha + pi_2), sin(alpha + pi_2));
  float len_max = 0,
        len_min = 0,
        len_max2 = 0,
        len_min2 = 0;
  for(vector<RegionPercept::Segment*>::const_iterator child = region->childs.begin(); child != region->childs.end(); child++)
  {
    const RegionPercept::Segment* seg = *child;
    const Vector2<> childRelToSwp(float(seg->x - spot.x_s), float(seg->y - spot.y_s)),
          child2RelToSwp(float(seg->x - spot.x_s), float(seg->y + seg->length - spot.y_s));

    const float dist = childRelToSwp * achse1;
    if(dist > len_max)
      len_max = dist;
    else if(dist < len_min)
      len_min = dist;

    const float dist2 = child2RelToSwp * achse1;
    if(dist2 > len_max)
      len_max = dist2;
    else if(dist2 < len_min)
      len_min = dist2;

    const float dist3 = childRelToSwp * achse2;
    if(dist3 > len_max2)
      len_max2 = dist3;
    else if(dist3 < len_min2)
      len_min2 = dist3;

    const float dist4 = child2RelToSwp * achse2;
    if(dist4 > len_max2)
      len_max2 = dist4;
    else if(dist4 < len_min2)
      len_min2 = dist4;

  }
  spot.alpha_len = abs(len_min) < len_max ? abs(len_min) : len_max;
  spot.alpha_len2 = abs(len_min2) < len_max2 ? abs(len_min2) : len_max2;
  if(spot.alpha_len2 == 0)
    spot.alpha_len2 = (float) theRegionPercept.gridStepSize;
  if(spot.alpha_len == 0) //I don't think this can happen, but.....
    spot.alpha_len = (float) theRegionPercept.gridStepSize;

  spot.p1 = Vector2<int>((int)(x_s + cos(spot.alpha) * len_max), (int)(y_s + sin(spot.alpha) * len_max));
  spot.p2 = Vector2<int>((int)(x_s + cos(spot.alpha) * len_min), (int)(y_s + sin(spot.alpha) * len_min));
  direction = alpha;

  return ret;
}

void RegionAnalyzer::analyzeRegions(BallSpots& ballSpots, const RegionPercept& theRegionPercept)
{
  LineSpots::LineSpot lineSpot;

  for(const RegionPercept::Region* region = theRegionPercept.regions; region - theRegionPercept.regions < theRegionPercept.regionsCounter; region++)
  {
    if(region->childs.size() == 0)
      continue;

    switch(region->color)
    {
    case ColorClasses::white:
    {
      float dir;
      if(isLine(region, dir, lineSpot, theRegionPercept))
      {
        lineSpots.spots.push_back(lineSpot);
      }
      else
      {
        if(lineSpot.alpha != TOTALE_GRUETZE)
        {
          //LINE("module:RegionAnalyzer:robots", lineSpot.x_s, lineSpot.y_s, lineSpot.x_s + (int)(cos(lineSpot.alpha + pi_2) * lineSpot.alpha_len2), lineSpot.y_s + (int)(sin(lineSpot.alpha + pi_2)*lineSpot.alpha_len2), 0, Drawings::ps_solid, ColorClasses::black);
          //ARROW("module:RegionAnalyzer:robots", lineSpot.p1.x, lineSpot.p1.y, lineSpot.p2.x, lineSpot.p2.y, 0, Drawings::ps_solid, ColorClasses::black);
          while(lineSpot.alpha > pi)
            lineSpot.alpha -= 2 * pi;
          while(lineSpot.alpha < -pi)
            lineSpot.alpha += 2 * pi;

          if(region->size > parameters.minRobotRegionSize && abs(abs(lineSpot.alpha) - pi_2) < parameters.maxRobotRegionAlphaDiff && lineSpot.alpha_len / lineSpot.alpha_len2 > parameters.minRobotWidthRatio)
          {
            Vector2<int> c = lineSpot.p1.y > lineSpot.p2.y ? lineSpot.p1 : lineSpot.p2;//region->getCenter();
            Vector2<int> c2 = lineSpot.p1.y > lineSpot.p2.y ? lineSpot.p2 : lineSpot.p1;//region->getCenter();
            //CROSS("module:RegionAnalyzer:robots", c.x, c.y, 4, 2, Drawings::ps_solid, ColorClasses::red);
            //CROSS("module:RegionAnalyzer:robots", c2.x, c2.y, 4, 2, Drawings::ps_solid, ColorClasses::red);
            //DRAWTEXT("module:RegionAnalyzer:robots", c.x, c.y, 150, ColorClasses::black, region->size);
            LineSpots::NonLineSpot spot;
            spot.p1 = c;
            spot.p2 = c2;
            spot.size = region->size;
            lineSpots.nonLineSpots.push_back(spot);
          }
        }
      }
    }
    break;
    case ColorClasses::orange:
    {
      Vector2<int> c = region->getCenter();

      int m00 = region->calcMoment00(),
          m10 = region->calcMoment10(),
          m01 = region->calcMoment01(),
          swpX = m10 / m00,
          swpY = m01 / m00,
          m20 = region->calcCMoment20(swpX);

      float m02 = region->calcCMoment02(swpY),
            m11 = region->calcCMoment11(swpX, swpY);

      float eccentricity;
      //balls far away might just be scanned by one grid, so eccentricity would be 0
      if(region->childs.size() > 1)
        eccentricity = 1.0f - (sqr(m20 - m02) + 4.0f * sqr(m11)) / (sqr(m20 + m02));
      else
        //TODO: think about balls at imageborder and blurred balls
        //so they are marked with:
        eccentricity = parameters.minEccentricityOfBallRegion + 0.2f;
      if(eccentricity > parameters.minEccentricityOfBallRegion)
        ballSpots.addBallSpot(c.x, c.y, eccentricity);

      break;
    }
    default:
      break;
    }
  }
}

//MAKE_MODULE(RegionAnalyzer, Perception)