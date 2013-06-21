/**
* @file Modules/MotionControl/HeadMotionEngine.h
* This file declares a module that creates head joint angles from desired head motion.
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander H�rtl</A>
*/

#pragma once

#include "../../Tools/Math/Vector2.h"
//#include "Tools/Module/Module.h"
#include "../../Representations/MotionControl/HeadAngleRequest.h"
#include "../../Representations/MotionControl/HeadJointRequest.h"
#include "../../Representations/Configuration/RobotDimensions.h"
#include "../../Representations/Configuration/JointCalibration.h"
#include "../../Representations/Infrastructure/JointData.h"
#include "../../Representations/Infrastructure/FrameInfo.h"
#include "../../Tools/Math/Geometry.h"

/*MODULE(HeadMotionEngine)
  REQUIRES(HeadAngleRequest)
  REQUIRES(FilteredJointData)
  REQUIRES(RobotDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(JointCalibration)
  PROVIDES_WITH_MODIFY(HeadJointRequest)
END_MODULE
*/

class HeadMotionEngine//: public HeadMotionEngineBase
{
private:
  Vector2<> lastSpeed;
  unsigned int lastIteration;
  Geometry::Circle deathPoints[4];

  void init(const FrameInfo& theFrameInfo);

  /**
  * The update method to generate the head joint angles from desired head motion.
  */
public:
  void update(HeadJointRequest& headJointRequest
              ,const RobotDimensions& theRobotDimensions
              ,const JointCalibration& theJointCalibration
              ,const HeadAngleRequest& theHeadAngleRequest
              ,const FrameInfo& theFrameInfo
              ,const FilteredJointData& theFilteredJointData);
};
