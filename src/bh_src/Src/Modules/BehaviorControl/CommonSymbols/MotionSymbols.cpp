/**
* @file MotionSymbols.cpp
* Implementation of class MotionSymbols.
* @author Max Risler
*/
#include "MotionSymbols.h"

void MotionSymbols::registerSymbols(xabsl::Engine& engine)
{
  // motion.type enumeration
  char s[256];
  char* dest = s + sprintf(s, "motion.type.");
  for(int i = 0; i < MotionRequest::numOfMotions; ++i)
  {
    getXabslString(dest, MotionRequest::getName((MotionRequest::Motion)i));
    engine.registerEnumElement("motion.type", s, i);
  }

  // motion.special_action enumeration
  dest = s + sprintf(s, "motion.special_action.");
  for(int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
  {
    getXabslString(dest, SpecialActionRequest::getName((SpecialActionRequest::SpecialActionID)i));
    engine.registerEnumElement("motion.special_action", s, i);
  }

  //walkKick.type enumeration
  dest = s + sprintf(s, "walkKick.type.");
  for(int i =0; i < WalkRequest::numOfKickTypes; ++i)
  {
	  getXabslString(dest, WalkRequest::getName((WalkRequest::KickType)i));
	  engine.registerEnumElement("walkKick.type", s, i);
  }
  // MotionRequest
  engine.registerEnumeratedOutputSymbol("motion.type", "motion.type", (int*)&motionRequest.motion);

  engine.registerBooleanOutputSymbol("motion.dribbling", &motionRequest.walkRequest.dribbling);

  engine.registerDecimalOutputSymbol("motion.walk_target.x", this, &MotionSymbols::setWalkTargetX,
                                     &MotionSymbols::getWalkTargetX);
  engine.registerDecimalOutputSymbol("motion.walk_target.y", this, &MotionSymbols::setWalkTargetY,
                                     &MotionSymbols::getWalkTargetY);
  engine.registerDecimalOutputSymbol("motion.walk_target.rot",
                                     this, &MotionSymbols::setWalkTargetRot, &MotionSymbols::getWalkTargetRot);
  engine.registerBooleanInputSymbol("motion.walk_target.reached", this, &MotionSymbols::getWalkTargetReached);
  engine.registerBooleanOutputSymbol("motion.walk_pedantic", &motionRequest.walkRequest.pedantic);

  engine.registerDecimalOutputSymbol("motion.walk_speed",
                                     this, &MotionSymbols::setWalkSpeed, &MotionSymbols::getWalkSpeed);

  engine.registerDecimalOutputSymbol<MotionSymbols>("motion.walk_speed.x", this, &MotionSymbols::setWalkSpeedX, &MotionSymbols::getWalkSpeedX);
  engine.registerDecimalOutputSymbol<MotionSymbols>("motion.walk_speed.y", this, &MotionSymbols::setWalkSpeedY, &MotionSymbols::getWalkSpeedY);
  engine.registerDecimalOutputSymbol<MotionSymbols>("motion.walk_speed.rot", this, &MotionSymbols::setWalkSpeedRot, &MotionSymbols::getWalkSpeedRot);

  engine.registerDecimalOutputSymbol<MotionSymbols>("motion.walk_speed.percentage.x", this, &MotionSymbols::setPercentageSpeedX, &MotionSymbols::getPercentageSpeedX);
  engine.registerDecimalOutputSymbol<MotionSymbols>("motion.walk_speed.percentage.y", this, &MotionSymbols::setPercentageSpeedY, &MotionSymbols::getPercentageSpeedY);
  engine.registerDecimalOutputSymbol<MotionSymbols>("motion.walk_speed.percentage.rot", this, &MotionSymbols::setPercentageSpeedRot, &MotionSymbols::getPercentageSpeedRot);

  engine.registerEnumeratedOutputSymbol("motion.special_action", "motion.special_action",
                                        this, &MotionSymbols::setSpecialAction, &MotionSymbols::getSpecialAction);
  engine.registerBooleanOutputSymbol("motion.special_action.mirror", &motionRequest.specialActionRequest.mirror);
  engine.registerEnumeratedOutputSymbol("motion.last_special_action", "motion.special_action", (int*)&lastSpecialAction); //??

  // MotionInfo
  engine.registerEnumeratedInputSymbol("executed_motion.type", "motion.type", (const int*)&motionInfo.motion);
  engine.registerEnumeratedInputSymbol("executed_motion.special_action", "motion.special_action", (const int*)&motionInfo.specialActionRequest.specialAction);
  engine.registerEnumeratedInputSymbol("executed_walkKick.type", "walkKick.type", (const int*)&motionInfo.walkRequest.kickType);
  engine.registerBooleanInputSymbol("executed_motion.special_action.mirror", &motionInfo.specialActionRequest.mirror);
  

  engine.registerEnumeratedOutputSymbol("walkKick.type","walkKick.type", this, &MotionSymbols::setWalkKickType, &MotionSymbols::getWalkKicktype);
  engine.registerDecimalOutputSymbol<MotionSymbols>("kickBallPosition.x", this, &MotionSymbols::setKickBallPositionX, &MotionSymbols::getKickBallPositionX);
  engine.registerDecimalOutputSymbol<MotionSymbols>("kickBallPosition.y", this, &MotionSymbols::setKickBallPositionY, &MotionSymbols::getKickBallPositionY);
  engine.registerDecimalOutputSymbol<MotionSymbols>("kickTarget.x", this, &MotionSymbols::setKickTargetX, &MotionSymbols::getKickTargetX);
  engine.registerDecimalOutputSymbol<MotionSymbols>("kickTarget.y", this, &MotionSymbols::setKickTargetY, &MotionSymbols::getKickTargetY);

}

void MotionSymbols::update()
{
  motionRequest.walkRequest.pedantic = false;
  motionRequest.walkRequest.mode = WalkRequest::targetMode;
}

void MotionSymbols::setWalkTargetX(float x)
{
  motionRequest.walkRequest.target.translation.x = x;
}

float MotionSymbols::getWalkTargetX()
{
  return motionRequest.walkRequest.target.translation.x;
}

void MotionSymbols::setWalkTargetY(float y)
{
  motionRequest.walkRequest.target.translation.y = y;
}

float MotionSymbols::getWalkTargetY()
{
  return motionRequest.walkRequest.target.translation.y;
}

void MotionSymbols::setWalkSpeed(float speed)
{
  Pose2D& targetSpeed = motionRequest.walkRequest.speed;
  speed /= 100.f;
  if(speed < 0)
    speed = 0.;
  if(speed > 1)
    speed = 1.;
  targetSpeed.translation.x = speed;
  targetSpeed.translation.y = speed;
  targetSpeed.rotation = speed;
}

float MotionSymbols::getWalkSpeed()
{
  return motionRequest.walkRequest.speed.translation.x * 100.f;
}

void MotionSymbols::setWalkSpeedX(float x)
{
  motionRequest.walkRequest.mode = WalkRequest::speedMode;
  motionRequest.walkRequest.speed.translation.x = x;
}

float MotionSymbols::getWalkSpeedX()
{
  return motionRequest.walkRequest.mode == WalkRequest::speedMode ? motionRequest.walkRequest.speed.translation.x : 0;
}

void MotionSymbols::setWalkSpeedY(float y)
{
  motionRequest.walkRequest.mode = WalkRequest::speedMode;
  motionRequest.walkRequest.speed.translation.y = y;
}

float MotionSymbols::getWalkSpeedY()
{
  return motionRequest.walkRequest.mode == WalkRequest::speedMode ? motionRequest.walkRequest.speed.translation.y : 0;
}

void MotionSymbols::setWalkSpeedRot(float rot)
{
  motionRequest.walkRequest.mode = WalkRequest::speedMode;
  motionRequest.walkRequest.speed.rotation = fromDegrees(rot);
}

float MotionSymbols::getWalkSpeedRot()
{
  return motionRequest.walkRequest.mode == WalkRequest::speedMode ? toDegrees(motionRequest.walkRequest.speed.rotation) : 0;
}

void MotionSymbols::setPercentageSpeedX(float percent)
{
  motionRequest.walkRequest.mode = WalkRequest::percentageSpeedMode;
  motionRequest.walkRequest.speed.translation.x = percent / 100.0f;
}

float MotionSymbols::getPercentageSpeedX()
{
  return motionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode ? motionRequest.walkRequest.speed.translation.x : 0;
}

void MotionSymbols::setPercentageSpeedY(float percent)
{
  motionRequest.walkRequest.mode = WalkRequest::percentageSpeedMode;
  motionRequest.walkRequest.speed.translation.y = percent / 100.0f;
}

float MotionSymbols::getPercentageSpeedY()
{
  return motionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode ? motionRequest.walkRequest.speed.translation.y : 0;
}

void MotionSymbols::setPercentageSpeedRot(float percent)
{
  motionRequest.walkRequest.mode = WalkRequest::percentageSpeedMode;
  motionRequest.walkRequest.speed.rotation = percent / 100.0f;
}

float MotionSymbols::getPercentageSpeedRot()
{
  return motionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode ? motionRequest.walkRequest.speed.rotation : 0;
}

void MotionSymbols::setWalkTargetRot(float rot)
{
  motionRequest.walkRequest.target.rotation = fromDegrees(rot);
}

float MotionSymbols::getWalkTargetRot()
{
  return toDegrees(motionRequest.walkRequest.target.rotation);
}

bool MotionSymbols::getWalkTargetReached()
{
  return motionInfo.motion == MotionRequest::walk &&
         motionInfo.walkRequest.mode == WalkRequest::targetMode &&
         motionInfo.walkRequest.target.translation.squareAbs() < 10.f * 10.f &&
         abs(motionInfo.walkRequest.target.rotation) < fromDegrees(1.f);
}

void MotionSymbols::setSpecialAction(int specialAction)
{
  SpecialActionRequest& specialActionRequest(motionRequest.specialActionRequest);
  specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID(specialAction);
}

int MotionSymbols::getSpecialAction()
{
  return int(motionRequest.specialActionRequest.specialAction);
}

void MotionSymbols::setWalkKickType(int walkKickType)
{
	WalkRequest& walkRequest(motionRequest.walkRequest);
	walkRequest.kickType = WalkRequest::KickType(walkKickType);
}
int MotionSymbols::getWalkKicktype()
{
	return int(motionRequest.walkRequest.kickType);
}
void MotionSymbols::setKickBallPositionX(float x)
{
	motionRequest.walkRequest.kickBallPosition.x = x;
}
float MotionSymbols::getKickBallPositionX()
{
	return motionRequest.walkRequest.kickBallPosition.x;
}
void MotionSymbols::setKickBallPositionY(float y)
{
	motionRequest.walkRequest.kickBallPosition.y = y;
}
float MotionSymbols::getKickBallPositionY()
{
	return motionRequest.walkRequest.kickBallPosition.y;
}
void MotionSymbols::setKickTargetX(float x)
{
	motionRequest.walkRequest.kickTarget.x = x/100;
}
float MotionSymbols::getKickTargetX()
{
	return motionRequest.walkRequest.kickTarget.x*100;
}
void MotionSymbols::setKickTargetY(float y)
{
	motionRequest.walkRequest.kickTarget.y = y/100;
}
float MotionSymbols::getKickTargetY()
{
	return motionRequest.walkRequest.kickTarget.y*100;
}