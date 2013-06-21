#include "SelfLocatorParameter.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Global.h"

SelfLocatorParameter::SelfLocatorParameter()
{
  load("selfloc.cfg");
}

void SelfLocatorParameter::load(const std::string& fileName)
{
  /*
  InConfigMap file(Global::getSettings().expandLocationFilename(fileName));
  ASSERT(file.exists());
  file >> *this;
  */
  this->numberOfSamples = 100;
  this->standardDeviationFieldLines = 1024;
  this->standardDeviationCorners = 512;
  this->standardDeviationGoalpostAngle = 0.2;
  this->standardDeviationGoalpostBearingDistance = 0.4;
  this->standardDeviationGoalpostSizeDistance = 0.2;
  this->standardDeviationGoalpostSampleBearingDistance = 150;
  this->standardDeviationGoalpostSampleSizeDistance = 150;
  this->standardDeviationCenterCircleAngle = 0.2;
  this->standardDeviationCenterCircleDistance = 0.4;
  this->alphaSlow = 0.0059;
  this->alphaFast = 0.006;
  this->resamplingThreshold = 4;
  this->translationNoise = 25;
  this->rotationNoise = 0.1;
  this->movedDistWeight = 0.002;
  this->movedAngleWeight = 1;
  this->majorDirTransWeight = 2;
  this->minorDirTransWeight = 1;
  this->maxCrossingLength = 180;
  this->numberOfObservations = 6;
  this->disableSensorResetting = false;
  this->considerGameState = true;
  this->knownStartPose = false;
  this->startPose = Pose2D(0,0,0);
  this->startPoseStandardDeviation = Pose2D(0,300,300);
  this->clipTemplateGeneration = false;
  this->clipTemplateGenerationRangeX = Range<>(0,0);
  this->clipTemplateGenerationRangeY = Range<>(0,0);
  this->templateMaxKeepTime = 5000;

}
