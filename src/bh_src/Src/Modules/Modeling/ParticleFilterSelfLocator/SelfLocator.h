/**
* @file SelfLocator.h
* Declares a class that performs self-localization using a particle filter.
*
* The difference to the ParticleFilterSelfLocator is as follows:
*
* - May choose different implementations for pose calculation
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Tools/SampleSet.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/Configuration/BehaviorConfiguration.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotPoseHypotheses.h"
#include "Tools/RingBuffer.h"
#include "Tools/Math/GaussianTable.h"
#include "SampleTemplateGenerator.h"
#include "SensorModels/PerceptValidityChecker.h"
#include "SensorModels/SensorModel.h"
#include "SensorModels/FieldModel.h"
#include <vector>

#include "../../../../../src/UChProcess/UChBlackBoard.h"


template <typename Sample, typename SampleContainer>
class PoseCalculator;
class SelfLocatorParameter;

/*
MODULE(SelfLocator)
  REQUIRES(FieldDimensions)
  REQUIRES(GameInfo)
  REQUIRES(FrameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(LinePercept)
  REQUIRES(GoalPercept)
  REQUIRES(BehaviorConfiguration)
  REQUIRES(OdometryData)
  REQUIRES(MotionInfo)
  REQUIRES(CameraMatrix)
  PROVIDES_WITH_MODIFY_AND_DRAW(PotentialRobotPose)
  REQUIRES(PotentialRobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotPose)
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPoseHypotheses)
END_MODULE
*/


/**
* @class SelfLocator
* A SelfLocator using a particle filter
*/
class SelfLocator //: public SelfLocatorBase
{
private:
  /**
  * A sample/particle (defined in Tools/SampleSet.h)
  */
  typedef SelfLocatorSample Sample;

  ENUM(PoseCalculatorType,
    POSE_CALCULATOR_2D_BINNING,
    POSE_CALCULATOR_PARTICLE_HISTORY,
    POSE_CALCULATOR_BEST_PARTICLE,
    POSE_CALCULATOR_OVERALL_AVERAGE,
    POSE_CALCULATOR_K_MEANS_CLUSTERING
  );
  PoseCalculatorType poseCalculatorType;

  SelfLocatorParameter* parameter; /**< All tweaking values. */
  FieldModel fieldModel; /**< The model of proximity to features on the field. */
  SampleSet<Sample>* samples; /**< Container for all samples. */
  float totalWeighting; /**< The current weighting sum of all samples. */
  float slowWeighting; /**< This value follows the average weighting slowly. */
  float fastWeighting; /**< This value follows the average weighting more quickly. */
  Pose2D lastOdometry; /**< The last odometry state for calculating the offset since the last call. */
  bool updatedBySensors; /**< Was there an actual update during the previous observation update? */
  PoseCalculator< Sample, SampleSet<Sample> >* poseCalculator; /**< External class for computing a pose from a set of samples */
  PerceptValidityChecker* perceptValidityChecker; /** A module for checking perceptions before their use by the self locator */
  unsigned lastPoseComputationTimeStamp; /**< Point of time of last pose computation*/
  RobotPose lastComputedPose;            /**< The pose computed at lastPoseComputationTimeStamp*/
  SampleTemplateGenerator sampleTemplateGenerator; /**< Submodule for generating new samples */
  std::vector<SensorModel*> sensorModels; /**< List of all sensor models applied to the sample set*/
  std::vector<float> sensorModelWeightings; /**< Weightings for sample set after execution of sensor model*/
  int gameInfoPenaltyLastFrame; /**< Was the robot penalised in the last frame? */
  int gameInfoGameStateLastFrame; /**< The game state in the last frame */
  std::vector<const LinePercept::Line*> lines; /**< Pointers to the lines of the line percept. */
  std::vector<SensorModel::Observation> observations, /**< The indices of the observations that might be selected for sensor update. */
      selectedObservations; /**< The indices of the observations that are selected for sensor update. */
  std::vector<int> selectedIndices; /**< The indices of the observations that are selected to be updated by a single sensor model. */

  /**
  * The method provides the robot pose.
  * @param robotPose The potential robot pose representation that is updated by this module.
  */
public:
  void update(PotentialRobotPose& robotPose);

  /**
  * The method provides the potential robot pose as robot pose.
  * @param robotPose The robot pose representation that is updated by this module.
  */
  void update(RobotPose& robotPose);

  /**
  * The method provides a set of different hypotheses about the robot pose.
  * @param robotPoseHypotheses The representation that is updated by this module.
  */
  void update(RobotPoseHypotheses& robotPoseHypotheses);

private:
  /**
  * The method prepares execution and initializes some values.
  * @param robotPose The robot pose representation that is updated by this module.
  */
  void preExecution(RobotPose& robotPose);

  /**
  * The method performs the motion update step of particle filter localization.
  * @param noise Add random noise to the particles?
  */
  void motionUpdate(bool noise);

  /**
  * Applies all sensor models to adjust the weightings of the samples
  * @return true, if any new weightings have been computed. false, otherwise.
  */
  bool applySensorModels();

  /**
  * The method performs the resampling step of particle filter localization.
  * It might add new samples generated from templates.
  */
  void resampling();

  /**
  * Adapts the weightings for resampling percentage
  */
  void adaptWeightings();

  /**
  * Computes a template pose.
  * @param sample The resulting template sample.
  */
  void generateTemplate(Sample& sample);

  /**
  * Draws the particles.
  */
  void drawSamples();

  /**
  * A method that contains all intialisations that require
  * representations from the blackboard.
  */
public:
  virtual void init();
private:
  /**
  * Initializes the sample set.
  * If the vectors are empty (i.e. no positions given) the samples will
  * represent a uniform probability distribution which covers the whole state space.
  * If one or more positions are given, the samples will be randomly (again uniform) assigned
  * to these positions. Each position is represented by a gaussian distribution. Within these
  * distributions, the samples are randomly placed.
  * @param poses A vector of preferred sample poses
  * @param standardDeviations The standard deviations of the distributions of the points
  */
  void initSamplesAtGivenPositions(const std::vector<Pose2D>& poses,
                                   const std::vector<Pose2D>& standardDeviations);

  void setNumberOfSamples(const int num);

  UChBlackBoard* bb;

public:
  /** Default constructor. */
  SelfLocator(UChBlackBoard* blackboard);

  /** Destructor */
  ~SelfLocator();
};
