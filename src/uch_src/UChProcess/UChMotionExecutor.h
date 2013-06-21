#ifndef UCHMOTIONEXECUTOR_H
#define UCHMOTIONEXECUTOR_H

//Modules
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Sensing/FallDownStateDetector.h"
#include "Modules/Sensing/GroundContactDetector.h"
#include "Modules/Sensing/InertiaSensorCalibrator.h"
#include "Modules/Sensing/InertiaSensorFilter.h"
#include "Modules/Sensing/InertiaSensorInspector.h"
#include "Modules/Sensing/JointFilter.h"
#include "Modules/Sensing/RobotModelProvider.h"
#include "Modules/Sensing/SensorFilter.h"
#include "Modules/Sensing/TorsoMatrixProvider.h"
#include "Modules/MotionControl/BLAME.h"
#include "Modules/MotionControl/MotionCombinator.h"
#include "Modules/MotionControl/HeadMotionEngine.h"
#include "Modules/MotionControl/MotionSelector.h"
#include "Modules/MotionControl/SpecialActions.h"
#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"
#include "Platform/SystemCall.h"

#include "process.h"

#include <boost/thread/thread_time.hpp>

class process;

class UChMotionExecutor
{
public:
    UChMotionExecutor(process* processPointer_, UChBlackBoard* blackBoard_);

    ~UChMotionExecutor();
    void init();
    void runModules();
    void setSensors(float* sensors);

    //void copyCognition2MotionBB(const bh_motion::Cognition2Motion& cognition2Motion);  //DLF
    void copyCognition2MotionBB(const bh_motion::Cognition2Motion::ConstPtr& cognition2Motion);  //DLF

    void copyMotion2CognitionBB(bh_motion::Motion2Cognition& motion2Cognition);   //DLF

    process* processPointer;
    UChBlackBoard* bb;

    boost::system_time initialTime;
    boost::system_time finalTime;



    //Modules
    NaoProvider naoProvider;
    JointFilter jointFilter;
    GroundContactDetector groundContactDetector;
    RobotModelProvider robotModelProvider;
    InertiaSensorInspector inertiaSensorInspector;
    InertiaSensorCalibrator inertiaSensorCalibrator;
    InertiaSensorFilter inertiaSensorFilter;
    SensorFilter sensorFilter;
    FallDownStateDetector fallDownStateDetector;
    TorsoMatrixProvider torsoMatrixProvider;
    MotionSelector motionSelector;
    HeadMotionEngine headMotionEngine;
    WalkingEngine walkingEngine;
    MotionCombinator motionCombinator;
    SpecialActions specialActions;
    BLAME blame;
};

#endif // UCHMOTIONEXECUTOR_H
