#ifndef _PROCESS_
#define _PROCESS_

#include "ros/ros.h"    //DLF
#include <ros/callback_queue.h> //DLF
#include "bh_motion/Cognition2Motion.h" //DLF
#include "bh_motion/Motion2Cognition.h" //DLF

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread_time.hpp>

#include <iostream>

#include "../UChRobotComm/UChRobotData.h"
#include "UChBlackBoard.h"
//#include "UChCognitionExecutor.h"
#include "UChMotionExecutor.h"

class UChMotionExecutor;
//class UChCognitionExecutor; //DLF

class process
{
public:
    process();
    ~process();

    void cognition2MotionCallBack(const bh_motion::Cognition2Motion::ConstPtr& cognition2Motion_);  //DLF
    bh_motion::Motion2Cognition motion2Cognition; //DLF

    void startThreads();
    void sensorsFromSharedMemory();
    void actuatorsToSharedMemory();
    void runMotion();
    void startThread();

    UChRobotData* data;
    void* addr;
    static const char* shm;

    UChBlackBoard blackboardMotion;

    boost::thread motionThread;
    //boost::thread cognitionThread;

    //boost::mutex commMutexMotion; //DLF, por ahora no lo estoy usando pero quizas posteriormente si.

    UChMotionExecutor* uChMotionExecutor;

protected:
    ros::NodeHandle n;  //DLF
    ros::Subscriber cog2motion_subs;  //DLF
    ros::Publisher motion2cog_pub; // DLF


};

#endif
