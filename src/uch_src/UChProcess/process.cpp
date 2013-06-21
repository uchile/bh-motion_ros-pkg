#include "process.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <iostream>

using namespace boost::interprocess;

process::process()
{
    //Open or create the shared memory object
    shared_memory_object shm(open_only, "UChRobotData", read_write);
    //Map the whole shared memory in this process
    static mapped_region region(shm, read_write);
    //Get the address of the mapped region
    addr = region.get_address();
    //Construct the shared structure in memory
    data = static_cast<UChRobotData*>(addr);

    data->mutex_sensors.unlock();
    data->mutex_actuators.unlock();

    cog2motion_subs = n.subscribe("cog2motionTopic", 1, &process::cognition2MotionCallBack, this); //DLF
    motion2cog_pub = n.advertise<bh_motion::Motion2Cognition>("motion2cogTopic", 1); //DLF
    ROS_INFO_STREAM("The motionNode is running" );   //DLF

    uChMotionExecutor = new UChMotionExecutor(this, &blackboardMotion);
    startThread();
}

process::~process()
{
    delete uChMotionExecutor;
    delete data;
}

void process::startThread()
{
    motionThread    = boost::thread(&process::runMotion, this);
}

void process::runMotion()
{
    boost::system_time tic; //DLF

    ROS_INFO_STREAM("Antes del while(true) de runmodels" );  //DLF

    long unsigned int cont=0;
    while(n.ok())  // while motionNode is running
    {

        tic = boost::get_system_time();  //DLF

        //ROS_INFO("Motion: Antes de spinOnce");
        //ros::spinOnce();  // DLF   It will call cognition2MotionCallBack waiting to be called at that point in time.
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.0));

        sensorsFromSharedMemory(); //escritura de sensores desde la memoria compartida
        uChMotionExecutor->runModules();
        actuatorsToSharedMemory(); //escritura de actuadores (salida de bhwalk) a la memoria compartida

        //tic = boost::get_system_time();  //DLF

        uChMotionExecutor->copyMotion2CognitionBB(motion2Cognition);  //DLF

        // ros' publication
        motion2Cognition.cont = cont;
        motion2cog_pub.publish(motion2Cognition);  //DLF
        //ROS_INFO("Motion: I've published msg # %ld ", motion2Cognition.cont);

        boost::posix_time::time_duration diff = boost::get_system_time() - tic;
        if (diff.total_milliseconds() > 14 )
           std::cout <<  "WARNING: Motion time (us) = " << diff.total_microseconds() << std::endl; //DLF

        //std::cout <<  "Motion: cpBBm2c time (us) = " << diff.total_microseconds() << std::endl; //DLF
        //std::cout <<  "Motion time (us) = " << diff.total_microseconds() << std::endl; //DLF

        cont++;
    }
}

// DLF <-----
void process::cognition2MotionCallBack(const bh_motion::Cognition2Motion::ConstPtr& cognition2Motion_)
{
    //boost::system_time tic = boost::get_system_time();  //DLF

    //ROS_INFO("Motion: I receive a callback from cognition # %ld\n", cognition2Motion_->cont);
    uChMotionExecutor -> copyCognition2MotionBB(cognition2Motion_);  //DLF

    //ROS_INFO("Motion: I receive walk target: x=%f, y=%f, theta=%f", cognition2Motion.motionRequest.walkRequest.target.translation.x, cognition2Motion.motionRequest.walkRequest.target.translation.y, cognition2Motion.motionRequest.walkRequest.target.rotation);

    //boost::posix_time::time_duration diff = boost::get_system_time() - tic;
    //std::cout <<  "Motion callback.c2m Time (us) = " << diff.total_microseconds() << std::endl; //DLF
}
// DLF ---->

void process::sensorsFromSharedMemory()
{
    scoped_lock<interprocess_mutex> lock(data->mutex_sensors);
    data->cond_sensors.wait(lock);
    data->readingSensors = data->newestSensors;
    uChMotionExecutor->setSensors(data->sensors[data->newestSensors]);
}

void process::actuatorsToSharedMemory()
{
    int writingActuators = 0;

    data->mutex_actuators.lock();
    if(writingActuators == data->newestActuators)
        ++writingActuators;
    if(writingActuators == data->readingActuators)
        if(++writingActuators == data->newestActuators)
            ++writingActuators;

    for(unsigned int i = 0; i < numOfActuatorIds; i++)
    {
        data->actuators[writingActuators][i] = uChMotionExecutor->naoProvider.actuators[i];
    }
    data->newestActuators = writingActuators;
    data->mutex_actuators.unlock();
}
