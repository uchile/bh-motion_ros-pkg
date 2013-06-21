#ifndef _UCHROBOTDATA_
#define _UCHROBOTDATA_

#pragma once

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

enum SensorIds
{
    // joint data
    headYawPositionSensor,
    headYawCurrentSensor,
    headYawTemperatureSensor,
    headPitchPositionSensor,
    headPitchCurrentSensor,
    headPitchTemperatureSensor,
    lShoulderPitchPositionSensor,
    lShoulderPitchCurrentSensor,
    lShoulderPitchTemperatureSensor,
    lShoulderRollPositionSensor,
    lShoulderRollCurrentSensor,
    lShoulderRollTemperatureSensor,
    lElbowYawPositionSensor,
    lElbowYawCurrentSensor,
    lElbowYawTemperatureSensor,
    lElbowRollPositionSensor,
    lElbowRollCurrentSensor,
    lElbowRollTemperatureSensor,
    rShoulderPitchPositionSensor,
    rShoulderPitchCurrentSensor,
    rShoulderPitchTemperatureSensor,
    rShoulderRollPositionSensor,
    rShoulderRollCurrentSensor,
    rShoulderRollTemperatureSensor,
    rElbowYawPositionSensor,
    rElbowYawCurrentSensor,
    rElbowYawTemperatureSensor,
    rElbowRollPositionSensor,
    rElbowRollCurrentSensor,
    rElbowRollTemperatureSensor,
    lHipYawPitchPositionSensor,
    lHipYawPitchCurrentSensor,
    lHipYawPitchTemperatureSensor,
    lHipRollPositionSensor,
    lHipRollCurrentSensor,
    lHipRollTemperatureSensor,
    lHipPitchPositionSensor,
    lHipPitchCurrentSensor,
    lHipPitchTemperatureSensor,
    lKneePitchPositionSensor,
    lKneePitchCurrentSensor,
    lKneePitchTemperatureSensor,
    lAnklePitchPositionSensor,
    lAnklePitchCurrentSensor,
    lAnklePitchTemperatureSensor,
    lAnkleRollPositionSensor,
    lAnkleRollCurrentSensor,
    lAnkleRollTemperatureSensor,
    rHipRollPositionSensor,
    rHipRollCurrentSensor,
    rHipRollTemperatureSensor,
    rHipPitchPositionSensor,
    rHipPitchCurrentSensor,
    rHipPitchTemperatureSensor,
    rKneePitchPositionSensor,
    rKneePitchCurrentSensor,
    rKneePitchTemperatureSensor,
    rAnklePitchPositionSensor,
    rAnklePitchCurrentSensor,
    rAnklePitchTemperatureSensor,
    rAnkleRollPositionSensor,
    rAnkleRollCurrentSensor,
    rAnkleRollTemperatureSensor,

    // sensor data
    gyroXSensor,
    gyroYSensor,
    gyroRefSensor,
    accXSensor,
    accYSensor,
    accZSensor,
    batteryChargeSensor,
    lFSRFrontLeftSensor,
    lFSRFrontRightSensor,
    lFSRRearLeftSensor,
    lFSRRearRightSensor,
    rFSRFrontLeftSensor,
    rFSRFrontRightSensor,
    rFSRRearLeftSensor,
    rFSRRearRightSensor,
    lUsSensor,
    rUsSensor,
    angleXSensor,
    angleYSensor,

    // key states
    rBumperRightSensor,
    rBumperLeftSensor,
    lBumperRightSensor,
    lBumperLeftSensor,
    chestButtonSensor,

    // board info
    chestBoardAckSensor,
    chestBoardNackSensor,
    chestBoardErrorSensor,
    batteryAckSensor,
    batteryNackSensor,
    batteryErrorSensor,
    uSBoardAckSensor,
    uSBoardNackSensor,
    uSBoardErrorSensor,
    inertialSensorAckSensor,
    inertialSensorNackSensor,
    inertialSensorErrorSensor,
    headBoardAckSensor,
    headBoardNackSensor,
    headBoardErrorSensor,
    earLedsAckSensor,
    earLedsNackSensor,
    earLedsErrorSensor,
    faceBoardAckSensor,
    faceBoardNackSensor,
    faceBoardErrorSensor,
    leftShoulderBoardAckSensor,
    leftShoulderBoardNackSensor,
    leftShoulderBoardErrorSensor,
    leftArmBoardAckSensor,
    leftArmBoardNackSensor,
    leftArmBoardErrorSensor,
    RightShoulderBoardAckSensor,
    rightShoulderBoardNackSensor,
    rightShoulderBoardErrorSensor,
    rightArmBoardAckSensor,
    rightArmBoardNackSensor,
    rightArmBoardErrorSensor,
    leftHipBoardAckSensor,
    leftHipBoardNackSensor,
    leftHipBoardErrorSensor,
    leftThighBoardAckSensor,
    leftThighBoardNackSensor,
    leftThighBoardErrorSensor,
    leftShinBoardAckSensor,
    leftShinBoardNackSensor,
    leftShinBoardErrorSensor,
    leftFootBoardAckSensor,
    leftFootBoardNackSensor,
    leftFootBoardErrorSensor,
    rightHipBoardAckSensor,
    rightHipBoardNackSensor,
    rightHipBoardErrorSensor,
    rightThighBoardAckSensor,
    rightThighBoardNackSensor,
    rightThighBoardErrorSensor,
    rightShinBoardAckSensor,
    rightShinBoardNackSensor,
    rightShinBoardErrorSensor,
    rightFootBoardAckSensor,
    rightFootBoardNackSensor,
    rightFootBoardErrorSensor,

    numOfSensorIds,
};

enum ActuatorIds
{
    // joint request
    headYawPositionActuator,        //0
    headPitchPositionActuator,      //1
    lShoulderPitchPositionActuator, //
    lShoulderRollPositionActuator,  //
    lElbowYawPositionActuator,
    lElbowRollPositionActuator,
    rShoulderPitchPositionActuator,
    rShoulderRollPositionActuator,
    rElbowYawPositionActuator,
    rElbowRollPositionActuator,
    lHipYawPitchPositionActuator,
    lHipRollPositionActuator,
    lHipPitchPositionActuator,
    lKneePitchPositionActuator,
    lAnklePitchPositionActuator,
    lAnkleRollPositionActuator,
    rHipRollPositionActuator,
    rHipPitchPositionActuator,
    rKneePitchPositionActuator,
    rAnklePitchPositionActuator,
    rAnkleRollPositionActuator,
    NumOfPositionActuatorIds,      //21

    headYawHardnessActuator = NumOfPositionActuatorIds, //21
    headPitchHardnessActuator,
    lShoulderPitchHardnessActuator,
    lShoulderRollHardnessActuator,
    lElbowYawHardnessActuator,
    lElbowRollHardnessActuator,
    rShoulderPitchHardnessActuator,
    rShoulderRollHardnessActuator,
    rElbowYawHardnessActuator,
    rElbowRollHardnessActuator,
    lHipYawPitchHardnessActuator,
    lHipRollHardnessActuator,
    lHipPitchHardnessActuator,
    lKneePitchHardnessActuator,
    lAnklePitchHardnessActuator,
    lAnkleRollHardnessActuator,
    rHipRollHardnessActuator,
    rHipPitchHardnessActuator,
    rKneePitchHardnessActuator,
    rAnklePitchHardnessActuator,
    rAnkleRollHardnessActuator,    //41
    NumOfHardnessActuatorIds = rAnkleRollHardnessActuator + 1 - headYawHardnessActuator,

    // led request
    faceLedRedLeft0DegActuator = rAnkleRollHardnessActuator + 1,
    faceLedRedLeft45DegActuator,
    faceLedRedLeft90DegActuator,
    faceLedRedLeft135DegActuator,
    faceLedRedLeft180DegActuator,
    faceLedRedLeft225DegActuator,
    faceLedRedLeft270DegActuator,
    faceLedRedLeft315DegActuator,
    faceLedGreenLeft0DegActuator,
    faceLedGreenLeft45DegActuator,
    faceLedGreenLeft90DegActuator,
    faceLedGreenLeft135DegActuator,
    faceLedGreenLeft180DegActuator,
    faceLedGreenLeft225DegActuator,
    faceLedGreenLeft270DegActuator,
    faceLedGreenLeft315DegActuator,
    faceLedBlueLeft0DegActuator,
    faceLedBlueLeft45DegActuator,
    faceLedBlueLeft90DegActuator,
    faceLedBlueLeft135DegActuator,
    faceLedBlueLeft180DegActuator,
    faceLedBlueLeft225DegActuator,
    faceLedBlueLeft270DegActuator,
    faceLedBlueLeft315DegActuator,
    faceLedRedRight0DegActuator,
    faceLedRedRight45DegActuator,
    faceLedRedRight90DegActuator,
    faceLedRedRight135DegActuator,
    faceLedRedRight180DegActuator,
    faceLedRedRight225DegActuator,
    faceLedRedRight270DegActuator,
    faceLedRedRight315DegActuator,
    faceLedGreenRight0DegActuator,
    faceLedGreenRight45DegActuator,
    faceLedGreenRight90DegActuator,
    faceLedGreenRight135DegActuator,
    faceLedGreenRight180DegActuator,
    faceLedGreenRight225DegActuator,
    faceLedGreenRight270DegActuator,
    faceLedGreenRight315DegActuator,
    faceLedBlueRight0DegActuator,
    faceLedBlueRight45DegActuator,
    faceLedBlueRight90DegActuator,
    faceLedBlueRight135DegActuator,
    faceLedBlueRight180DegActuator,
    faceLedBlueRight225DegActuator,
    faceLedBlueRight270DegActuator,
    faceLedBlueRight315DegActuator,
    earsLedLeft36DegActuator,
    earsLedLeft72DegActuator,
    earsLedLeft108DegActuator,
    earsLedLeft144DegActuator,
    earsLedLeft180DegActuator,
    earsLedLeft216DegActuator,
    earsLedLeft252DegActuator,
    earsLedLeft288DegActuator,
    earsLedLeft324DegActuator,
    earsLedLeft0DegActuator,
    earsLedRight0DegActuator,
    earsLedRight36DegActuator,
    earsLedRight72DegActuator,
    earsLedRight108DegActuator,
    earsLedRight144DegActuator,
    earsLedRight180DegActuator,
    earsLedRight216DegActuator,
    earsLedRight252DegActuator,
    earsLedRight288DegActuator,
    earsLedRight324DegActuator,
    chestBoardLedRedActuator,
    chestBoardLedGreenActuator,
    chestBoardLedBlueActuator,
    lFootLedRedActuator,
    lFootLedGreenActuator,
    lFootLedBlueActuator,
    rFootLedRedActuator,
    rFootLedGreenActuator,
    rFootLedBlueActuator,
    NumOfLedActuatorIds = rFootLedBlueActuator + 1 - faceLedRedLeft0DegActuator,

    NumOfNormalActuatorIds = rFootLedBlueActuator + 1,

    usActuator = NumOfNormalActuatorIds,

    numOfActuatorIds,
};

struct UChRobotData
{
    UChRobotData()
    {
        //mutex_buffer_sensors0.unlock();
        mutex_sensors.unlock();
        mutex_actuators.unlock();

    for(unsigned int i = 0; i<numOfSensorIds; i++)
    {
        sensors[0][i] = 0.0f;
        sensors[1][i] = 0.0f;
    }

    for(unsigned int i = 0; i<numOfActuatorIds; i++)
    {
        actuators[0][i] = 0.0f;
        actuators[1][i] = 0.0f;
    }

    }

    //Índices
    volatile int readingSensors;
    volatile int newestSensors;
    volatile int readingActuators;
    volatile int newestActuators;

    //Buffer de tamaño 3.
    float sensors[3][numOfSensorIds];
    float actuators[3][numOfActuatorIds];

    //
    boost::interprocess::interprocess_condition cond_sensors;

    //Mutex anónimos para acceder a los buffers. Sacarlos de aquí y usar mutex identificados.
    boost::interprocess::interprocess_mutex mutex_sensors;
    boost::interprocess::interprocess_mutex mutex_actuators;
    //boost::interprocess::interprocess_mutex mutex_buffer_sensors1;
    //boost::interprocess::interprocess_mutex mutex_buffer_actuators0;
    //boost::interprocess::interprocess_mutex mutex_buffer_actuators1;
};

#ifdef __cplusplus
}
#endif

#endif
