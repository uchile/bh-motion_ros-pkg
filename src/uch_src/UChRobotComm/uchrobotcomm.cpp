#include "uchrobotcomm.h"

//#include <boost/interprocess/sync/named_mutex.hpp>

//#include <alvalue/alvalue.h>
//#include <alcommon/alproxy.h>
//#include <alcommon/albroker.h>

#include <alcore/altypes.h>
#include <alcore/alerror.h>
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>

//#include <iostream>

#include <boost/bind.hpp>

#include <iostream>
#include <time.h>

#define ALLOWED_FRAMEDROPS 3
int frameDrops = ALLOWED_FRAMEDROPS + 1;

static const char* sensorNames[] =
{
  "Device/SubDeviceList/HeadYaw/Position/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Position/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value",

  "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value",
  "Device/SubDeviceList/Battery/Charge/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/US/Left/Sensor/Value",
  "Device/SubDeviceList/US/Right/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/ChestBoard/Button/Sensor/Value",
  "Device/DeviceList/ChestBoard/Ack",
  "Device/DeviceList/ChestBoard/Nack",
  "Device/DeviceList/ChestBoard/Error",
  "Device/DeviceList/Battery/Ack",
  "Device/DeviceList/Battery/Nack",
  "Device/DeviceList/Battery/Error",
  "Device/DeviceList/USBoard/Ack",
  "Device/DeviceList/USBoard/Nack",
  "Device/DeviceList/USBoard/Error",
  "Device/DeviceList/InertialSensor/Ack",
  "Device/DeviceList/InertialSensor/Nack",
  "Device/DeviceList/InertialSensor/Error",
  "Device/DeviceList/HeadBoard/Ack",
  "Device/DeviceList/HeadBoard/Nack",
  "Device/DeviceList/HeadBoard/Error",
  "Device/DeviceList/EarLeds/Ack",
  "Device/DeviceList/EarLeds/Nack",
  "Device/DeviceList/EarLeds/Error",
  "Device/DeviceList/FaceBoard/Ack",
  "Device/DeviceList/FaceBoard/Nack",
  "Device/DeviceList/FaceBoard/Error",
  "Device/DeviceList/LeftShoulderBoard/Ack",
  "Device/DeviceList/LeftShoulderBoard/Nack",
  "Device/DeviceList/LeftShoulderBoard/Error",
  "Device/DeviceList/LeftArmBoard/Ack",
  "Device/DeviceList/LeftArmBoard/Nack",
  "Device/DeviceList/LeftArmBoard/Error",
  "Device/DeviceList/RightShoulderBoard/Ack",
  "Device/DeviceList/RightShoulderBoard/Nack",
  "Device/DeviceList/RightShoulderBoard/Error",
  "Device/DeviceList/RightArmBoard/Ack",
  "Device/DeviceList/RightArmBoard/Nack",
  "Device/DeviceList/RightArmBoard/Error",
  "Device/DeviceList/LeftHipBoard/Ack",
  "Device/DeviceList/LeftHipBoard/Nack",
  "Device/DeviceList/LeftHipBoard/Error",
  "Device/DeviceList/LeftThighBoard/Ack",
  "Device/DeviceList/LeftThighBoard/Nack",
  "Device/DeviceList/LeftThighBoard/Error",
  "Device/DeviceList/LeftShinBoard/Ack",
  "Device/DeviceList/LeftShinBoard/Nack",
  "Device/DeviceList/LeftShinBoard/Error",
  "Device/DeviceList/LeftFootBoard/Ack",
  "Device/DeviceList/LeftFootBoard/Nack",
  "Device/DeviceList/LeftFootBoard/Error",
  "Device/DeviceList/RightHipBoard/Ack",
  "Device/DeviceList/RightHipBoard/Nack",
  "Device/DeviceList/RightHipBoard/Error",
  "Device/DeviceList/RightThighBoard/Ack",
  "Device/DeviceList/RightThighBoard/Nack",
  "Device/DeviceList/RightThighBoard/Error",
  "Device/DeviceList/RightShinBoard/Ack",
  "Device/DeviceList/RightShinBoard/Nack",
  "Device/DeviceList/RightShinBoard/Error",
  "Device/DeviceList/RightFootBoard/Ack",
  "Device/DeviceList/RightFootBoard/Nack",
  "Device/DeviceList/RightFootBoard/Error",
};

static const char* actuatorNames[] =
{
  "HeadYaw/Position/Actuator/Value",
  "HeadPitch/Position/Actuator/Value",
  "LShoulderPitch/Position/Actuator/Value",
  "LShoulderRoll/Position/Actuator/Value",
  "LElbowYaw/Position/Actuator/Value",
  "LElbowRoll/Position/Actuator/Value",
  "RShoulderPitch/Position/Actuator/Value",
  "RShoulderRoll/Position/Actuator/Value",
  "RElbowYaw/Position/Actuator/Value",
  "RElbowRoll/Position/Actuator/Value",
  "LHipYawPitch/Position/Actuator/Value",
  "LHipRoll/Position/Actuator/Value",
  "LHipPitch/Position/Actuator/Value",
  "LKneePitch/Position/Actuator/Value",
  "LAnklePitch/Position/Actuator/Value",
  "LAnkleRoll/Position/Actuator/Value",
  "RHipRoll/Position/Actuator/Value",
  "RHipPitch/Position/Actuator/Value",
  "RKneePitch/Position/Actuator/Value",
  "RAnklePitch/Position/Actuator/Value",
  "RAnkleRoll/Position/Actuator/Value",

  "HeadYaw/Hardness/Actuator/Value",
  "HeadPitch/Hardness/Actuator/Value",
  "LShoulderPitch/Hardness/Actuator/Value",
  "LShoulderRoll/Hardness/Actuator/Value",
  "LElbowYaw/Hardness/Actuator/Value",
  "LElbowRoll/Hardness/Actuator/Value",
  "RShoulderPitch/Hardness/Actuator/Value",
  "RShoulderRoll/Hardness/Actuator/Value",
  "RElbowYaw/Hardness/Actuator/Value",
  "RElbowRoll/Hardness/Actuator/Value",
  "LHipYawPitch/Hardness/Actuator/Value",
  "LHipRoll/Hardness/Actuator/Value",
  "LHipPitch/Hardness/Actuator/Value",
  "LKneePitch/Hardness/Actuator/Value",
  "LAnklePitch/Hardness/Actuator/Value",
  "LAnkleRoll/Hardness/Actuator/Value",
  "RHipRoll/Hardness/Actuator/Value",
  "RHipPitch/Hardness/Actuator/Value",
  "RKneePitch/Hardness/Actuator/Value",
  "RAnklePitch/Hardness/Actuator/Value",
  "RAnkleRoll/Hardness/Actuator/Value",

  "Face/Led/Red/Left/0Deg/Actuator/Value",
  "Face/Led/Red/Left/45Deg/Actuator/Value",
  "Face/Led/Red/Left/90Deg/Actuator/Value",
  "Face/Led/Red/Left/135Deg/Actuator/Value",
  "Face/Led/Red/Left/180Deg/Actuator/Value",
  "Face/Led/Red/Left/225Deg/Actuator/Value",
  "Face/Led/Red/Left/270Deg/Actuator/Value",
  "Face/Led/Red/Left/315Deg/Actuator/Value",
  "Face/Led/Green/Left/0Deg/Actuator/Value",
  "Face/Led/Green/Left/45Deg/Actuator/Value",
  "Face/Led/Green/Left/90Deg/Actuator/Value",
  "Face/Led/Green/Left/135Deg/Actuator/Value",
  "Face/Led/Green/Left/180Deg/Actuator/Value",
  "Face/Led/Green/Left/225Deg/Actuator/Value",
  "Face/Led/Green/Left/270Deg/Actuator/Value",
  "Face/Led/Green/Left/315Deg/Actuator/Value",
  "Face/Led/Blue/Left/0Deg/Actuator/Value",
  "Face/Led/Blue/Left/45Deg/Actuator/Value",
  "Face/Led/Blue/Left/90Deg/Actuator/Value",
  "Face/Led/Blue/Left/135Deg/Actuator/Value",
  "Face/Led/Blue/Left/180Deg/Actuator/Value",
  "Face/Led/Blue/Left/225Deg/Actuator/Value",
  "Face/Led/Blue/Left/270Deg/Actuator/Value",
  "Face/Led/Blue/Left/315Deg/Actuator/Value",
  "Face/Led/Red/Right/0Deg/Actuator/Value",
  "Face/Led/Red/Right/45Deg/Actuator/Value",
  "Face/Led/Red/Right/90Deg/Actuator/Value",
  "Face/Led/Red/Right/135Deg/Actuator/Value",
  "Face/Led/Red/Right/180Deg/Actuator/Value",
  "Face/Led/Red/Right/225Deg/Actuator/Value",
  "Face/Led/Red/Right/270Deg/Actuator/Value",
  "Face/Led/Red/Right/315Deg/Actuator/Value",
  "Face/Led/Green/Right/0Deg/Actuator/Value",
  "Face/Led/Green/Right/45Deg/Actuator/Value",
  "Face/Led/Green/Right/90Deg/Actuator/Value",
  "Face/Led/Green/Right/135Deg/Actuator/Value",
  "Face/Led/Green/Right/180Deg/Actuator/Value",
  "Face/Led/Green/Right/225Deg/Actuator/Value",
  "Face/Led/Green/Right/270Deg/Actuator/Value",
  "Face/Led/Green/Right/315Deg/Actuator/Value",
  "Face/Led/Blue/Right/0Deg/Actuator/Value",
  "Face/Led/Blue/Right/45Deg/Actuator/Value",
  "Face/Led/Blue/Right/90Deg/Actuator/Value",
  "Face/Led/Blue/Right/135Deg/Actuator/Value",
  "Face/Led/Blue/Right/180Deg/Actuator/Value",
  "Face/Led/Blue/Right/225Deg/Actuator/Value",
  "Face/Led/Blue/Right/270Deg/Actuator/Value",
  "Face/Led/Blue/Right/315Deg/Actuator/Value",
  "Ears/Led/Left/36Deg/Actuator/Value",
  "Ears/Led/Left/72Deg/Actuator/Value",
  "Ears/Led/Left/108Deg/Actuator/Value",
  "Ears/Led/Left/144Deg/Actuator/Value",
  "Ears/Led/Left/180Deg/Actuator/Value",
  "Ears/Led/Left/216Deg/Actuator/Value",
  "Ears/Led/Left/252Deg/Actuator/Value",
  "Ears/Led/Left/288Deg/Actuator/Value",
  "Ears/Led/Left/324Deg/Actuator/Value",
  "Ears/Led/Left/0Deg/Actuator/Value",
  "Ears/Led/Right/0Deg/Actuator/Value",
  "Ears/Led/Right/36Deg/Actuator/Value",
  "Ears/Led/Right/72Deg/Actuator/Value",
  "Ears/Led/Right/108Deg/Actuator/Value",
  "Ears/Led/Right/144Deg/Actuator/Value",
  "Ears/Led/Right/180Deg/Actuator/Value",
  "Ears/Led/Right/216Deg/Actuator/Value",
  "Ears/Led/Right/252Deg/Actuator/Value",
  "Ears/Led/Right/288Deg/Actuator/Value",
  "Ears/Led/Right/324Deg/Actuator/Value",
  "ChestBoard/Led/Red/Actuator/Value",
  "ChestBoard/Led/Green/Actuator/Value",
  "ChestBoard/Led/Blue/Actuator/Value",
  "LFoot/Led/Red/Actuator/Value",
  "LFoot/Led/Green/Actuator/Value",
  "LFoot/Led/Blue/Actuator/Value",
  "RFoot/Led/Red/Actuator/Value",
  "RFoot/Led/Green/Actuator/Value",
  "RFoot/Led/Blue/Actuator/Value",

  "US/Actuator/Value"
};

#define pi_2 1.5707963267948966f

static const float sitDownAngles[21] =
{
  -0.0135177,
  -(0.043983),
  0, //-( 0.780565-lbh_pi_2),
  -1.42764 + pi_2,
  0.126611,
  -0.25758,
  0, //-( 0.780528-lbh_pi_2),
  -(-1.43436 + pi_2),
  -(0.135423),
  -(-0.250192),
  -0.00440383,
  -(0.00831337),
  -0.88, //-1.00943,
  2.17113,
  -1.20265,
  -(0.00354111),
  0.00174092,
  -0.88, //-1.01021,
  2.17649,
  -1.20632,
  -0.00542875,
};

float zeroFloat = 0.f;

using namespace boost::interprocess;

uchrobotcomm::uchrobotcomm(
        boost::shared_ptr<AL::ALBroker> broker,
        const std::string& name): AL::ALModule(broker, name)
{
  setModuleDescription("This is an autogenerated module, this descriptio needs to be updated.");

  functionName("echo", getName(), "A simple echo function");
  addParam("foo", "Any parameter");
  setReturn("return", "Returns the given parameter");
  BIND_METHOD(uchrobotcomm::echo);

  dcmTime = 0;           // Tiempo del dcm, actualizado en cada llamada a onPostProcess
  frameRatio = 1.f;      //
  shuttingDown = false;  // Is Nao power off?
  pause_robotComm = false;

  data  = new UChRobotData;

  //DCMProxy
  try
  {
      dcmProxy = new AL::DCMProxy(broker);
  }
  catch (AL::ALError& e)
  {
      throw ALERROR(getName(), "uchrobotcomm", "Impossible to create DCM Proxy : " + e.toString());
  }

  //MemoryProxy
  try
  {
      memoryProxy = new AL::ALMemoryProxy(broker);
  }
  catch (AL::ALError& e)
  {
      throw ALERROR(getName(), "uchrobotcomm", "Impossible to create Memory Proxy : " + e.toString());
  }

  CreateMemoryMapped(); //Creación de la memoria compartida.
  CreateAlias();        //Creación de los aliasREQUEST en el DCM.
  CreateRequest();      //Creación de los request (propiamente tal) para escribir instrucciones en los actuadores por medio de los alias.
  SetSensorPointers();  //Punteros a los valores de los sensores.
  ConnectToDCMloop();   //Suscripción a pre y post process.

}

uchrobotcomm::~uchrobotcomm()
{
    //shared_memory_object::remove(shm);
    delete dcmProxy;
    delete memoryProxy;
}

inline unsigned int getSystemTime()
{
    static unsigned int base = 0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    unsigned int time = (unsigned int)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000l);
    if(!base)
        base = time - 10000; // avoid time == 0, because it is often used as a marker
    return time - base;
}

void uchrobotcomm::CreateMemoryMapped()
{
    //Open or create the shared memory object
    shared_memory_object shm(open_or_create, "UChRobotData", read_write);

    //Set size
    shm.truncate(sizeof(UChRobotData));

    //Map the whole shared memory in this process
    static mapped_region region(shm, read_write);

    //Get the address of the mapped region
    addr = region.get_address();

    //Construct the shared structure in memory
    data = new (addr)UChRobotData;
    //data = (UChRobotData*)addr;

    data->mutex_sensors.unlock();
    data->mutex_actuators.unlock();

    std::cout << "CreateMemoryMapped" << std::endl;
}

void uchrobotcomm::CreateAlias()
{
    //Creación del "positionRequest"
    AL::ALValue alias;
    AL::ALValue result;
    alias.arraySetSize(2); //[0] nombre, [1][i] nombre de cada joint

    alias[0] = std::string("positionRequest");
    alias[1].arraySetSize(NumOfPositionActuatorIds);
    for(unsigned int i = 0; i < NumOfPositionActuatorIds; ++i)
    {
        alias[1][i] = std::string(actuatorNames[i]);
    }

    result = dcmProxy->createAlias(alias);

    //Creación del "stiffnessRequest"
    alias[0] = std::string("stiffnessRequest");
    alias[1].arraySetSize(NumOfHardnessActuatorIds);
    for(unsigned int i = 0; i < NumOfHardnessActuatorIds; ++i)
    {
        alias[1][i] = std::string(actuatorNames[headYawHardnessActuator + i]);
    }

    result = dcmProxy->createAlias(alias);

    //Creación del "usRequest"
    alias[0] = std::string("usRequest");
    alias[1].arraySetSize(1);
    alias[1][0] = std::string(actuatorNames[usActuator]);

    result = dcmProxy->createAlias(alias);
}

void uchrobotcomm::CreateRequest()
{
    //Configuración del "motorPositionRequest"
    motorPositionRequest.arraySetSize(6);
    motorPositionRequest[0] = std::string("positionRequest");
    motorPositionRequest[1] = std::string("ClearAll");          //"Merge", revisar otras opciones. "ClearAll" borra todo lo anterior.
    motorPositionRequest[2] = std::string("time-separate");
    motorPositionRequest[3] = 0;
    motorPositionRequest[4].arraySetSize(1);
    motorPositionRequest[5].arraySetSize(NumOfPositionActuatorIds);
    for(unsigned int i = 0; i < NumOfPositionActuatorIds; ++i)
    {
        (motorPositionRequest)[5][i].arraySetSize(1);
    }

    //Configuración del "motorStiffnessRequest"
    motorStiffnessRequest.arraySetSize(6);
    motorStiffnessRequest[0] = std::string("stiffnessRequest");
    motorStiffnessRequest[1] = std::string("ClearAll"); //"Merge";
    motorStiffnessRequest[2] = std::string("time-separate");
    motorStiffnessRequest[3] = 0;
    motorStiffnessRequest[4].arraySetSize(1);
    motorStiffnessRequest[5].arraySetSize(NumOfHardnessActuatorIds);
    for(unsigned int i = 0; i < NumOfHardnessActuatorIds; ++i)
    {
      (motorStiffnessRequest)[5][i].arraySetSize(1);
    }

    //Configuración del "usRequest"
    usRequest.arraySetSize(6);
    usRequest[0] = std::string("usRequest");
    usRequest[1] = std::string("Merge"); // doesn't work with "ClearAll"
    usRequest[2] = std::string("time-separate");
    usRequest[3] = 0;
    usRequest[4].arraySetSize(1);
    usRequest[5].arraySetSize(1);
    usRequest[5][0].arraySetSize(1);

    //REVISAR ESTE LED REQUEST CON LA DOCUMENTACIÓN!
    //Configuración del "ledRequest"
    ledRequest.arraySetSize(3);
    //(*ledRequest)[0] = std::string("Face/Led/Green/Right/180Deg/Actuator/Value");
    ledRequest[1] = std::string("ClearAll");
    ledRequest[2].arraySetSize(1);
    ledRequest[2][0].arraySetSize(2);
    //(*ledReqest)[2][0][0] = 1.0;
    ledRequest[2][0][1] = 0;
}

void uchrobotcomm::SetSensorPointers()
{
    // prepare sensor pointers
    for(unsigned int i = 0; i < numOfSensorIds; ++i)
    {
        if(!strcmp(sensorNames[i], ""))
        {
            sensorPtrs[i] = &zeroFloat;
        }
        else
        {
            sensorPtrs[i] = (float*)memoryProxy->getDataPtr(sensorNames[i]);
        }
    }
}

void uchrobotcomm::ConnectToDCMloop()
{
    dcmProxy->getGenericProxy()->getModule()->atPostProcess(boost::bind(&uchrobotcomm::onPostProcess, this));
    dcmProxy->getGenericProxy()->getModule()->atPreProcess(boost::bind(&uchrobotcomm::onPreProcess, this));
}

void uchrobotcomm::sitDown(float lastFrameRatio, float* actuators)
{
    static bool isSittingDown = false;
    if(frameRatio > lastFrameRatio && frameRatio < 1.f)
    {
      static float startAngles[headYawHardnessActuator];
      static float startHardness[headYawHardnessActuator];
      if(!isSittingDown)
      {
        for(int i = 0; i < headYawHardnessActuator; ++i)
        {
          startAngles[i] = motorPositionRequest[5][i][0];
          startHardness[i] = motorStiffnessRequest[5][i][0];
        }
        frameRatio = 0.f;
        isSittingDown = true;
      }
      for(int i = 0; i < headYawHardnessActuator; ++i)
      {
        actuators[i] = startAngles[i] * (1.f - frameRatio) + sitDownAngles[i] * frameRatio;
        actuators[i + headYawHardnessActuator] = startHardness[i];
        if(actuators[i + headYawHardnessActuator] > 0.3f)
          actuators[i + headYawHardnessActuator] = 0.3f; // reduced hardness
      }
    }
    else
      isSittingDown = false;
}

void uchrobotcomm::standUp(float lastFrameRatio, float* destActuators, float* actuators)
{
  static bool isStandingUp = false;
  if(frameRatio < lastFrameRatio && frameRatio > 0.f)
  {
    static float startAngles[headYawHardnessActuator];
    if(!isStandingUp)
    {
      if(lastFrameRatio == 1.f)
        for(int i = 0; i < headYawHardnessActuator; ++i)
          startAngles[i] = *(sensorPtrs[i * 3]);
      else
        for(int i = 0; i < headYawHardnessActuator; ++i)
          startAngles[i] = motorPositionRequest[5][i][0];
      frameRatio = 1.f;
      isStandingUp = true;
    }
    for(int i = 0; i < headYawHardnessActuator; ++i)
    {
      actuators[i] = destActuators[i] * (1.f - frameRatio) + startAngles[i] * frameRatio;
      float h = destActuators[i + headYawHardnessActuator];
      if(h > 0.5f)
        h = 0.5f;
      actuators[i + headYawHardnessActuator] = destActuators[i + headYawHardnessActuator] * (1.f - frameRatio) + h * frameRatio;
    }
  }
  else
    isStandingUp = false;

  //std::cout << "Parándose" << std::endl;

}

//Lectura de sensores
void uchrobotcomm::onPostProcess()
{
    //std::cout << "lectura" << std::endl;
    int writingSensors = 0;
    if(writingSensors == data->newestSensors)
        ++writingSensors;
    if(writingSensors == data->readingSensors)
        if(++writingSensors == data->newestSensors)
             ++writingSensors;
    for(unsigned int i = 0; i < numOfSensorIds; i++)
    {
        data->sensors[writingSensors][i] = *sensorPtrs[i];
    }

    data->newestSensors = writingSensors;
    data->cond_sensors.notify_one();

    // detect shutdown request via chest-button
    static int startButtonChestPressedTime = dcmTime;
    static int startButtonLeftBumperPressedTIme = dcmTime;

    if(!*(sensorPtrs[chestButtonSensor]))
      startButtonChestPressedTime = dcmTime;
    else if(startButtonChestPressedTime && !shuttingDown && dcmTime - startButtonChestPressedTime > 3000)
    {
      if(*(sensorPtrs[rBumperRightSensor]) || *(sensorPtrs[rBumperLeftSensor]) || *(sensorPtrs[lBumperRightSensor]) || *(sensorPtrs[lBumperLeftSensor]))
      {
        //system("shutdown -n -r now"); TODO! Se necesita ser root
      }
      else{
        //system("shutdown -h now");    TODO! Se necesita ser root
      }
      shuttingDown = true;
    }

    if(!*(sensorPtrs[lBumperRightSensor]) || !*(sensorPtrs[lBumperLeftSensor]))
    {
      startButtonLeftBumperPressedTIme = dcmTime;
    }
    else if(startButtonLeftBumperPressedTIme && !pause_robotComm && dcmTime - startButtonLeftBumperPressedTIme > 50 && frameRatio == 0.f)
    {
        pause_robotComm = true;
    }
    else if(startButtonLeftBumperPressedTIme &&  pause_robotComm && dcmTime - startButtonLeftBumperPressedTIme > 50 && frameRatio == 1.f)
    {
        pause_robotComm = false;
    }
}

//Escritura en los motores
void uchrobotcomm::onPreProcess()
{
    //std::cout << "frameRatio: "      << frameRatio << std::endl;

    float* actuators = 0;
    dcmTime = (int)getSystemTime();// + timeOffset; TODO! calcular offset, diferencia de tiempo entre NaoQi y robotComm

    data->readingActuators = data->newestActuators;
    static int lastReadingActuators = -1;
    static int actuatorsDrops = 0;

    if(data->readingActuators == lastReadingActuators)
    {
        if(actuatorsDrops == 0)
        {
            std::cout << "missed actuator request" << std::endl;
        }
        actuatorsDrops++;
    }
    else{
        actuatorsDrops = 0;
    }

    //if (data->mutex_actuators.try_lock())
    //{
        lastReadingActuators = data->readingActuators;

        actuators = data->actuators[data->readingActuators];

        //std::cout << "solicitud" << std::endl;
        //std::cout << actuators[lHipPitchHardnessActuator] << std::endl;

    //    data->mutex_actuators.unlock();
    //}
    //else{
        //actuators = sensorPtrs[0];
    //    std::cout << "missed actuator request blocking" << std::endl;
    //    actuatorsDrops++;
    //}


    //actualización del frameRatio
    float lastFrameRatio = frameRatio;
    if (/*frameDrops > ALLOWED_FRAMEDROPS || */shuttingDown || pause_robotComm || (frameRatio >= 1.f && actuators[lHipPitchHardnessActuator] == 0.f && actuators[rHipPitchHardnessActuator] == 0.f))
    {
        frameRatio += 0.005f;
    }
    else{
        frameRatio -= 0.005f;
    }
    if(frameRatio < 0.f)
    {
        frameRatio = 0.f;
    }
    else if(frameRatio > 1.f)
    {
        frameRatio = 1.f;
    }

    if(frameRatio > 0.f)
    {
        static float controlledActuators[numOfActuatorIds];

        for(unsigned int i = 0; i < numOfActuatorIds; i++)
        {
            controlledActuators[i] = 0.f;
        }

        static bool wasActive = false;

        if(frameRatio < 0.2f && actuators[lHipPitchHardnessActuator] > 0.11f)
        {
            wasActive  = true;
        }

        if(wasActive)
        {
            controlledActuators[lShoulderPitchPositionActuator] = sitDownAngles[lShoulderPitchPositionActuator - headYawPositionActuator];
            controlledActuators[rShoulderPitchPositionActuator] = sitDownAngles[rShoulderPitchPositionActuator - headYawPositionActuator];
            controlledActuators[lShoulderPitchHardnessActuator] = 0.11f;
            controlledActuators[rShoulderPitchHardnessActuator] = 0.11f;
            controlledActuators[lHipPitchPositionActuator] = sitDownAngles[lHipPitchPositionActuator - headYawPositionActuator];
            controlledActuators[rHipPitchPositionActuator] = sitDownAngles[rHipPitchPositionActuator - headYawPositionActuator];
            controlledActuators[lHipPitchHardnessActuator] = 0.1f;
            controlledActuators[rHipPitchHardnessActuator] = 0.1f;
        }

        sitDown(lastFrameRatio, controlledActuators);
        standUp(lastFrameRatio, actuators, controlledActuators);
        //setEyeLeds(controlledActuators);
        if(frameDrops == 0 && !shuttingDown)
        {
            //copyLeds(actuators, controlledActuators);
        }
        actuators = controlledActuators;
        //std::cout << actuators[lHipPitchHardnessActuator] << std::endl;
        //std::cout << controlledActuators[lHipPitchHardnessActuator] << std::endl;
    }

    // setBatteryLeds(actuators);

    // motorPositionRequest
    (motorPositionRequest)[4][0] = dcmProxy->getTime(0);  // 0 delay!
    for(unsigned int i = 0; i < NumOfPositionActuatorIds; i++)
    {
        (motorPositionRequest)[5][i][0] = actuators[i];
    }
    dcmProxy->setAlias(motorPositionRequest);

    // motorStiffnessRequest
    (motorStiffnessRequest)[4][0] = dcmProxy->getTime(0); // 0 delay!
    for(unsigned int i = 0; i < NumOfHardnessActuatorIds; i++)
    {
        //if (i == 0 || i == 1)
        //{
        //(motorStiffnessRequest)[5][i][0] = 0.f;
        //}else{
        (motorStiffnessRequest)[5][i][0] = actuators[i+headYawHardnessActuator];
        //}
    }
    dcmProxy->setAlias(motorStiffnessRequest);


    //    if(frameRatio > 0.f)
    //    {
    //        static float controlledActuators[numOfActuatorIds];
    //        sitDown(lastFrameRatio, controlledActuators);
    //        standUp(lastFrameRatio, actuators, controlledActuators);
    //        actuators = controlledActuators;
    //    }

    //    if (buttonState)
    //        frameRatio -= 0.005f;
    //    else
    //        frameRatio += 0.005f;
    //    if(frameRatio < 0.f)
    //        frameRatio = 0.f;
    //    else if(frameRatio > 1.f)
    //        frameRatio = 1.f;

    //    if (!(frameRatio == 1.f))
    //    {
    //        data->mutex_actuators.try_lock();
    //        data->readingActuators = data->newestActuators;
    //        int DCMtime;
    //        DCMtime = dcmProxy->getTime(0);
    //        (motorPositionRequest)[4][0] = DCMtime;  // 0 delay!
    //        for(unsigned int i = 0; i < NumOfPositionActuatorIds; i++)
    //        {
    //            (motorPositionRequest)[5][i][0] = actuators[i];
    //        }
    //        dcmProxy->setAlias(motorPositionRequest);

    //        (motorStiffnessRequest)[4][0] = DCMtime; // 0 delay!
    //        for(unsigned int i = 0; i < NumOfHardnessActuatorIds; i++)
    //        {
    //            (motorStiffnessRequest)[5][i][0] = 0.3;//actuators[i+headYawHardnessActuator];
    //        }
    //        dcmProxy->setAlias(motorStiffnessRequest);
    //        data->mutex_actuators.unlock();
    //    }

}

AL::ALValue uchrobotcomm::echo(const AL::ALValue& foo)
{
    return foo;
}
