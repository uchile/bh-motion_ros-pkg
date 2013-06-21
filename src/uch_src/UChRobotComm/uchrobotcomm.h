#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/thread.hpp>

#include "UChRobotData.h"

#ifdef __cplusplus
extern "C"
{
#endif

namespace AL
{
  class ALBroker;
}

class uchrobotcomm : public AL::ALModule
{	
public:
    uchrobotcomm(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);

    virtual ~uchrobotcomm();

    /**
    * echo: simply returns the paramter given as first arguemtn
    * The AL::ALValue is an union type which can be a std::string, an int, a float,
    * a vector of floats, and so on.
    */

    AL::ALValue echo(const AL::ALValue& foo);

    //void startThread();
    //void run();

    void CreateMemoryMapped(); //Creación de la memoria compartida.
    void CreateAlias();        //Creación de los aliasREQUEST en el DCM.
    void CreateRequest();      //Creación de los request (propiamente tal) para escribir instrucciones en los actuadores por medio de los alias.
    void SetSensorPointers();  //Punteros a los valores de los sensores.
    void ConnectToDCMloop();   //Suscripción a pre y post process.

    void onPostProcess();
    void onPreProcess();

    void sitDown(float lastRatio, float* actuators);
    void standUp(float lastRatio, float* destActuators, float* actuators);

    void* addr;
    UChRobotData* data; //Puntero al bloque de memoria compartida con xxxx proceso

    float* sensorPtrs[numOfSensorIds]; //Punteros a los valores de los sensores almacenados en memoryProxy

    AL::ALValue motorPositionRequest;
    AL::ALValue motorStiffnessRequest;
    AL::ALValue usRequest;
    AL::ALValue ledRequest;

    AL::DCMProxy*      dcmProxy;
    AL::ALMemoryProxy* memoryProxy;

    int dcmTime;          // Tiempo del dcm, actualizado en cada llamada a onPostProcess
    float frameRatio;     //
    bool shuttingDown;    // Is Nao pow off?
    bool pause_robotComm;

};

extern "C" int _createModule(boost::shared_ptr<AL::ALBroker> pBroker) //Función que llama NaoQi, para crear y partir el módulo.
{
  AL::ALModule::createModule<uchrobotcomm>(pBroker, "uchrobotcomm");
  return 0;
}

extern "C" int _closeModule() // NaoQi doesn't invoke this function
                              // thats why we register the "BHuman" module for using the constructor and destructor
{
  return 0;
}

#ifdef __cplusplus
}
#endif
