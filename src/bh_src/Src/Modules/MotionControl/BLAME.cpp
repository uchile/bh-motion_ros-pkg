/*
* @file BLAME.cpp
* This file implements a module that creates motions.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#include "BLAME.h"

//#include "Tools/Debugging/DebugDrawings.h"
//#include "Tools/Debugging/DebugDrawings3D.h"
#include "../../Representations/MotionControl/BikeRequest.h"
//#include "Tools/Debugging/Modify.h"
#include "BIKEParameters.h"
#include "../../Tools/InverseKinematic.h"
#include "../../Tools/Math/Pose3D.h"
#include <errno.h>
#include <iostream>

BLAME::BLAME():
  compensate(false),
  compensated(false)
{
  params.reserve(10);

  char dirname[260];

#ifdef WIN32
  sprintf(dirname, "%s/Config/Bike/*.bmc", File::getBHDir());
  WIN32_FIND_DATA findFileData;
  std::string fileName;
  HANDLE hFind = FindFirstFile(dirname, &findFileData);


  while(hFind != INVALID_HANDLE_VALUE)
  {
    char name[512];

    fileName = findFileData.cFileName;

    sprintf(name, "Bike/%s", fileName.c_str());


    InConfigMap stream(name);
    ASSERT(stream.exists());

    BIKEParameters parameters;
    stream >> parameters;

    char temp[260];

    sprintf(temp, "%s", fileName.c_str());


    for(int i = 0; i < 260; i++)
    {
      if(temp[i] == '.') temp[i] = 0;
    }

    strcpy(parameters.name, temp);
    if(BikeRequest::getBMotionFromName(parameters.name) < BikeRequest::numOfBMotionIDs)
    {
      params.push_back(parameters);
    }
    else
    {
      OUTPUT(idText, text, "Warning: BikeRequest is missing the id for " << parameters.name);
    }

    if(!FindNextFile(hFind, &findFileData))break;
  }

#else //LINUX
  sprintf(dirname, "%s/Config/Bike/", File::getBHDir());
  DIR* dir = opendir("/home/nao/.config/naoqi/Data/Config/Bike");
  ASSERT(dir);
  struct dirent* file = readdir(dir);

  while(file != NULL)
  {
    char name[260];
    sprintf(name, "Bike/%s", file->d_name);

    if(strstr(name, ".bmc"))
    {
      //TODO:Hacer funcionar el stream.

      const std::string name2 = "/home/nao/.config/naoqi/Data/Config/Bike/kickForward.bmc";
      InConfigMap stream(name2);
      ASSERT(stream.exists());

      BIKEParameters parameters;
      std::cout << "Leyendo kickForward.bcm" << std::endl;
      stream >> parameters;

      /*parameters.footOrigin.x=0;
        parameters.footOrigin.y=55;
        parameters.footOrigin.z=-230;
        parameters.footRotOrigin.x=0;
        parameters.footRotOrigin.y=0;
        parameters.footRotOrigin.z=0;
        parameters.armOrigin.x=0;
        parameters.armOrigin.y=120;
        parameters.armOrigin.z=80;
        parameters.handRotOrigin.x=0;
        parameters.handRotOrigin.y=1;
        parameters.handRotOrigin.z=0.5;
        parameters.comOrigin.x=10;
        parameters.comOrigin.y=5;
        parameters.kpx = 0.3 ;
        parameters.kix = 0.04 ;
        parameters.kdx = 0 ;
        parameters.kpy = 0.3 ;
        parameters.kiy = 0.04 ;
        parameters.kdy = 0 ;
        parameters.preview = 100 ;
        parameters.loop = false ;
        parameters.autoComTra = false ;
        parameters.numberOfPhases = 7 ;
        std::vector<Phase> phaseParameters (7);

        //Primer elemento de phasePameters
        Phase phase;
        phase.duration=2000;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=55;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=55;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=0;
        phase.controlPoints[Phase::rightFootTra][0].y=-55;
        phase.controlPoints[Phase::rightFootTra][0].z=-230;
        phase.controlPoints[Phase::rightFootTra][1].x=0;
        phase.controlPoints[Phase::rightFootTra][1].y=-55;
        phase.controlPoints[Phase::rightFootTra][1].z=-230;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=120;
        phase.controlPoints[Phase::leftArmTra][0].z=80;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=120;
        phase.controlPoints[Phase::leftArmTra][1].z=80;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-120;
        phase.controlPoints[Phase::rightArmTra][0].z=80;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-120;
        phase.controlPoints[Phase::rightArmTra][1].z=80;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=1;
        phase.controlPoints[Phase::rightHandRot][0].z=-0.5;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=1;
        phase.controlPoints[Phase::rightHandRot][1].z=-0.5;
        phase.comTra[0].x=10;
        phase.comTra[0].y=0;
        phase.comTra[1].x=10;
        phase.comTra[1].y=0;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[0]=phase;

        //Segundo elemento de phasePameters
        phase.duration=2000;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=-5;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=-5;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=0;
        phase.controlPoints[Phase::rightFootTra][0].y=-115;
        phase.controlPoints[Phase::rightFootTra][0].z=-225;
        phase.controlPoints[Phase::rightFootTra][1].x=0;
        phase.controlPoints[Phase::rightFootTra][1].y=-115;
        phase.controlPoints[Phase::rightFootTra][1].z=-225;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=120;
        phase.controlPoints[Phase::leftArmTra][0].z=80;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=120;
        phase.controlPoints[Phase::leftArmTra][1].z=80;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-120;
        phase.controlPoints[Phase::rightArmTra][0].z=80;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-120;
        phase.controlPoints[Phase::rightArmTra][1].z=80;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=1;
        phase.controlPoints[Phase::rightHandRot][0].z=-0.5;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=1;
        phase.controlPoints[Phase::rightHandRot][1].z=-0.5;
        phase.comTra[0].x=10;
        phase.comTra[0].y=-5;
        phase.comTra[1].x=10;
        phase.comTra[1].y=-5;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[1]=phase;

        //Tercer elemento de phasePameters
        phase.duration=1000;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=-5;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=-5;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=0;
        phase.controlPoints[Phase::rightFootTra][0].y=-100;
        phase.controlPoints[Phase::rightFootTra][0].z=-200;
        phase.controlPoints[Phase::rightFootTra][1].x=0;
        phase.controlPoints[Phase::rightFootTra][1].y=-100;
        phase.controlPoints[Phase::rightFootTra][1].z=-200;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=120;
        phase.controlPoints[Phase::leftArmTra][0].z=80;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=120;
        phase.controlPoints[Phase::leftArmTra][1].z=80;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-120;
        phase.controlPoints[Phase::rightArmTra][0].z=80;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-120;
        phase.controlPoints[Phase::rightArmTra][1].z=80;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=1;
        phase.controlPoints[Phase::rightHandRot][0].z=-0.5;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=1;
        phase.controlPoints[Phase::rightHandRot][1].z=-0.5;
        phase.comTra[0].x=10;
        phase.comTra[0].y=-5;
        phase.comTra[1].x=10;
        phase.comTra[1].y=-5;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[2]=phase;

        //Cuarto elemento de phasePameters
        phase.duration=1000;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=-5;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=-5;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=-90;
        phase.controlPoints[Phase::rightFootTra][0].y=-100;
        phase.controlPoints[Phase::rightFootTra][0].z=-160;
        phase.controlPoints[Phase::rightFootTra][1].x=-90;
        phase.controlPoints[Phase::rightFootTra][1].y=-100;
        phase.controlPoints[Phase::rightFootTra][1].z=-160;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=160;
        phase.controlPoints[Phase::leftArmTra][0].z=80;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=160;
        phase.controlPoints[Phase::leftArmTra][1].z=80;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-120;
        phase.controlPoints[Phase::rightArmTra][0].z=80;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-120;
        phase.controlPoints[Phase::rightArmTra][1].z=80;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=1;
        phase.controlPoints[Phase::rightHandRot][0].z=-0.5;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=1;
        phase.controlPoints[Phase::rightHandRot][1].z=-0.5;
        phase.comTra[0].x=10;
        phase.comTra[0].y=-5;
        phase.comTra[1].x=10;
        phase.comTra[1].y=-5;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[3]=phase;

        //Quinto elemento de phasePameters
        phase.duration=500;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=-5;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=-5;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=70.2095;
        phase.controlPoints[Phase::rightFootTra][0].y=-100;
        phase.controlPoints[Phase::rightFootTra][0].z=-160;
        phase.controlPoints[Phase::rightFootTra][1].x=75;
        phase.controlPoints[Phase::rightFootTra][1].y=-100;
        phase.controlPoints[Phase::rightFootTra][1].z=-160;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=120;
        phase.controlPoints[Phase::leftArmTra][0].z=80;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=120;
        phase.controlPoints[Phase::leftArmTra][1].z=80;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-160;
        phase.controlPoints[Phase::rightArmTra][0].z=80;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-160;
        phase.controlPoints[Phase::rightArmTra][1].z=80;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=1;
        phase.controlPoints[Phase::rightHandRot][0].z=-0.5;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=1;
        phase.controlPoints[Phase::rightHandRot][1].z=-0.5;
        phase.comTra[0].x=10;
        phase.comTra[0].y=-5;
        phase.comTra[1].x=10;
        phase.comTra[1].y=-5;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[4]=phase;

        //Sexto elemento de phasePameters
        phase.duration=2000;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=-5;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=-5;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=-17.6936;
        phase.controlPoints[Phase::rightFootTra][0].y=-100;
        phase.controlPoints[Phase::rightFootTra][0].z=-210;
        phase.controlPoints[Phase::rightFootTra][1].x=-3.9187;
        phase.controlPoints[Phase::rightFootTra][1].y=-100;
        phase.controlPoints[Phase::rightFootTra][1].z=-220;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=150;
        phase.controlPoints[Phase::leftArmTra][0].z=150;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=150;
        phase.controlPoints[Phase::leftArmTra][1].z=150;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-150;
        phase.controlPoints[Phase::rightArmTra][0].z=150;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-150;
        phase.controlPoints[Phase::rightArmTra][1].z=150;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=1;
        phase.controlPoints[Phase::rightHandRot][0].z=-0.5;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=1;
        phase.controlPoints[Phase::rightHandRot][1].z=-0.5;
        phase.comTra[0].x=10;
        phase.comTra[0].y=-5;
        phase.comTra[1].x=10;
        phase.comTra[1].y=-5;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[5]=phase;

        //Septimo elemento de phasePameters
        phase.duration=4000;
        phase.controlPoints[Phase::leftFootTra][0].x=0;
        phase.controlPoints[Phase::leftFootTra][0].y=25;
        phase.controlPoints[Phase::leftFootTra][0].z=-230;
        phase.controlPoints[Phase::leftFootTra][1].x=0;
        phase.controlPoints[Phase::leftFootTra][1].y=55;
        phase.controlPoints[Phase::leftFootTra][1].z=-230;
        phase.controlPoints[Phase::leftFootRot][0].x=0;
        phase.controlPoints[Phase::leftFootRot][0].y=0;
        phase.controlPoints[Phase::leftFootRot][0].z=0;
        phase.controlPoints[Phase::leftFootRot][1].x=0;
        phase.controlPoints[Phase::leftFootRot][1].y=0;
        phase.controlPoints[Phase::leftFootRot][1].z=0;
        phase.controlPoints[Phase::rightFootTra][0].x=0;
        phase.controlPoints[Phase::rightFootTra][0].y=-85;
        phase.controlPoints[Phase::rightFootTra][0].z=-230;
        phase.controlPoints[Phase::rightFootTra][1].x=0;
        phase.controlPoints[Phase::rightFootTra][1].y=-55;
        phase.controlPoints[Phase::rightFootTra][1].z=-230;
        phase.controlPoints[Phase::rightFootRot][0].x=0;
        phase.controlPoints[Phase::rightFootRot][0].y=0;
        phase.controlPoints[Phase::rightFootRot][0].z=0;
        phase.controlPoints[Phase::rightFootRot][1].x=0;
        phase.controlPoints[Phase::rightFootRot][1].y=0;
        phase.controlPoints[Phase::rightFootRot][1].z=0;
        phase.controlPoints[Phase::leftArmTra][0].x=0;
        phase.controlPoints[Phase::leftArmTra][0].y=150;
        phase.controlPoints[Phase::leftArmTra][0].z=80;
        phase.controlPoints[Phase::leftArmTra][1].x=0;
        phase.controlPoints[Phase::leftArmTra][1].y=150;
        phase.controlPoints[Phase::leftArmTra][1].z=80;
        phase.controlPoints[Phase::leftHandRot][0].x=0;
        phase.controlPoints[Phase::leftHandRot][0].y=1;
        phase.controlPoints[Phase::leftHandRot][0].z=0.5;
        phase.controlPoints[Phase::leftHandRot][1].x=0;
        phase.controlPoints[Phase::leftHandRot][1].y=1;
        phase.controlPoints[Phase::leftHandRot][1].z=0.5;
        phase.controlPoints[Phase::rightArmTra][0].x=0;
        phase.controlPoints[Phase::rightArmTra][0].y=-150;
        phase.controlPoints[Phase::rightArmTra][0].z=150;
        phase.controlPoints[Phase::rightArmTra][1].x=0;
        phase.controlPoints[Phase::rightArmTra][1].y=-150;
        phase.controlPoints[Phase::rightArmTra][1].z=150;
        phase.controlPoints[Phase::rightHandRot][0].x=0;
        phase.controlPoints[Phase::rightHandRot][0].y=0;
        phase.controlPoints[Phase::rightHandRot][0].z=0;
        phase.controlPoints[Phase::rightHandRot][1].x=0;
        phase.controlPoints[Phase::rightHandRot][1].y=0;
        phase.controlPoints[Phase::rightHandRot][1].z=0;
        phase.comTra[0].x=10;
        phase.comTra[0].y=0;
        phase.comTra[1].x=10;
        phase.comTra[1].y=0;
        phase.odometryOffset.x=0;
        phase.odometryOffset.y=0;
        phase.odometryOffset.z=0;

        phaseParameters[6]=phase;

        parameters.phaseParameters=phaseParameters;*/

      sprintf(name, "%s", file->d_name);
      for(int i = 0; i < 260; i++)
      {
        if(name[i] == '.') name[i] = 0;
      }
      strcpy(parameters.name, name);

      if(BikeRequest::getBMotionFromName(parameters.name) < BikeRequest::none)
      {
        params.push_back(parameters);
      }
      else
      {
        //OUTPUT(idText, text, "Warning: BikeRequest is missing the id for " << parameters.name);
        fprintf(stderr, "Warning: BikeRequest is missing the id for %s \n", parameters.name);
      }

    }
    file = readdir(dir);
  }
  closedir(dir);

#endif //LINUX

  for(int i = 0; i < BikeRequest::numOfBMotionIDs - 2; ++i)
  {
    int id = -1;
    for(unsigned int p = 0; p < params.size(); ++p)
    {
      if(BikeRequest::getBMotionFromName(&params[p].name[0]) == i)
      {
        id = i;
        break;
      }
    }
    if(id == -1)
    {
      //OUTPUT(idText, text, "Warning: The bike motion file for id " << BikeRequest::getName((BikeRequest::BMotionID) i) << " is missing.");
      fprintf(stderr, "Warning: The bike motion file for id %s is missing. \n", BikeRequest::getName((BikeRequest::BMotionID) i));
    }
  }

  //This is needed for adding new kicks
#ifndef RELEASE
  BIKEParameters newBMotion;
  strcpy(newBMotion.name, "newKick");
  params.push_back(newBMotion);
#endif
};

void BLAME::update(BikeEngineOutput& blameOutput
                   ,const MotionSelection& theMotionSelection
                   ,const FrameInfo& theFrameInfo
                   ,const RobotModel& theRobotModel
                   ,const FilteredJointData& theFilteredJointData
                   ,const FilteredSensorData& theFilteredSensorData
                   ,const WalkingEngineStandOutput& theWalkingEngineStandOutput
                   ,const MotionRequest& theMotionRequest
                   ,const RobotDimensions& theRobotDimensions
                   ,const TorsoMatrix& theTorsoMatrix
                   ,const JointCalibration& theJointCalibration
                   ,const MassCalibration& theMassCalibration)
{
  if(theMotionSelection.ratios[MotionRequest::bike] > 0.f)
  {
    data.setCycleTime(theFrameInfo.cycleTime);

    if(theMotionSelection.ratios[MotionRequest::bike] < 1.f && !compensated) compensate = true;

    data.setRobotModel(theRobotModel);

    //calculate COM with measured head joint data
    blameOutput.angles[JointData::HeadYaw] = theFilteredJointData.angles[JointData::HeadYaw];
    blameOutput.angles[JointData::HeadPitch] = theFilteredJointData.angles[JointData::HeadPitch];

    if(data.sitOutTransitionDisturbance(compensate, compensated, theFilteredSensorData, blameOutput, theWalkingEngineStandOutput, theFrameInfo))
    {
      if(data.activateNewMotion(theMotionRequest.bikeRequest, blameOutput.isLeavingPossible))
      {
        data.initData(compensated, theFrameInfo, theMotionRequest, theRobotDimensions, params, theFilteredJointData, theTorsoMatrix);
        data.setCurrentBikeRequest(theMotionRequest);
        data.setExecutedBikeRequest(blameOutput.executedBikeRequest);

        blameOutput.isLeavingPossible = false;

        blameOutput.odometryOffset.translation = Vector2<>(0.f, 0.f);
        blameOutput.odometryOffset.fromAngle(0.f);

        for(int i = JointData::LShoulderPitch; i < JointData::numOfJoints; ++i)
          blameOutput.jointHardness.hardness[i] = 100;
      }


      if(data.checkPhaseTime(theFrameInfo, theRobotDimensions, theFilteredJointData, theTorsoMatrix))
      {
        data.calcPhaseState();
        data.calcPositions();
        data.setStaticReference();
      }
      else
      {
        blameOutput.isLeavingPossible = true;
      }


      if(data.calcJoints(blameOutput, theRobotDimensions, theFilteredJointData))
      {
#ifndef RELEASE
        data.debugFormMode(params);
#endif
        data.balanceCOM(blameOutput, theRobotDimensions, theMassCalibration);
        data.calcJoints(blameOutput, theRobotDimensions, theFilteredJointData);
        data.mirrorIfNecessary(blameOutput);
      }


      data.addGyroBalance(blameOutput, theJointCalibration, theFilteredSensorData, theMotionSelection.ratios[MotionRequest::bike]);
    }

    //don't overwrite headJointRequest in combinator
    blameOutput.angles[JointData::HeadYaw] = JointData::ignore;
    blameOutput.angles[JointData::HeadPitch] = JointData::ignore;
  }
  else
  {
    compensated = false;
  }

  data.setEngineActivation(theMotionSelection.ratios[MotionRequest::bike]);
  data.ModifyData(theMotionRequest.bikeRequest, blameOutput, params);
}

//MAKE_MODULE(BLAME, Motion Control)

