/**
* @file CognitionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Tools/Configuration/ConfigMap.h"
//#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Tools/Global.h"

#include "Tools/Streams/InStreams.h"
#include <iostream>


//PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider) CognitionConfigurationDataProvider::theInstance = 0;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider() :
  theFieldDimensions(0),
  theCameraSettings(0),
  theCameraCalibration(0),
  theColorTable64(0),
  theRobotDimensions(0),
  theBehaviorConfiguration(0),
  theDamageConfiguration(0)
{
  //theInstance = this;

    cout << "Constructor de readFieldDimensions" << endl;
    readFieldDimensions();
    cout << "Constructor de readCameraSettings" << endl;
    readCameraSettings();
    cout << "Constructor de readCameraCalibration" << endl;
    readCameraCalibration();
    cout << "Constructor de readColorTable64" << endl;
    readColorTable64();
    cout << "Constructor de readRobotDimensions" << endl;
    readRobotDimensions();
    cout << "FIn constructor CognitionConfigurationDataProvider" << endl;

    readBehaviorConfiguration();
    readDamageConfiguration();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theCameraSettings)
    delete theCameraSettings;
  if(theCameraCalibration)
    delete theCameraCalibration;
  if(theColorTable64)
    delete theColorTable64;
  if(theBehaviorConfiguration)
    delete theBehaviorConfiguration;
  if(theDamageConfiguration)
    delete theDamageConfiguration;
  //theInstance = 0;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if(theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraSettings& cameraSettings)
{
  if(theCameraSettings)
  {
    cameraSettings = *theCameraSettings;
    delete theCameraSettings;
    theCameraSettings = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  if(theCameraCalibration)
  {
    cameraCalibration = *theCameraCalibration;
    delete theCameraCalibration;
    theCameraCalibration = 0;
  }
}

void CognitionConfigurationDataProvider::update(ColorTable64& colorTable64)
{
  if(theColorTable64)
  {
    colorTable64 = *theColorTable64;
    delete theColorTable64;
    theColorTable64 = 0;
  }
  //DEBUG_RESPONSE("module:CognitionConfigurationDataProvider:hashct",
  //{
  //  OUTPUT(idText, text, "colortable hash: " << colorTable64.hash());
  //});
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(BehaviorConfiguration& behaviorConfiguration)
{
  if(theBehaviorConfiguration)
  {
    behaviorConfiguration = *theBehaviorConfiguration;
    delete theBehaviorConfiguration;
    theBehaviorConfiguration = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfiguration& damageConfiguration)
{
  if(theDamageConfiguration)
  {
    damageConfiguration = *theDamageConfiguration;
    delete theDamageConfiguration;
    theDamageConfiguration = 0;
  }
}

void CognitionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  //InConfigFile stream(Global::getSettings().expandLocationFilename("field.cfg"));
  InConfigFile stream("/home/nao/.config/naoqi/Data/Config/Locations/Default/field.cfg");
  if(!stream.exists())
  {
    std::cout << "No se pudo leer field.cfg" << std::endl;
    return;
  }
  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();

  cout << "theFieldDimensions->xPosOpponentFieldBorder: " << theFieldDimensions->xPosOpponentFieldBorder << endl;
  cout << "theFieldDimensions->xPosOwnFieldBorder:      " << theFieldDimensions->xPosOwnFieldBorder      << endl;
  cout << "theFieldDimensions->yPosLeftFieldBorder:     " << theFieldDimensions->yPosLeftFieldBorder     << endl;
  cout << "theFieldDimensions->yPosRightFieldBorder:    " << theFieldDimensions->yPosRightFieldBorder    << endl;

  /** Solucion bruta para setear archivos de configuracion*/
 /*
  theFieldDimensions = new FieldDimensions;

  theFieldDimensions->xPosOpponentFieldBorder          = 3700;
  theFieldDimensions->xPosOpponentGoal                 = 3500;
  theFieldDimensions->xPosOpponentGoalpost             = 3000;
  theFieldDimensions->xPosOpponentGroundline           = 3000;
  theFieldDimensions->xPosOpponentSideCorner           = 3000;
  theFieldDimensions->xPosOpponentPenaltyArea          = 2400;
  theFieldDimensions->xPosOpponentPenaltyMark          = 1200;
  theFieldDimensions->xPosHalfWayLine                  = 0;
  theFieldDimensions->xPosOwnPenaltyArea               = -2400;
  theFieldDimensions->xPosOwnPenaltyMark               = -1200;
  theFieldDimensions->xPosOwnSideCorner                = -3000;
  theFieldDimensions->xPosOwnGroundline                = -3000;
  theFieldDimensions->xPosOwnGoalpost                  = -3000;
  theFieldDimensions->xPosOwnGoal                      = -3500;
  theFieldDimensions->xPosOwnFieldBorder               = -3700;

  theFieldDimensions->yPosLeftFieldBorder              = 2700;
  theFieldDimensions->yPosLeftSideline                 = 2000;
  theFieldDimensions->yPosLeftGroundline               = 2000;
  theFieldDimensions->yPosLeftPenaltyArea              = 1100;
  theFieldDimensions->yPosLeftGoal                     = 700;
  theFieldDimensions->yPosCenterGoal                   = 0;
  theFieldDimensions->yPosRightGoal                    = -700;
  theFieldDimensions->yPosRightPenaltyArea             = -1100;
  theFieldDimensions->yPosRightGroundline              = -2000;
  theFieldDimensions->yPosRightSideline                = -2000;
  theFieldDimensions->yPosRightFieldBorder             = -2700;

  //other dimensions
  theFieldDimensions->centerCircleRadius               = 600;
  theFieldDimensions->goalHeight                       = 800;
  theFieldDimensions->ballRadius                       = 33;
  theFieldDimensions->ballFriction                     = 150; // in mm/s^2
  theFieldDimensions->fieldLinesWidth                  = 50;
  theFieldDimensions->goalPostRadius                   = 48;

  //throw-in points
  theFieldDimensions->xPosThrowInPointOpponentHalf     = 1200;
  theFieldDimensions->xPosThrowInPointCenter           = 0;
  theFieldDimensions->xPosThrowInPointOwnHalf          = -1200;
*/
}

void CognitionConfigurationDataProvider::readCameraSettings()
{
    ASSERT(!theCameraSettings);

    // TODO
    //InConfigMap* stream = new InConfigMap(Global::getSettings().expandRobotLocationFilename("camera.cfg"));
    InConfigMap* stream = new InConfigMap("/home/nao/.config/naoqi/Data/Config/Locations/Default/camera.cfg");
    if(!stream->exists())
    {
        delete stream;
        std::cout << "No se pudo leer camera.cfg" << std::endl;
        return;
        //stream = new InConfigMap(Global::getSettings().expandLocationFilename("camera.cfg"));

    }

    if(stream->exists())
    {
        theCameraSettings = new CameraSettings;
        *stream >> *theCameraSettings;
    }

    cout << "theCameraSettings->gain.value "       << theCameraSettings->gain.value       << endl;
    cout << "theCameraSettings->brightness.value " << theCameraSettings->brightness.value << endl;

    delete stream;
  /*theCameraSettings = new CameraSettings;

    theCameraSettings->exposure.value                       =   150;
    theCameraSettings->exposureCorrection.value             =   0;
    theCameraSettings->gain.value                           =   81;
    theCameraSettings->red.value                            =   88;
    theCameraSettings->blue.value                           =   113;
    theCameraSettings->brightness.value                     =   128;
    theCameraSettings->contrast.value                       =   80;
    theCameraSettings->saturation.value                     =   255;
    theCameraSettings->hue.value                            =   0;
    theCameraSettings->sharpness.value                      =   2;
    theCameraSettings->green.value                          =   64;
    theCameraSettings->uvsatResult.value                    =   0;
    theCameraSettings->edgeEnhancementFactor.value          =   0;
    theCameraSettings->denoiseStrength.value                =   0;
    theCameraSettings->contrastCenter.value                 =   0;
    theCameraSettings->autoExposure.value                   =   0;
    theCameraSettings->autoGain.value                       =   0;
    theCameraSettings->autoBlacklevelCompensation.value     =   1;
    theCameraSettings->autoSaturationAdjustment.value       =   0;
    theCameraSettings->autoContrastCenter.value             =   0;
*/
}

void CognitionConfigurationDataProvider::readCameraCalibration()
{
  ASSERT(!theCameraCalibration);


  //InConfigMap stream(Global::getSettings().expandRobotFilename("cameraCalibration.cfg"));
  InConfigMap* stream = new InConfigMap("/home/nao/.config/naoqi/Data/Config/Robots/Default/cameraCalibration.cfg");

  if(stream->exists())
  {
    theCameraCalibration = new CameraCalibration;
    *stream >> *theCameraCalibration;
  }
  /*

    theCameraCalibration = new CameraCalibration;

    theCameraCalibration->cameraTiltCorrection            =   0;
    theCameraCalibration->bodyRollCorrection              =   0;
    theCameraCalibration->cameraPanCorrection             =   0;
    theCameraCalibration->bodyTiltCorrection              =   0;
    theCameraCalibration->cameraRollCorrection            =   0;
    theCameraCalibration->bodyTranslationCorrection.x     =   0;
    theCameraCalibration->bodyTranslationCorrection.y     =   0;
    theCameraCalibration->bodyTranslationCorrection.z     =   0;
    theCameraCalibration->upper2lowerRotation.x           =   0;
    theCameraCalibration->upper2lowerRotation.y           =   -0.715585;
    theCameraCalibration->upper2lowerRotation.z           =   0;
    theCameraCalibration->upper2lowerTranslation.x        =   5.1;
    theCameraCalibration->upper2lowerTranslation.y        =   0;
    theCameraCalibration->upper2lowerTranslation.z        =   44.09;
    theCameraCalibration->colorTemperature                =  CameraCalibration::defaultCamera;*/
}

void CognitionConfigurationDataProvider::readColorTable64()
{
  ASSERT(!theColorTable64);


  std::string ctName = "coltable";
  //const CameraCalibration& cameraCalibration = theCameraCalibration ? *theCameraCalibration : CognitionConfigurationDataProviderBase::theCameraCalibration;
  const CameraCalibration& cameraCalibration = *theCameraCalibration;

  if(cameraCalibration.colorTemperature > CameraCalibration::defaultCamera)
  {
    std::string ext = CameraCalibration::getName(cameraCalibration.colorTemperature);
    ext[0] = toupper(ext[0]);
    ctName += ext;
  }
  ctName += ".c64";

  //std::string fileName = Global::getSettings().expandRobotLocationFilename(ctName);
  //std::string fileName = "/home/nao/.config/naoqi/Data/Config/Locations/Default/" + ctName;
  InBinaryFile* stream = new InBinaryFile("/home/nao/coltable.c64");
  /*
  if(!stream->exists())
  {
    delete stream;
    //std::string newFileName = Global::getSettings().expandLocationFilename(ctName);
    std::string newFileName = "/home/nao/.config/naoqi/Data/Config/Locations/Default/" + ctName;
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }
  if(!stream->exists())
  {
    //delete stream;
    ////std::string newFileName = Global::getSettings().expandRobotLocationFilename("coltable.c64");
    //std::string newFileName = "/home/nao/.config/naoqi/Data/Config/Robots/Nao/coltable.c64";
    //printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    //fileName = newFileName;
    //stream = new InBinaryFile(fileName);
  }
  if(!stream->exists())
  {
    delete stream;
    //std::string newFileName = Global::getSettings().expandLocationFilename("coltable.c64");
    std::string newFileName = "/home/nao/.config/naoqi/Data/Config/Locations/Default/coltable.c64";
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }
  */

  if(stream->exists())
  {
    printf("Cubo bien le´ido");
    theColorTable64 = new ColorTable64;
    *stream >> *theColorTable64;
    printf("colortable hash: \"%s\"\n", theColorTable64->hash().c_str());
  }

  delete stream;

}

void CognitionConfigurationDataProvider::readRobotDimensions()
{
    ASSERT(!theRobotDimensions);

    //InConfigMap stream(Global::getSettings().expandRobotFilename("robotDimensions.cfg"));
    InConfigMap stream("/home/nao/.config/naoqi/Data/Config/Robots/Default/robotDimensions.cfg");
    if(stream.exists())
    {
        theRobotDimensions = new RobotDimensions;
        stream >> *theRobotDimensions;
    }
    else{
        cout << "No se encontro robotDimensions.cfg" << endl;
    }
    cout << "theRobotDimensions->headTiltToCameraTilt: "<< theRobotDimensions->headTiltToCameraTilt << endl;
/*
    theRobotDimensions = new RobotDimensions;
  theRobotDimensions->lengthBetweenLegs             =   100;
  theRobotDimensions->upperLegLength                =   100;
  theRobotDimensions->lowerLegLength                =   102.9;
  theRobotDimensions->heightLeg5Joint               =   45.19;
  theRobotDimensions->zLegJoint1ToHeadPan           =   211.5;
  theRobotDimensions->xHeadTiltToCamera             =   48.8;
  theRobotDimensions->zHeadTiltToCamera             =   23.81;
  theRobotDimensions->headTiltToCameraTilt          =   0.698132;
  theRobotDimensions->xHeadTiltToUpperCamera        =   53.9;
  theRobotDimensions->zHeadTiltToUpperCamera        =   67.9;
  theRobotDimensions->headTiltToUpperCameraTilt     =   0;
  theRobotDimensions->armOffset.x                   =   0;
  theRobotDimensions->armOffset.y                   =   98;
  theRobotDimensions->armOffset.z                   =   185;
  theRobotDimensions->yElbowShoulder                =   15;
  theRobotDimensions->upperArmLength                =   105;
  theRobotDimensions->lowerArmLength                =   130;
  theRobotDimensions->imageRecordingTime            =   0.034;
  theRobotDimensions->imageRecordingDelay           =   -0.003;*/
}

void CognitionConfigurationDataProvider::readBehaviorConfiguration()
{
  ASSERT(!theBehaviorConfiguration);

  //InConfigMap stream(Global::getSettings().expandLocationFilename("behavior.cfg"));
  InConfigMap stream("/home/nao/.config/naoqi/Data/Config/Locations/Default/behavior.cfg");
  if(stream.exists())
  {
    theBehaviorConfiguration = new BehaviorConfiguration;
    stream >> *theBehaviorConfiguration;
  }

  //theBehaviorConfiguration->agent = "soccer";
}

void CognitionConfigurationDataProvider::readDamageConfiguration()
{
  ASSERT(!theDamageConfiguration);

  //InConfigMap stream(Global::getSettings().expandRobotFilename("damageConfiguration.cfg"));
  InConfigMap stream("/home/nao/.config/naoqi/Data/Config/Robots/Nao/damageConfiguration.cfg");
  if(stream.exists())
  {
    theDamageConfiguration = new DamageConfiguration;
    stream >> *theDamageConfiguration;
  }
}
/*
bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
{
  CognitionConfigurationDataProvider* instrance = theInstance;
  return instrance && instrance->handleMessage2(message);
}

bool CognitionConfigurationDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idColorTable64:
    if(!theColorTable64)
      theColorTable64 = new ColorTable64;
    message.bin >> *theColorTable64;
    return true;

  case idWriteColorTable64:
    if(!theColorTable64)
      theColorTable64 = new ColorTable64;
    message.bin >> *theColorTable64;
    {
      std::string ctName = "coltable";
      const CameraCalibration& cameraCalibration = theCameraCalibration ? *theCameraCalibration : CognitionConfigurationDataProviderBase::theCameraCalibration;
      if(cameraCalibration.colorTemperature > CameraCalibration::defaultCamera)
      {
        std::string ext = CameraCalibration::getName(cameraCalibration.colorTemperature);
        ext[0] = toupper(ext[0]);
        ctName += ext;
      }
      ctName += ".c64";
      OutBinaryFile stream(Global::getSettings().expandRobotLocationFilename(ctName));
      if(stream.exists())
      {
        stream << *theColorTable64;
      }
      else
      {
        OutBinaryFile stream(Global::getSettings().expandLocationFilename(ctName));
        stream << *theColorTable64;
      }
    }
    return true;

  default:
    return false;
  }
}*/

//MAKE_MODULE(CognitionConfigurationDataProvider, Infrastructure)
