/**
* @file CameraProvider.h
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

//#include "Tools/Module/Module.h"
//#include "Tools/Settings.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/ImageInfo.h"

class NaoCamera;
/*
MODULE(CameraProvider)
  REQUIRES(CameraSettings)
  REQUIRES(Image)
  USES(ImageRequest)
  PROVIDES_WITH_OUTPUT(Image)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageInfo)
  PROVIDES(CameraInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
  PROVIDES_WITH_MODIFY(CognitionFrameInfo)
END_MODULE
*/

class CameraProvider //: public CameraProviderBase
{
private:
  //PROCESS_WIDE_STORAGE_STATIC(CameraProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* camera;
  //void* theInstance;
  static CameraProvider* theInstance;

public:
  void update(Image& image, const CameraSettings& theCameraSettings);
  void update(ImageInfo& imageInfo, const ImageRequest& theImageRequest);
  void update(CameraInfo& cameraInfo) {} // nothing to do here
  void update(FrameInfo& frameInfo, const Image& theImage);
  void update(CognitionFrameInfo& cognitionFrameInfo, const Image& theImage);

  int numberOfImages;

public:
  /**
  * Default constructor.
  */
  CameraProvider();

  /**
  * Destructor.
  */
  ~CameraProvider();

  /**
  * The method returns whether a new image is available.
  * @return Is an new image available?
  */

  static bool isFrameDataComplete();

  /**
  * The method waits for a new image.
  */
  static void waitForFrameData();
};
