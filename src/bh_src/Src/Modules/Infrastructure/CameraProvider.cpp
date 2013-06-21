/**
* @file CameraProvider.cpp
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>
#include <iostream>

#include "CameraProvider.h"
//#include "Representations/Perception/JPEGImage.h"
#include "Platform/Camera.h"
#include "Platform/SystemCall.h"

//PROCESS_WIDE_STORAGE(CameraProvider) CameraProvider::theInstance = 0;
CameraProvider* CameraProvider::theInstance = 0;

CameraProvider::CameraProvider()
{
#ifdef CAMERA_INCLUDED
  camera = new NaoCamera();
#else
  camera = NULL;
#endif
  //theInstance = (void*) malloc(sizeof(CameraProvider));
  theInstance = this;

  numberOfImages = 0;
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
  if(camera)
    delete camera;
#endif
  //theInstance = 0;
}

void CameraProvider::update(Image& image, const CameraSettings& theCameraSettings)
{
#ifdef CAMERA_INCLUDED
  //ASSERT(camera->getImage());
  unsigned char* imagen = const_cast<unsigned char*>(camera->getImage1());

  /*
  // -----------------------------------------
  if(numberOfImages==0)
  {
  ofstream outImage("/home/nao/Image0.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==1)
  {
  ofstream outImage("/home/nao/Image1.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==2)
  {
  ofstream outImage("/home/nao/Image2.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==3)
  {
  ofstream outImage("/home/nao/Image3.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==4)
  {
  ofstream outImage("/home/nao/Image4.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==5)
  {
  ofstream outImage("/home/nao/Image5.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==6)
  {
  ofstream outImage("/home/nao/Image6.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==7)
  {
  ofstream outImage("/home/nao/Image7.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==8)
  {
  ofstream outImage("/home/nao/Image8.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }

  else if(numberOfImages==9)
  {
  ofstream outImage("/home/nao/Image9.txt");

   for (int i=0;i<640*480*2; ++i)
    {
        outImage << (unsigned int) *(imagen++) << "\t";
    }

    outImage.close();
    numberOfImages++;
  }



  // -----------------------------------------
  */


  image.setImage(imagen /* const_cast<unsigned char*>(camera->getImage1()) */);
  image.timeStamp = camera->getTimeStamp1();
  //cout << "\t pixel: "<< (unsigned int)(*imagen) << endl;
  //cout << "\t TImeStamp: "<< image.timeStamp << endl;

  //camera->setSettings(theCameraSettings);
  //DEBUG_RESPONSE("module:CameraProvider:assertCameraSettings", camera->assertCameraSettings(););
  //DEBUG_RESPONSE("module:CameraProvider:writeCameraSettings", camera->writeCameraSettings(););
#endif // CAMERA_INCLUDED
  //DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););

  //cout << "saliendo del Update de CameraProvider" << endl;
}

void CameraProvider::update(ImageInfo& imageInfo, const ImageRequest& theImageRequest)
{
#ifdef CAMERA_INCLUDED
  imageInfo.prevCamera = imageInfo.camera;
  imageInfo.camera = camera->getCurrentCamera();
  if(theImageRequest.requestedCamera != imageInfo.camera)
  {
    //camera->switchCamera(theImageRequest.requestedCamera);
    //imageInfo.camera = camera->getCurrentCamera();
      std::cout << "Alerta: Se ha solicitado un cambio de c´amara, ahora existen las dos c´amaras" << std::endl;
  }
#endif
  return;
}

void CameraProvider::update(FrameInfo& frameInfo, const Image& theImage)
{
  frameInfo.time = theImage.timeStamp;
}

void CameraProvider::update(CognitionFrameInfo& cognitionFrameInfo, const Image& theImage)
{
  cognitionFrameInfo.time = theImage.timeStamp;
}

bool CameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->camera->getImage1() ? true : false;
else
#endif
    return true;
}

void CameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  static bool reset = false;
  //DEBUG_RESPONSE("module:CameraProvider:resetCamera", reset = true;)
  if(theInstance && !theInstance->camera->captureNew1() || reset) // Método de NaoCamera (V4L2) que llena el buffer de la cámara
  {
    cout << "DESTRUYENDO TODO" << endl;
    //OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting camera.");
    delete theInstance->camera;
    theInstance->camera = NULL;
    theInstance->camera = new NaoCamera();
    reset = false;
  }
#endif
}

//MAKE_MODULE(CameraProvider, Infrastructure)
