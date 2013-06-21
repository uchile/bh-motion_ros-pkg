/**
* \file Platform/linux/NaoCamera.cpp
* Interface to the Nao camera using linux-uvc.
* \author Colin Graf
* \author Thomas Röfer
*/
//#define NO_NAO_EXTENSIONS
#if defined(TARGET_SIM) && !defined(NO_NAO_EXTENSIONS)
//#define NO_NAO_EXTENSIONS
#endif

//#define USE_USERPTR

#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <poll.h>
#ifdef USE_USERPTR
#include <malloc.h> // memalign
#endif

#include "/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/linux/videodev2.h"
#include "/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/linux/version.h"

#include "/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/unistd.h"

#include "NaoCamera.h"
#include "BHAssert.h"
#include "SystemCall.h"
#include "Representations/Infrastructure/CameraInfo.h"
//#include "../../Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"

#undef __STRICT_ANSI__
//#include <linux/videodev2.h>
#include "/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/linux/videodev2.h"

//#include <linux/version.h>
#include "/home/nao/nao-atom-cross-toolchain-1.12.5.3_2012-06-03/sysroot/usr/include/linux/version.h"

//#include <linux/i2c-dev.h> Ruta original de BHuman
#include "../../../Util/I2C/include/linux/i2c-dev.h"
#include <iostream>


#define __STRICT_ANSI__

#ifndef V4L2_CID_AUTOEXPOSURE
#  define V4L2_CID_AUTOEXPOSURE     (V4L2_CID_BASE+32)
#endif

#ifndef V4L2_CID_CAM_INIT
#  define V4L2_CID_CAM_INIT         (V4L2_CID_BASE+33)
#endif

#ifndef V4L2_CID_AUDIO_MUTE
#  define V4L2_CID_AUDIO_MUTE       (V4L2_CID_BASE+9)
#endif

#ifndef V4L2_CID_POWER_LINE_FREQUENCY
#  define V4L2_CID_POWER_LINE_FREQUENCY  (V4L2_CID_BASE+24)
enum v4l2_power_line_frequency
{
  V4L2_CID_POWER_LINE_FREQUENCY_DISABLED  = 0,
  V4L2_CID_POWER_LINE_FREQUENCY_50HZ  = 1,
  V4L2_CID_POWER_LINE_FREQUENCY_60HZ  = 2,
};

#define V4L2_CID_HUE_AUTO      (V4L2_CID_BASE+25)
#define V4L2_CID_WHITE_BALANCE_TEMPERATURE  (V4L2_CID_BASE+26)
#define V4L2_CID_SHARPNESS      (V4L2_CID_BASE+27)
#define V4L2_CID_BACKLIGHT_COMPENSATION   (V4L2_CID_BASE+28)

#define V4L2_CID_CAMERA_CLASS_BASE     (V4L2_CTRL_CLASS_CAMERA | 0x900)
#define V4L2_CID_CAMERA_CLASS       (V4L2_CTRL_CLASS_CAMERA | 1)

#define V4L2_CID_EXPOSURE_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+1)
enum  v4l2_exposure_auto_type
{
  V4L2_EXPOSURE_MANUAL = 0,
  V4L2_EXPOSURE_AUTO = 1,
  V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
  V4L2_EXPOSURE_APERTURE_PRIORITY = 3
};
#define V4L2_CID_EXPOSURE_ABSOLUTE    (V4L2_CID_CAMERA_CLASS_BASE+2)
#define V4L2_CID_EXPOSURE_AUTO_PRIORITY    (V4L2_CID_CAMERA_CLASS_BASE+3)

#define V4L2_CID_PAN_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+4)
#define V4L2_CID_TILT_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+5)
#define V4L2_CID_PAN_RESET      (V4L2_CID_CAMERA_CLASS_BASE+6)
#define V4L2_CID_TILT_RESET      (V4L2_CID_CAMERA_CLASS_BASE+7)

#define V4L2_CID_PAN_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+8)
#define V4L2_CID_TILT_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+9)

#define V4L2_CID_FOCUS_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+10)
#define V4L2_CID_FOCUS_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+11)
#define V4L2_CID_FOCUS_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+12)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26) */

NaoCamera* NaoCamera::theInstance = 0;

NaoCamera::NaoCamera() :
  currentCamera(ImageInfo::lowerCamera),
  currentBuf0(0),
  currentBuf1(0),
  timeStamp0(0),
  timeStamp1(0)

{

  //cout << "NO_NAO_EXTENSIONS: " << NO_NAO_EXTENSIONS << endl;
  cout << "Hola, estoy pasando por el constructor" << endl;

  ASSERT(theInstance == 0);
  theInstance = this;

  ImageInfo::Camera current = currentCamera;
  ImageInfo::Camera other = currentCamera == ImageInfo::upperCamera ? ImageInfo::lowerCamera : ImageInfo::upperCamera;

  //VERIFY(verifyNaoVersion());

  /*
  initSelectCamera(ImageInfo::upperCamera);                 // Establecer conexión con cámara de arriba I2C
  initOpenVideoDevice();                                    // Apertura del dispositivo /dev/video0
  //cout << "Primer y segundo llamado a ioctl"<<endl;
  //cout << "Errno: " << errno << endl;
  initSetCameraDefaults();                                  // Seteo de valores default del dispositivo. Primer llamada a ioctl
  initSelectCamera(ImageInfo::lowerCamera);                 // Establecer conexión con cámara de abajo I2C
  //cout << "Tercero y cuarto llamado a ioctl"<<endl;
  //cout << "Errno: " << errno << endl;
  initSetCameraDefaults();                                  // Seteo de valores default del dispositivo. Segunda llamada a ioctl

  initSelectCamera(other);                                  // Selección de la cámara de arriba
  initRequestAndMapBuffers();                               // Creación de buffers
  initQueueAllBuffers(); //VIDIOC_QBUF                      // Preparación de buffers: Buffer del dispositivo o de usuario

  initResetCrop();                                          // Seteo de (...) -> Revisar
  initSetImageFormat();                                     // Seteo de opciones de tamaño, espacio de color. Formato de imagen
  initSetFrameRate();                                       // Seteo de frame rate 30fps
  initDefaultControlSettings();                             // Seteo de Opciones default de control: autoganancia, exposición, balance de blancos.
                                                            // Se obtiene de CameraSettings si no está definido NO_NAO_EXTENSIONS

  initSelectCamera(current);                                // Selección de cámara de abajo
  initResetCrop();                                          // Seteo de (...) -> Revisar
  initSetImageFormat();                                     // Seteo de opciones de tamaño, espacio de color. Formato de imagen
  initSetFrameRate();                                       // Seteo de frame rate 30fps
  initDefaultControlSettings();                             // Seteo de Opciones default de control: autoganancia, exposición, balance de blancos.
  */

  /** Inicialización para dos cámaras*/
  initOpenVideoDevice();        // Apertura de los dos dispositivos /dev/video0 - /dev/video1
  initSetCameraDefaults();      // Seteo de todos los valores por default del dispositivo. Primer llamada a ioctl

  initRequestAndMapBuffers0();  // Creación de buffers para video0
  initRequestAndMapBuffers1();  // Creación de buffers para video1

  initQueueAllBuffers0();       // Preparación de buffers para video0. Uso de buffer del dispositivo
  initQueueAllBuffers1();       // Preparación de buffers para video1. Uso de buffer del dispositivo

  //initResetCrop();              // ok - No usa CameraSettings
  initSetImageFormat();         // ok - No usa CameraSettings - Setea el dispositivo como capturador, 640x480, YUV422, Progressive
  initSetFrameRate();           // ok - No usa CameraSettings
  initDefaultControlSettings(); // ok - Unico uso de Camera Settings. Definición de parámetros default:
                                //      - Autoexposure
                                //      - AutoWhiteBalance
                                //      - AutoGain
                                //      - Hue_Auto
                                //      - Exposure_Auto
                                //      - HFlip
                                //      - VFlip


  startCapturing(); //VIDIOC_STREAMON                       // Comienzo de captura de datos
}

NaoCamera::~NaoCamera()
{
  // disable streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd0, VIDIOC_STREAMOFF, &type) != -1);
  VERIFY(ioctl(fd1, VIDIOC_STREAMOFF, &type) != -1);

  // unmap buffer 0
  for(int i = 0; i < frameBufferCount; ++i)
#ifdef USE_USERPTR
    free(mem0[i]);
#else
    munmap(mem0[i], memLength0[i]);
#endif

  // unmap buffer 1
  for(int i = 0; i < frameBufferCount; ++i)
#ifdef USE_USERPTR
    free(mem1[i]);
#else
    munmap(mem1[i], memLength1[i]);
#endif

  // close the device
  close(fd0);
  close(fd1);
  free(buf0);
  free(buf1);

  theInstance = 0;
}

void NaoCamera::releaseBuffer0()
{
   if(ioctl(fd0, VIDIOC_QBUF, currentBuf0) == -1)
       cout << "Error en releaseBuffer dev0"<< endl;
}

void NaoCamera::releaseBuffer1()
{
   if(ioctl(fd1, VIDIOC_QBUF, currentBuf1) == -1)
       cout << "Error en releaseBuffer dev1"<< endl;
}

bool NaoCamera::captureNew0()
{
  //cout << " ---------- CAPTURE NEW 0 ----------- " << endl;
  //cout << "currentBuf0: " << currentBuf0 << endl;
  //cout << "buf0: " << buf0 << endl;
  //cout << "buf0 bytes used: " << buf0->bytesused << endl;


  // requeue the buffer of the last captured image which is obsolete now
  if(currentBuf0)
  {
    //BH_TRACE;
    //cout << "currentBuf: " << currentBuf0<< endl;

    VERIFY(ioctl(fd0, VIDIOC_QBUF, currentBuf0) != -1);
  }

  //BH_TRACE;
  struct pollfd pollfd = {fd0, POLLIN | POLLPRI, 0};
  int polled = poll(&pollfd, 1, 500); // Fail after missing 15 frames (0.5s)

  //cout << "POLLED: " << polled << endl;

  if(polled < 0)
  {
    //OUTPUT_ERROR("NaoCamera: Cannot poll. Reason: " << strerror(errno));
      cout << "NaoCamera:Cannot poll. Reason: " << strerror(errno) << endl;
    ASSERT(false);
  }
  else if(polled == 0)
  {
    //OUTPUT_ERROR("NaoCamera: 0.5 seconds passed and there's still no image to read from the camera. Terminating.");
      cout << "NaoCamera: 0.5 seconds passed and there's still no image to read from the camera. Terminating." << endl;
    //return false;
  }
  else if(pollfd.revents & (POLLERR | POLLNVAL))
  {
    cout << pollfd.revents << endl;
    cout << POLLERR << endl;
    cout << POLLNVAL << endl;

    //OUTPUT_ERROR("NaoCamera: Polling failed.");
    cout << "NaoCamera: Polling failed." << endl;
    return false;
  }

  //cout << "PRE IOCTL" << endl;
  //cout << "\t buf0: " << buf0 << endl;
  //cout << "\t buf0 bytes used: " << buf0->bytesused << endl;
  //cout << "\t SIZE: " << SIZE << endl;


  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd0, VIDIOC_DQBUF, buf0) != -1);
  timeStamp0 = SystemCall::getCurrentSystemTime();
  //BH_TRACE;

  //cout << "POST IOCTL" << endl;
  //cout << "\t buf0: " << buf0 << endl;
  //cout << "\t buf0 bytes used: " << buf0->bytesused << endl;
  //cout << "\t SIZE: " << SIZE << endl;

  //ASSERT(buf0->bytesused == SIZE);
  if(buf0->bytesused != SIZE)
      cout << "Tamaño de buf0 DISTINTO de SIZE"<< endl;


  currentBuf0 = buf0;

  static bool shout = true;
  if(shout)
  {
    shout = false;
    printf("Camera is working\n");
  }

  return true;
}

bool NaoCamera::captureNew1()
{
    //cout << "\t setControlSetting0 CONTRAST: "                << setControlSetting0(V4L2_CID_CONTRAST, 0)               << endl;
    //cout << "\t setControlSetting1 CONTRAST: "                << setControlSetting1(V4L2_CID_CONTRAST, 0)               << endl;
  // requeue the buffer of the last captured image which is obsolete now
  if(currentBuf1)
  {
    //BH_TRACE;

    //cout << "currentBuf: " << currentBuf1<< endl;
    //releaseBuffer1();

   VERIFY(ioctl(fd1, VIDIOC_QBUF, currentBuf1) != -1);
  }
  //BH_TRACE;

  struct pollfd pollfd = {fd1, POLLIN | POLLPRI, 0};
  int polled = poll(&pollfd, 1, 500); // Fail after missing 15 frames (0.5s)

  //cout << "polled: " << polled << endl;

  if(polled < 0)
  {
    //OUTPUT_ERROR("NaoCamera: Cannot poll. Reason: " << strerror(errno));
    ASSERT(false);
  }
  else if(polled == 0)
  {
    //OUTPUT_ERROR("NaoCamera: 0.5 seconds passed and there's still no image to read from the camera. Terminating.");
    return false;
  }
  else if(pollfd.revents & (POLLERR | POLLNVAL))
  {
    //cout << pollfd.revents << endl;
    //cout << POLLERR << endl;
    //cout << POLLNVAL << endl;

    //OUTPUT_ERROR("NaoCamera: Polling failed.");
    return false;
  }
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd1, VIDIOC_DQBUF, buf1) != -1);
  timeStamp1 = SystemCall::getCurrentSystemTime();
  //BH_TRACE;

  //ASSERT(buf1->bytesused == SIZE);
  if(buf1->bytesused != SIZE)
      cout << "Tamaño de buf1 DISTINTO de SIZE"<< endl;


  currentBuf1 = buf1;

  static bool shout = true;
  if(shout)
  {
    shout = false;
    printf("Camera is working\n");
  }

  return true;
}

const unsigned char* NaoCamera::getImage0() const
{
#ifdef USE_USERPTR
  return currentBuf0 ? (unsigned char*)currentBuf0->m.userptr : 0;
#else
    return currentBuf0 ? static_cast<unsigned char*>(mem0[currentBuf0->index]) : 0;
#endif
}

const unsigned char* NaoCamera::getImage1() const
{
#ifdef USE_USERPTR
  return currentBuf1 ? (unsigned char*)currentBuf1->m.userptr : 0;
#else
    //cout << "\t setControlSetting0 CONTRAST: "                << setControlSetting0(V4L2_CID_CONTRAST, 0)               << endl;
    //cout << "\t setControlSetting0 CONTRAST: "                << setControlSetting1(V4L2_CID_CONTRAST, 0)               << endl;
  return currentBuf1 ? static_cast<unsigned char*>(mem1[currentBuf1->index]) : 0;
#endif
}

unsigned int NaoCamera::getTimeStamp0() const
{
  ASSERT(currentBuf0);
  return timeStamp0;
}

unsigned int NaoCamera::getTimeStamp1() const
{
  ASSERT(currentBuf1);
  return timeStamp1;
}

int NaoCamera::getControlSetting0(unsigned int id)
{
  /*
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd0, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return -1;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return -1; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return -1; // not supported

  struct v4l2_control control_s;
  control_s.id = id;
  if(ioctl(fd0, VIDIOC_G_CTRL, &control_s) < 0)
    return -1;
  if(control_s.value == queryctrl.default_value)
    return -1;
  return control_s.value;
  */
   struct v4l2_control control_s;
  control_s.id = id;
  if (ioctl(fd0, VIDIOC_G_CTRL, &control_s) < 0)
     {
       printf("CAMERA::Warning::Getting control failed.\n");
        return -1;
     }
   return control_s.value;

}

int NaoCamera::getControlSetting1(unsigned int id)
{
  /*
    struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd1, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return -1;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return -1; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return -1; // not supported

  struct v4l2_control control_s;
  control_s.id = id;
  if(ioctl(fd1, VIDIOC_G_CTRL, &control_s) < 0)
    return -1;
  if(control_s.value == queryctrl.default_value)
    return -1;
  return control_s.value;
  */

   struct v4l2_control control_s;
   control_s.id = id;
   if (ioctl(fd1, VIDIOC_G_CTRL, &control_s) < 0)
      {
        printf("CAMERA::Warning::Getting control failed.\n");
         return -1;
      }
   return control_s.value;

}

bool NaoCamera::setControlSettings0(std::list<CameraSettings::V4L2Setting> controlsettings)
{
  std::list<CameraSettings::V4L2Setting>::const_iterator it = controlsettings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = controlsettings.end();
  bool success = true;
  for(; it != end; it++)
    if(!setControlSetting0((*it).command, (*it).value))
      success = false;
  return success;
}

bool NaoCamera::setControlSettings1(std::list<CameraSettings::V4L2Setting> controlsettings)
{
  std::list<CameraSettings::V4L2Setting>::const_iterator it = controlsettings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = controlsettings.end();
  bool success = true;
  for(; it != end; it++)
    if(!setControlSetting1((*it).command, (*it).value))
      success = false;
  return success;
}

bool NaoCamera::setControlSetting0(unsigned int id, int value)
{
  /*
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd0, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return false;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return false; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return false; // not supported

  // clip value
  if(value < queryctrl.minimum)
    value = queryctrl.minimum;
  if(value > queryctrl.maximum)
    value = queryctrl.maximum;
  if(value < 0)
    value = queryctrl.default_value;

  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;
  if(ioctl(fd0, VIDIOC_S_CTRL, &control_s) < 0)
    return false;
  return true;
  */

   struct v4l2_control control_s;
   control_s.id = id;
   control_s.value = value;

   int counter = 0;

    // Have to make sure the setting "sticks"
    while(getControlSetting0(id) != value)
    {
        if (ioctl(fd0, VIDIOC_S_CTRL, &control_s) < 0)
        {
            printf("CAMERA::Warning::Control setting failed.\n");
            return false;
        }
        counter++;
        if(counter > 30)
          {
            printf("CAMERA::Warning::Timeout while setting a parameter.\n");
            return false;
          }
    }
    return true;

}

bool NaoCamera::setControlSetting1(unsigned int id, int value)
{
  /*
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd1, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return false;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return false; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return false; // not supported

  // clip value
  if(value < queryctrl.minimum)
    value = queryctrl.minimum;
  if(value > queryctrl.maximum)
    value = queryctrl.maximum;
  if(value < 0)
    value = queryctrl.default_value;

  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;
  if(ioctl(fd1, VIDIOC_S_CTRL, &control_s) < 0)
    return false;
  return true;
  */

    struct v4l2_control control_s;
    control_s.id = id;
    control_s.value = value;

    int counter = 0;

     // Have to make sure the setting "sticks"
     while(getControlSetting1(id) != value)
     {
         if (ioctl(fd1, VIDIOC_S_CTRL, &control_s) < 0)
         {
             printf("CAMERA::Warning::Control setting failed.\n");
             return false;
         }
         counter++;
         if(counter > 30)
           {
             printf("CAMERA::Warning::Timeout while setting a parameter.\n");
             return false;
           }
     }
     return true;


}

void NaoCamera::setSettings(const CameraSettings& newset)
{
  if(settings == newset)
    return;

  std::list<CameraSettings::V4L2Setting> changes = settings.getChangesAndAssign(newset);

  //ImageInfo::Camera current = getCurrentCamera();
  //ImageInfo::Camera other = current == ImageInfo::lowerCamera ? ImageInfo::upperCamera : ImageInfo::lowerCamera;

  //switchCamera(other);
  VERIFY(setControlSettings0(changes));
  //switchCamera(current);
  VERIFY(setControlSettings1(changes));
}

/*
ImageInfo::Camera NaoCamera::switchCamera(ImageInfo::Camera camera)
{
#ifndef NO_NAO_EXTENSIONS
  unsigned char cmd[2] = {camera, 0};
  int flip = camera == ImageInfo::upperCamera ? 1 : 0;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // disable streaming
  //VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);
  cout << "ioctl disable streaming: " << ioctl(fd, VIDIOC_STREAMOFF, &type) << endl;

  // switch camera
  int i2cfd = openI2CAdapter();
  //VERIFY(i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) != -1);
  cout << "switchCamera i2c_smbus_write_block_data(i2cfd, 220, 1, cmd): " << i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) << endl;

  closeI2CAdapter(i2cfd);
  VERIFY(setControlSetting(V4L2_CID_VFLIP, flip));
  VERIFY(setControlSetting(V4L2_CID_HFLIP, flip));


  // enable streaming
  //VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);
  cout << "ioctl enable streaming: " << ioctl(fd, VIDIOC_STREAMON, &type) << endl;

  currentCamera = camera;
  return camera;
#else
  return ImageInfo::lowerCamera;
#endif // NO_NAO_EXTENSIONS
}


ImageInfo::Camera NaoCamera::switchToUpper()
{
  if(currentCamera == ImageInfo::upperCamera)
    return ImageInfo::upperCamera;

  return switchCamera(ImageInfo::upperCamera);
}

ImageInfo::Camera NaoCamera::switchToLower()
{
  if(currentCamera == ImageInfo::lowerCamera)
    return ImageInfo::lowerCamera;

  return switchCamera(ImageInfo::lowerCamera);
}


void NaoCamera::initSelectCamera(ImageInfo::Camera camera)
{
#ifndef NO_NAO_EXTENSIONS

  cout << "== initSelectCamera ==" << endl;

  unsigned char cmd[2] = {camera, 0};

  int i2cfd = openI2CAdapter();

  //VERIFY(i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) != -1); // select camera
  cout << "initSelectCamera i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) :" << i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) << endl;

  closeI2CAdapter(i2cfd);
#endif

  currentCamera = camera;
}
*/

void NaoCamera::initOpenVideoDevice()
{
  // open device 0
  fd0 = open("/dev/video0", O_RDWR);

  fd1 = open("/dev/video1", O_RDWR);

  //cout << "\t initOpenVideoDevice CONTRAST: "                << setControlSetting0(V4L2_CID_CONTRAST, 0)               << endl;
  //cout << "\t initOpenVideoDevice CONTRAST: "                << setControlSetting1(V4L2_CID_CONTRAST, 0)               << endl;

  //cout <<"fd0: "<< fd0 << endl;
  //cout <<"fd1: "<< fd1 << endl;

  //cout << "errno open video device: " << errno << endl;

  /*
  cameraAdapterFd0 = open("/dev/i2c-camera0", O_RDWR);
  cameraAdapterFd1 = open("/dev/i2c-camera1", O_RDWR);

  if(cameraAdapterFd0 == -1)
      printf("CAMERA::ERROR::Camera adapter FD is WRONG.\n");
  VERIFY(ioctl(cameraAdapterFd0, 0x703, 8));

  unsigned char cmd0[2] = { (unsigned char) 0x01, 0 };
  i2c_smbus_write_block_data(cameraAdapterFd0, 220, 1, cmd0);

  unsigned char cmd1[2] = { (unsigned char) 0x02, 0 };
  i2c_smbus_write_block_data(cameraAdapterFd1, 220, 1, cmd1);
  */

  ASSERT(fd0 != -1);
  ASSERT(fd1 != -1);
}

void NaoCamera::initSetCameraDefaults()
{
  // set default parameters
#ifndef NO_NAO_EXTENSIONS

    /** Seteo de parametros para video 0 */
//  struct v4l2_control control0;
//  memset(&control0, 0, sizeof(control0));
//  control0.id = V4L2_CID_CAM_INIT;
//  control0.value = 0;

//  VERIFY(ioctl(fd0, VIDIOC_S_CTRL, &control0) >= 0);

  v4l2_std_id esid0_0 = WIDTH == 320 ? 0x04000000UL : 0x08000000UL;

  VERIFY(!ioctl(fd0, VIDIOC_S_STD, &esid0_0));


  /** Seteo de parametros para video 1 */
//  struct v4l2_control control1;
//  memset(&control1, 0, sizeof(control1));
//  control1.id = V4L2_CID_CAM_INIT;
//  control1.value = 0;

//  VERIFY(ioctl(fd1, VIDIOC_S_CTRL, &control1) >= 0);

  v4l2_std_id esid0_1 = WIDTH == 320 ? 0x04000000UL : 0x08000000UL;

  VERIFY(!ioctl(fd1, VIDIOC_S_STD, &esid0_1));


#endif
}

void NaoCamera::initSetImageFormat()
{
  // set format
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(struct v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  VERIFY(!ioctl(fd0, VIDIOC_S_FMT, &fmt));
  VERIFY(!ioctl(fd1, VIDIOC_S_FMT, &fmt));

  ASSERT(fmt.fmt.pix.sizeimage == SIZE);
}

void NaoCamera::initSetFrameRate()
{
  // set frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(struct v4l2_streamparm));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd0, VIDIOC_G_PARM, &fps));
  VERIFY(!ioctl(fd1, VIDIOC_G_PARM, &fps));

  fps.parm.capture.timeperframe.numerator = 1;
  fps.parm.capture.timeperframe.denominator = 30;
  VERIFY(ioctl(fd0, VIDIOC_S_PARM, &fps) != -1);
  VERIFY(ioctl(fd1, VIDIOC_S_PARM, &fps) != -1);
}

void NaoCamera::initRequestAndMapBuffers0()
{
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
  cout << "\t USE_PTR definido. memory=V4L2_MEMORY_USEPTR" << endl;
  rb.memory = V4L2_MEMORY_USERPTR;
#else
  cout << "\t USE_PTR no definido. memory=V4L2_MEMORY_MMAP" << endl;
  rb.memory = V4L2_MEMORY_MMAP;
#endif
  VERIFY(ioctl(fd0, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  buf0 = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
#ifdef USE_USERPTR
  unsigned int bufferSize = SIZE;
  unsigned int pageSize = getpagesize();
  bufferSize = (bufferSize + pageSize - 1) & ~(pageSize - 1);
#endif
  for(int i = 0; i < frameBufferCount; ++i)
  {
#ifdef USE_USERPTR
    memLength0[i] = bufferSize;
    mem0[i] = memalign(pageSize, bufferSize);
#else
    buf0->index = i;
    buf0->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf0->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd0, VIDIOC_QUERYBUF, buf0) != -1);
    memLength0[i] = buf0->length;
    mem0[i] = mmap(0, buf0->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd0, buf0->m.offset);
    ASSERT(mem0[i] != MAP_FAILED);

    cout << "buf0 bytesused: " << buf0->bytesused<< endl;
    cout << "memLength0[" << i <<"]: " << memLength0[i] << endl;
    cout << "mem0[" << i <<"]: " << mem0[i] << endl;
#endif
  }
}

void NaoCamera::initRequestAndMapBuffers1()
{
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
  cout << "\t USE_PTR definido. memory=V4L2_MEMORY_USEPTR" << endl;
  rb.memory = V4L2_MEMORY_USERPTR;
#else
  cout << "\t USE_PTR no definido. memory=V4L2_MEMORY_MMAP" << endl;
  rb.memory = V4L2_MEMORY_MMAP;
#endif
  VERIFY(ioctl(fd1, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  buf1 = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
#ifdef USE_USERPTR
  unsigned int bufferSize = SIZE;
  unsigned int pageSize = getpagesize();
  bufferSize = (bufferSize + pageSize - 1) & ~(pageSize - 1);
#endif
  for(int i = 0; i < frameBufferCount; ++i)
  {
#ifdef USE_USERPTR
    memLength1[i] = bufferSize;
    mem1[i] = memalign(pageSize, bufferSize);
#else
    buf1->index = i;
    buf1->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf1->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd1, VIDIOC_QUERYBUF, buf1) != -1);
    memLength1[i] = buf1->length;
    mem1[i] = mmap(0, buf1->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd1, buf1->m.offset);
    ASSERT(mem1[i] != MAP_FAILED);
#endif
  }
}

void NaoCamera::initQueueAllBuffers0()
{
  // queue the buffers
  for(int i = 0; i < frameBufferCount; ++i)
  {
    buf0->index = i;
    buf0->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
    buf0->memory = V4L2_MEMORY_USERPTR;
    buf0->m.userptr = (unsigned long)mem0[i];
    buf0->length  = memLength0[i];
     cout << "Mi buffer" << endl;
#else
    buf0->memory = V4L2_MEMORY_MMAP;
     cout << "Tu buffer" << endl;
#endif
    //cout << "ioctl(fd0, VIDIOC_QBUF, buf0): "<< ioctl(fd0, VIDIOC_QBUF, buf0)<< endl;
    //cout << "errno open buffer: " << errno << endl;

    VERIFY(ioctl(fd0, VIDIOC_QBUF, buf0) != -1);
    cout << "buf0->bytesused : " << buf0->bytesused << endl;
  }
}

void NaoCamera::initQueueAllBuffers1()
{
  // queue the buffers
  for(int i = 0; i < frameBufferCount; ++i)
  {
    buf1->index = i;
    buf1->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
    buf1->memory = V4L2_MEMORY_USERPTR;
    buf1->m.userptr = (unsigned long)mem1[i];
    buf1->length  = memLength1[i];
    // cout << "Mi buffer" << endl;
#else
    buf1->memory = V4L2_MEMORY_MMAP;
    // cout << "Tu buffer" << endl;
#endif
    cout << "ioctl(fd1, VIDIOC_QBUF, buf1): "<< ioctl(fd1, VIDIOC_QBUF, buf1)<< endl;
    cout << "errno open buffer: " << errno << endl;

    //VERIFY(ioctl(fd1, VIDIOC_QBUF, buf1) != -1);

  }
}

void NaoCamera::initDefaultControlSettings()
{
  // make sure automatic stuff is off
#ifndef NO_NAO_EXTENSIONS
  // int flip = currentCamera == ImageInfo::upperCamera ? 1 : 0;
  //VERIFY(setControlSetting(V4L2_CID_HFLIP, flip));
  //VERIFY(setControlSetting(V4L2_CID_VFLIP, flip));


    cout << "\t \t AUTOEXPOSICION Y AUTO WHITE BALANCE = 0" << endl;
    cout << "\t setControlSetting0 EXPOSURE_AUTO: " << setControlSetting0(V4L2_CID_EXPOSURE_AUTO, 0) << endl;
    cout << "\t setControlSetting1 EXPOSURE_AUTO: " << setControlSetting1(V4L2_CID_EXPOSURE_AUTO, 0) << endl;
    cout << "\t setControlSetting0 AUTO_WHITE_BALANCE: " << setControlSetting0(V4L2_CID_AUTO_WHITE_BALANCE, 0) << endl;
    cout << "\t setControlSetting1 AUTO_WHITE_BALANCE: " << setControlSetting1(V4L2_CID_AUTO_WHITE_BALANCE, 0) << endl;
    cout << endl;

    cout << "\t \t AUTOEXPOSICION Y AUTO WHITE BALANCE = 1"<< endl;
    cout << "\t setControlSetting0 EXPOSURE_AUTO: " << setControlSetting0(V4L2_CID_EXPOSURE_AUTO, 1) << endl;
    cout << "\t setControlSetting1 EXPOSURE_AUTO: " << setControlSetting1(V4L2_CID_EXPOSURE_AUTO, 1) << endl;
    cout << "\t setControlSetting0 AUTO_WHITE_BALANCE: " << setControlSetting0(V4L2_CID_AUTO_WHITE_BALANCE, 1) << endl;
    cout << "\t setControlSetting1 AUTO_WHITE_BALANCE: " << setControlSetting1(V4L2_CID_AUTO_WHITE_BALANCE, 1) << endl;
    cout << endl;

    cout << "\t \t CONFIRMACION DE AUTOEXPOSICION Y AUTO WHITE BALANCE"<< endl;
    cout << "\t EXPOSURE_AUTO inicial 0 : " << getControlSetting0(V4L2_CID_EXPOSURE_AUTO)<< endl;
    cout << "\t EXPOSURE_AUTO inicial 1 : " << getControlSetting1(V4L2_CID_EXPOSURE_AUTO)<< endl;
    cout << "\t AUTO_WHITE_BALANCE inicial 0 : " << getControlSetting0(V4L2_CID_AUTO_WHITE_BALANCE)<< endl;
    cout << "\t AUTO_WHITE_BALANCE inicial 1 : " << getControlSetting1(V4L2_CID_AUTO_WHITE_BALANCE)<< endl;
    cout << endl;


    cout << "\t NO_NAO_EXTENSIONS NO Definido" <<endl;

    //-------- Código original de BHuman --------//
    VERIFY(setControlSetting0(V4L2_CID_HFLIP, 0));
    VERIFY(setControlSetting0(V4L2_CID_VFLIP, 1));

    VERIFY(setControlSetting1(V4L2_CID_HFLIP, 0));
    VERIFY(setControlSetting1(V4L2_CID_VFLIP, 0));

    //std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getInitSettings();
    //VERIFY(setControlSettings0(v4l2settings));
    //VERIFY(setControlSettings1(v4l2settings));

    //-------------------------------------------//


  cout << "\t \t SETEO COMPLETO DE PARAMETROS" << endl;

  /** SETEO DE PARAMETROS DE DISPOSITIVO 0 */
  // Seteo de Brillo, Contraste, Saturación, Sharpness y Compensación de Luz de Fondo (=0). Hue no funciona
  cout << "\t setControlSetting0 BRIGHTNESS: "              << setControlSetting0(V4L2_CID_BRIGHTNESS, 128)             << endl;
  cout << "\t setControlSetting0 CONTRAST: "                << setControlSetting0(V4L2_CID_CONTRAST, 80)                 << endl;
  cout << "\t setControlSetting0 SATURATION: "              << setControlSetting0(V4L2_CID_SATURATION, 255)             << endl;
  //cout << "\t setControlSetting0 HUE: "                     << setControlSetting0(V4L2_CID_HUE, 0)                      << endl;
  cout << "\t setControlSetting0 SHARPNESS: "               << setControlSetting0(V4L2_CID_SHARPNESS, 2)                << endl;
  cout << "\t setControlSetting0 BACKLIGHT_COMPENSATION: "  << setControlSetting0(V4L2_CID_BACKLIGHT_COMPENSATION, 0)   << endl;

  // Se apaga la Exposición automática.
  // Se setea la Exposición Manual y la Ganancia
  cout << "\t setControlSetting0 EXPOSURE_AUTO OFF: "       << setControlSetting0(V4L2_CID_EXPOSURE_AUTO, 0)            << endl;
  cout << "\t setControlSetting0 EXPOSURE: "                << setControlSetting0(V4L2_CID_EXPOSURE, 80)                 << endl;
  cout << "\t setControlSetting0 GAIN: "                    << setControlSetting0(V4L2_CID_GAIN, 81)                   << endl;

  // Auto White Balance Off
  // White Balance Manual no funciona (bug del driver)
  cout << "\t setControlSetting0 AUTO_WHITE_BALANCE OFF: "  << setControlSetting0(V4L2_CID_AUTO_WHITE_BALANCE, 0)       << endl;
  //cout << "\t setControlSetting0 WHITE_BALANCE: "           << setControlSetting0(V4L2_CID_DO_WHITE_BALANCE, 0)         << endl;



  cout << endl;
  /** SETEO DE PARAMETROS DE DISPOSITIVO 1 */
  // Seteo de Brillo, Contraste, Saturación, Sharpness y Compensación de Luz de Fondo (=0). Hue no funciona
  cout << "\t setControlSetting1 BRIGHTNESS: "              << setControlSetting1(V4L2_CID_BRIGHTNESS, 128)             << endl;
  cout << "\t setControlSetting1 CONTRAST: "                << setControlSetting1(V4L2_CID_CONTRAST, 80)                << endl;
  cout << "\t setControlSetting1 SATURATION: "              << setControlSetting1(V4L2_CID_SATURATION, 255)             << endl;
  //cout << "\t setControlSetting1 HUE: "                     << setControlSetting1(V4L2_CID_HUE, 0)                      << endl;
  cout << "\t setControlSetting1 SHARPNESS: "               << setControlSetting1(V4L2_CID_SHARPNESS, 2)                << endl;
  cout << "\t setControlSetting1 BACKLIGHT_COMPENSATION: "  << setControlSetting1(V4L2_CID_BACKLIGHT_COMPENSATION, 0)   << endl;

  // Se apaga la Exposición automática.
  // Se setea la Exposición Manual y la Ganancia
  cout << "\t setControlSetting1 EXPOSURE_AUTO OFF: "       << setControlSetting1(V4L2_CID_EXPOSURE_AUTO, 0)            << endl;
  cout << "\t setControlSetting1 EXPOSURE: "                << setControlSetting1(V4L2_CID_EXPOSURE, 80)                 << endl;
  cout << "\t setControlSetting1 GAIN: "                    << setControlSetting1(V4L2_CID_GAIN, 81)                   << endl;

  // Auto White Balance Off
  // White Balance Manual no funciona (bug del driver)
  cout << "\t setControlSetting1 AUTO_WHITE_BALANCE OFF: "  << setControlSetting1(V4L2_CID_AUTO_WHITE_BALANCE, 0)       << endl;
  //cout << "\t setControlSetting1 WHITE_BALANCE: "           << setControlSetting1(V4L2_CID_DO_WHITE_BALANCE, 0)         << endl;


  cout << endl;
  cout << endl;

  SystemCall::sleep(300);


  cout << "\t \t OBTENCION DE PARAMETROS 2" << endl;
  cout << "\t getControlSetting0 BRIGHTNESS: "              << getControlSetting0(V4L2_CID_BRIGHTNESS)                  << endl;
  cout << "\t getControlSetting0 CONTRAST: "                << getControlSetting0(V4L2_CID_CONTRAST)                    << endl;
  cout << "\t getControlSetting0 SATURATION: "              << getControlSetting0(V4L2_CID_SATURATION)                  << endl;
  //cout << "\t getControlSetting0 HUE: "                     << getControlSetting0(V4L2_CID_HUE)                         << endl;
  cout << "\t getControlSetting0 SHARPNESS: "               << getControlSetting0(V4L2_CID_SHARPNESS)                   << endl;
  cout << "\t getControlSetting0 BACKLIGHT_COMPENSATION: "  << getControlSetting0(V4L2_CID_BACKLIGHT_COMPENSATION)      << endl;

  cout << "\t getControlSetting0 EXPOSURE_AUTO OFF: "       << getControlSetting0(V4L2_CID_EXPOSURE_AUTO)               << endl;
  cout << "\t getControlSetting0 EXPOSURE: "                << getControlSetting0(V4L2_CID_EXPOSURE)                    << endl;
  cout << "\t getControlSetting0 GAIN: "                    << getControlSetting0(V4L2_CID_GAIN)                        << endl;

  cout << "\t getControlSetting0 AUTO_WHITE_BALANCE OFF: "  << getControlSetting0(V4L2_CID_AUTO_WHITE_BALANCE)          << endl;
  //cout << "\t getControlSetting0 WHITE_BALANCE: "           << getControlSetting0(V4L2_CID_DO_WHITE_BALANCE)            << endl;


  cout << endl;

  cout << "\t getControlSetting1 BRIGHTNESS: "              << getControlSetting1(V4L2_CID_BRIGHTNESS)                  << endl;
  cout << "\t getControlSetting1 CONTRAST: "                << getControlSetting1(V4L2_CID_CONTRAST)                    << endl;
  cout << "\t getControlSetting1 SATURATION: "              << getControlSetting1(V4L2_CID_SATURATION)                  << endl;
  //cout << "\t getControlSetting1 HUE: "                     << getControlSetting1(V4L2_CID_HUE)                         << endl;
  cout << "\t getControlSetting1 SHARPNESS: "               << getControlSetting1(V4L2_CID_SHARPNESS)                   << endl;
  cout << "\t getControlSetting1 BACKLIGHT_COMPENSATION: "  << getControlSetting1(V4L2_CID_BACKLIGHT_COMPENSATION)      << endl;

  cout << "\t getControlSetting1 EXPOSURE_AUTO OFF: "       << getControlSetting1(V4L2_CID_EXPOSURE_AUTO)               << endl;
  cout << "\t getControlSetting1 EXPOSURE: "                << getControlSetting1(V4L2_CID_EXPOSURE)                    << endl;
  cout << "\t getControlSetting1 GAIN: "                    << getControlSetting1(V4L2_CID_GAIN)                        << endl;

  cout << "\t getControlSetting1 AUTO_WHITE_BALANCE OFF: "  << getControlSetting1(V4L2_CID_AUTO_WHITE_BALANCE)          << endl;
  //cout << "\t getControlSetting1 WHITE_BALANCE: "           << getControlSetting1(V4L2_CID_DO_WHITE_BALANCE)            << endl;

  cout << endl;


  SystemCall::sleep(500);

#else
  //  cout << "\t NO_NAO_EXTENSIONS definido"<<endl;
  setControlSetting0(V4L2_CID_AUTOEXPOSURE , 0);
  setControlSetting0(V4L2_CID_AUTO_WHITE_BALANCE, 0);
  setControlSetting0(V4L2_CID_AUTOGAIN, 0);
  setControlSetting0(V4L2_CID_HUE_AUTO, 0);
  setControlSetting0(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  setControlSetting0(V4L2_CID_HFLIP, 1);
  setControlSetting0(V4L2_CID_VFLIP, 1);

  setControlSetting1(V4L2_CID_AUTOEXPOSURE , 0);
  setControlSetting1(V4L2_CID_AUTO_WHITE_BALANCE, 0);
  setControlSetting1(V4L2_CID_AUTOGAIN, 0);
  setControlSetting1(V4L2_CID_HUE_AUTO, 0);
  setControlSetting1(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  setControlSetting1(V4L2_CID_HFLIP, 0);
  setControlSetting1(V4L2_CID_VFLIP, 0);
#endif
}

void NaoCamera::initResetCrop()
{
#ifndef NO_NAO_EXTENSIONS
  struct v4l2_cropcap cropcap;
  memset(&cropcap, 0, sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  cout << "ioctlaaaa(fd0, VIDIOC_CROPCAP, &cropcap): " << ioctl(fd0, VIDIOC_CROPCAP, &cropcap) << endl;
  cout << "ERRNO fd0: " << errno << endl;
  cout << "ioctl(fd1, VIDIOC_CROPCAP, &cropcap): " << ioctl(fd1, VIDIOC_CROPCAP, &cropcap) << endl;
  cout << "ERRNO fd1: " << errno << endl;

  //VERIFY(ioctl(fd0, VIDIOC_CROPCAP, &cropcap) != -1);
  //VERIFY(ioctl(fd1, VIDIOC_CROPCAP, &cropcap) != -1);


  struct v4l2_crop crop;
  memset(&crop, 0, sizeof(crop));
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  crop.c = cropcap.defrect;
  /* errno will be set to EINVAL if cropping is unsupported,
   * which would be fine, too */
  VERIFY(ioctl(fd0, VIDIOC_S_CROP, &crop) != -1 || errno == EINVAL);
  VERIFY(ioctl(fd1, VIDIOC_S_CROP, &crop) != -1 || errno == EINVAL);
#endif
}

void NaoCamera::startCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd0, VIDIOC_STREAMON, &type) != -1);
  VERIFY(ioctl(fd1, VIDIOC_STREAMON, &type) != -1);

}

void NaoCamera::assertCameraSettings0()
{
  bool allFine = true;
  // check frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(fps));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd0, VIDIOC_G_PARM, &fps));
  if(fps.parm.capture.timeperframe.numerator != 1)
  {
    //OUTPUT(idText, text, "fps.parm.capture.timeperframe.numerator is wrong.");
    allFine = false;
  }
  if(fps.parm.capture.timeperframe.denominator != 30)
  {
    //OUTPUT(idText, text, "fps.parm.capture.timeperframe.denominator is wrong.");
    allFine = false;
  }

  // check camera settings
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  std::list<CameraSettings::V4L2Setting>::const_iterator it = v4l2settings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = v4l2settings.end();
  for(; it != end; it++)
  {
    int value = getControlSetting0((*it).command);
    if(value != (*it).value)
    {
       //OUTPUT(idText, text, "Value for command " << (*it).command << " is " << value << " but should be " << (*it).value << ".");
      allFine = false;
    }
  }

  if(allFine)
  {
     //OUTPUT(idText, text, "Camera settings match settings stored in hardware/driver.");
  }
}

void NaoCamera::assertCameraSettings1()
{
  bool allFine = true;
  // check frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(fps));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd1, VIDIOC_G_PARM, &fps));
  if(fps.parm.capture.timeperframe.numerator != 1)
  {
     //OUTPUT(idText, text, "fps.parm.capture.timeperframe.numerator is wrong.");
    allFine = false;
  }
  if(fps.parm.capture.timeperframe.denominator != 30)
  {
     //OUTPUT(idText, text, "fps.parm.capture.timeperframe.denominator is wrong.");
    allFine = false;
  }

  // check camera settings
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  std::list<CameraSettings::V4L2Setting>::const_iterator it = v4l2settings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = v4l2settings.end();
  for(; it != end; it++)
  {
    int value = getControlSetting1((*it).command);
    if(value != (*it).value)
    {
       //OUTPUT(idText, text, "Value for command " << (*it).command << " is " << value << " but should be " << (*it).value << ".");
      allFine = false;
    }
  }

  if(allFine)
  {
     //OUTPUT(idText, text, "Camera settings match settings stored in hardware/driver.");
  }
}

void NaoCamera::writeCameraSettings()
{
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  VERIFY(setControlSettings0(v4l2settings));
  VERIFY(setControlSettings1(v4l2settings));
}
/*
int NaoCamera::openI2CAdapter()
{
#ifndef NO_NAO_EXTENSIONS

  cout << "openI2CAdapter" << endl;

  int i2cfd = open("/dev/i2c-0", O_RDWR);

  cout << "i2cfd: " << i2cfd << endl;

  ASSERT(i2cfd != -1);
  VERIFY(ioctl(i2cfd, 0x703, 8) == 0);
  return i2cfd;
#else
  return -1;
#endif
}

void NaoCamera::closeI2CAdapter(int filedes)
{
#ifndef NO_NAO_EXTENSIONS
  close(filedes);
#endif
}


bool NaoCamera::verifyNaoVersion()
{
  int i2cfd = openI2CAdapter();
  bool versionok = i2c_smbus_read_byte_data(i2cfd, 170) >= 2;
  closeI2CAdapter(i2cfd);
  return versionok;
}
*/
