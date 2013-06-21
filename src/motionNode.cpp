/*
Autor: Leonardo Leottau. Octubre 23  de 2012
 */

#include "uch_src/UChProcess/process.h"     //TODO: HACER ESTA INCLUSION CON CMAKE

int main (int argc, char **argv)
{
  //ROS_INFO_STREAM("Antes robotProcess con ROS" );
  
  ros::init (argc, argv, "motionNode");

  process motionProcess;
  motionProcess.motionThread.join();

  //ROS_INFO_STREAM("Despues robotProcess con ROS" );

  return 0;
}
