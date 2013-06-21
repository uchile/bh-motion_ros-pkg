/*
Author: Leonardo Leottau. Octubre 25 de 2012
 */
#include "ros/ros.h"
#include "bh_motion/Cognition2Motion.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cognitionNode");
  ros::NodeHandle n;
  ros::Publisher cog2motionTopic_pub = n.advertise<bh_motion::Cognition2Motion>("cog2motionTopic", 1);
   
 ros::Rate loop_rate(0.2);

  bh_motion::Cognition2Motion cognition2Motion;

  cognition2Motion.motionRequest.walkRequest.mode=2;	
  cognition2Motion.motionRequest.walkRequest.target.rotation=0;
  cognition2Motion.motionRequest.walkRequest.target.translation.x=0;
  cognition2Motion.motionRequest.walkRequest.target.translation.y=0;  
  cognition2Motion.motionRequest.walkRequest.speed.rotation=1;
  cognition2Motion.motionRequest.walkRequest.speed.translation.x=1;
  cognition2Motion.motionRequest.walkRequest.speed.translation.y=1;  
  cognition2Motion.motionRequest.walkRequest.pedantic = false;	
  cognition2Motion.motionRequest.walkRequest.dribbling = false;	

//NUM(Motion: walk, bike, specialAction, stand)
// ENUM(SpecialActionID: playDead, standUpBackNao, standUpFrontNao, sitDown, sitDownKeeper, goUp, keeperJumpLeftSign);

int i=0;
  while (ros::ok())
  {
       
    if (i==0) 	{
        cognition2Motion.motionRequest.motion=0;   // walking
        cognition2Motion.motionRequest.walkRequest.target.rotation = 0;
        cognition2Motion.motionRequest.walkRequest.target.translation.x = 200;
	cognition2Motion.motionRequest.walkRequest.target.translation.y = 0;	


		}

    else if (i==1) {
        //cognition2Motion.motionRequest.motion=2;  // special action
        //cognition2Motion.motionRequest.specialActionRequest.specialAction=4;  //sitdown keeper

        cognition2Motion.motionRequest.motion=3;   // stand
        cognition2Motion.motionRequest.walkRequest.target.rotation = 0;
	cognition2Motion.motionRequest.walkRequest.target.translation.x = 0;
        cognition2Motion.motionRequest.walkRequest.target.translation.y = 0;

        cognition2Motion.headAngleRequest.pan = 1.0;
        cognition2Motion.headAngleRequest.tilt = 0.5;
        cognition2Motion.headAngleRequest.speed = 0.5;

		}

   else if (i==2) {
        cognition2Motion.motionRequest.motion=0;  // walking
        cognition2Motion.motionRequest.walkRequest.target.rotation = 120; //120
	cognition2Motion.motionRequest.walkRequest.target.translation.x = 0;
	cognition2Motion.motionRequest.walkRequest.target.translation.y = 0;		 
		}
   else {
        cognition2Motion.motionRequest.motion=2;  // special action
        cognition2Motion.motionRequest.specialActionRequest.specialAction=3;  //sitdown
	cognition2Motion.motionRequest.walkRequest.target.rotation = 0;
	cognition2Motion.motionRequest.walkRequest.target.translation.x = 0;
        cognition2Motion.motionRequest.walkRequest.target.translation.y = 0;

        cognition2Motion.headAngleRequest.pan = -1.0;
        cognition2Motion.headAngleRequest.tilt = -0.5;
        cognition2Motion.headAngleRequest.speed = 0.5;
	i=-1;
		}

	//ROS_INFO("%s", msg.data.c_str());
    ROS_INFO("Requesting: x= %f, y= %f, theta=%f", cognition2Motion.motionRequest.walkRequest.target.translation.x, cognition2Motion.motionRequest.walkRequest.target.translation.y, cognition2Motion.motionRequest.walkRequest.target.rotation);

// %Tag(PUBLISH)%
    cog2motionTopic_pub.publish(cognition2Motion);
    ros::spinOnce();

    loop_rate.sleep();
    i++;
}



  return 0;
}

