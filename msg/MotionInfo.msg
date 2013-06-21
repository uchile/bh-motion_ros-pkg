#class MotionInfo : public MotionRequest

MotionRequest motionRequest
bool isMotionStable   #/**< If true, the motion is stable, leading to a valid torso / camera matrix. */
Pose2D upcomingOdometryOffset   # /**< The remaining odometry offset for the currently executed motion. */
bool upcomingOdometryOffsetValid   # /**< Whether the \c upcomingOdometryOffset is precise enough to be used */

