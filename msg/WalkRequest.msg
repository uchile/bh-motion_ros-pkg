# This expresses a walk request.


#(ENUM(Mode: speedMode, /**< Interpret \c speed as absolute walking speed and ignore \c target. */
#    percentageSpeedMode, /**< Interpret \c speed as percentage walking speed and ignore \c target. */
#    targetMode /**< Use \c target as walking target relative to the current position of the robot and interpret \c speed as percentage walking speed. */);)

uint32 mode  #/**< The walking mode. */
Pose2D speed #/**< Walking target, in mm and radians, relative to the robot. Use either a speed or a target. */
Pose2D target #/**< Walking target, in mm and radians, relative to the robot. Use either a speed or a target. */
bool pedantic #/**< Allows to disable the step size stabilization. set it when precision is indispensable. */
bool dribbling

uint32 kickType
Vector2 kickBallPosition
Vector2 kickTarget
