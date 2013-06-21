#/** Current state of the robot's body. */
uint8 state #  ENUM(State: undefined, upright, onGround, staggering, falling);

#/** The robot is falling / fell into this direction. */
uint8 direction  # ENUM(Direction: none, front, left, back, right );

#/** Did the robot fell sidewards before? */
uint8 sidewards #ENUM(Sidestate: noot, /* since "not" is already a keyword...*/, leftwards, rightwards, fallen /* robot did not get up since last sideward fall */);

float32 odometryRotationOffset
