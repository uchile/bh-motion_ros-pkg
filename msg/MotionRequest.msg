# This expresses a Motion request.

uint8 motion  # /**< The selected motion. ENUM(Motion: walk, bike, specialAction, stand);*/

SpecialActionRequest specialActionRequest # /**< The special action request, if it is the selected motion. */
WalkRequest walkRequest  # /**< The walk request, if it is the selected motion. */
BikeRequest bikeRequest  # /**< The kick request, if it is the selected motion. */

