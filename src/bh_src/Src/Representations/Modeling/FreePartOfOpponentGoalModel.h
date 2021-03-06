/**
 * @file Representations/Modeling/FreePartOfOpponentGoalModel.h
 * Declaration of a representation that represents the free part of the opponent goal
 * @author Carsten Koenemann
 */

#pragma once

#include "Tools/Math/Vector2.h"

class FreePartOfOpponentGoalModel : public Streamable
{
public:
  /** relative position on field of left end of largest free part of opponent goal */
  Vector2<> leftEnd;

  /** relative position on field of right end of largest free part of opponent goal */
  Vector2<> rightEnd;

  /** if there is currently a reasonable part of the opponent goal free */
  bool isFree;

  /** Draws the free part of goal to the field view */
  //void draw() const;

private:
  /**
   * Makes the object streamable
   * @param in The stream from which the object is read
   * @param out The stream to which the object is written
   */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(leftEnd);
    STREAM(rightEnd);
    STREAM(isFree);
    STREAM_REGISTER_FINISH();
  }
};
