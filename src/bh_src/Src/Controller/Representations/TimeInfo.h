/**
* @file Controller/Representations/TimeInfo.h
*
* Declaration of class TimeInfo
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#pragma once

#include <string>
#include <tr1/unordered_map>

#include "Tools/RingBuffer.h"

class InMessage;

/**
* @class TimeInfo
*
* A class to represent information about the timing of modules.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/
class TimeInfo
{
private:
  enum {ringBufferSize = 100};

public:
  class Entry
  {
  public:
    unsigned startTime,
             endTime;
    unsigned timestamp,
             counter;
    Entry() : startTime(0), endTime(0), timestamp(0), counter(0) {}
    Entry(unsigned s, unsigned e, unsigned t, unsigned c)
      : startTime(s), endTime(e), timestamp(t), counter(c) {}
  };

  class Info
  {
  public:
    RingBuffer<Entry, ringBufferSize> entries;
    unsigned lastReceived;
    Info() : lastReceived(0) {}
  };

  typedef std::tr1::unordered_map<std::string, Info> Infos;
  Infos infos;
  unsigned int timeStamp; /**< The time stamp of the last change. */

  /**
  * Default constructor.
  */
  TimeInfo();

  /**
  * The function handles a stop watch message.
  * @param message The message.
  * @return Was it a stop watch message?
  */
  bool handleMessage(InMessage& message);

  /**
  * The function empties the time info object.
  */
  void reset();

  /**
  * The function empties the time info of a certain stop watch.
  * @param info Information on the stop watch to reset.
  */
  void reset(Info& info);

  /**
  * The function returns statistics about a certain stop watch.
  * @param info Information on the stop watch to query.
  * @param minTime The shortest measurement is returned to this variable.
  * @param maxTime The longest measurement is returned to this variable.
  * @param avgTime The average measurement is returned to this variable.
  * @param minDelta The minimum duration between two measurements is returned to this variable.
  * @param maxDelta The maximum duration between two measurements is returned to this variable.
  * @param avgFreq The average frequency is returned to this variable.
  * @return There were enough values to calculate the data.
  */
  bool getStatistics(const Info& info, float& minTime, float& maxTime, float& avgTime, float& minDelta, float& maxDelta, float& avgFreq) const;
};
