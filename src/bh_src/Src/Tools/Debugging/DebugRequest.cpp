/**
* @file DebugRequest.cpp
* Implementation of class DebugRequest
*
*/

#include <cstring>
#include <cstdio>

#include "DebugRequest.h"
#include "Platform/BHAssert.h"

DebugRequest::DebugRequest()
{
  description = "empty";
  enable = true;
}

DebugRequest::DebugRequest(const std::string& description, bool enable, bool once)
  : description(description),
    enable(enable),
    once(once)
{
}

DebugRequestTable::DebugRequestTable()
{
  currentNumberOfDebugRequests = 0;
  poll = false;
  alreadyPolledDebugRequestCounter = 0;
  lastName = 0;
}

void DebugRequestTable::addRequest(const DebugRequest& debugRequest)
{
  lastName = 0;
  nameToIndex.clear();
  if(debugRequest.description == "poll")
  {
    poll = true;
    pollCounter = 0;
    alreadyPolledDebugRequestCounter = 0;
  }
  else if(debugRequest.description == "disableAll")
    removeAllRequests();
  else
  {
    for(int i = 0; i < currentNumberOfDebugRequests; i++)
    {
      if(debugRequest.description == debugRequests[i].description)
      {
        if(debugRequest.enable == false && debugRequest.once == false)
          debugRequests[i] = debugRequests[--currentNumberOfDebugRequests];
        else
          debugRequests[i] = debugRequest;
        return;
      }
    }
    if(debugRequest.enable == true || debugRequest.once == true)
    {
      ASSERT(currentNumberOfDebugRequests < maxNumberOfDebugRequests);
      debugRequests[currentNumberOfDebugRequests++] = debugRequest;
    }
  }
}

void DebugRequestTable::disable(const char* name)
{
  lastName = 0;
  nameToIndex.clear();
  for(int i = 0; i < currentNumberOfDebugRequests; i++)
    if(debugRequests[i].description == name)
    {
      debugRequests[i] = debugRequests[--currentNumberOfDebugRequests];
      return;
    }
}

bool DebugRequestTable::notYetPolled(const char* name)
{
  for(int i = 0; i < alreadyPolledDebugRequestCounter; ++i)
    if(strcmp(name, alreadyPolledDebugRequests[i]) == 0)
      return false;
  alreadyPolledDebugRequests[alreadyPolledDebugRequestCounter++] = name;
  return true;
}

In& operator>>(In& stream, DebugRequest& debugRequest)
{
  unsigned char temp;
  stream >> temp;
  debugRequest.enable = temp == 1;
  stream >> temp;
  debugRequest.once = temp == 1;
  stream >> debugRequest.description;
  return stream;
}

Out& operator<<(Out& stream, const DebugRequest& debugRequest)
{
  stream << debugRequest.enable;
  stream << debugRequest.once;
  stream << debugRequest.description;
  return stream;
}

void DebugRequestTable::print(const char* message)
{
  fprintf(stderr, "%s\n", message);
}
