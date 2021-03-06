/**
* @file Tools/Settings.cpp
* Implementation of a class that provides access to settings-specific configuration directories.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Rfer</a>
*/

#include <cstring>

#include "Streams/InStreams.h"
#include "Settings.h"
#include "../Representations/Infrastructure/RoboCupGameControlData.h"
#ifdef TARGET_SIM
#include "../Controller/RoboCupCtrl.h"
#endif
#ifdef TARGET_ROBOT
#include "../Platform/linux/NaoBody.h"
#endif
#include "../Platform/BHAssert.h"
#include "../Platform/SystemCall.h"

Settings Settings::settings(true);
bool Settings::loaded = false;

Settings::Settings(bool master) :
  teamNumber(0),
  teamColor(TEAM_BLUE),
  playerNumber(0),
  location("Default"),
  teamPort(0),
  model(nao),
  recover(false)
{
  ASSERT(master);
}

Settings::Settings()
{
  if(!loaded)
  {
    VERIFY(settings.load());
    loaded = true;
  }
  *this = settings;

#ifdef TARGET_SIM
  if(SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    int index = RoboCupCtrl::controller->getRobotName()[5] - '1';
    teamNumber = index < 4 ? 1 : 2;
    teamPort = 10000 + teamNumber;
    teamColor = index < 4 ? TEAM_BLUE : TEAM_RED;
    playerNumber = (index & 3) + 1;
    model = nao;
  }
  else if(SystemCall::getMode() == SystemCall::physicalRobot)
  {
    const char* name = RoboCupCtrl::controller->getRobotFullName();
    int index = name[strlen(name) - 1] - '1' + 1;
    if(index >= 1 && index <= 4)
      playerNumber = index;
  }
#endif
}

bool Settings::load()
{
#ifdef TARGET_ROBOT
  robot = NaoBody().getName();
#else
  robot = "Nao";
#endif

  bool loadedSettingsFromFile = false;
#ifdef TARGET_ROBOT
  // load settings from team.cfg
  try
  {
    ConfigMap teams;
    if(teams.read("teams.cfg", false, &ConfigMap::printOnErr) >= 0)
    {
      teams.setFlags(teams.getFlags() | ConfigMap::READONLY);

      std::vector<std::string> keys = teams.getKeys();
      for(std::vector<std::string>::const_iterator i = keys.begin(), end = keys.end(); i != end; ++i)
      {
        const ConfigValue& teamValue = teams[*i];
        if(teamValue.getType() != ConfigValue::MAP)
          continue;
        const ConfigMap& team = teamValue;
        if(!team.hasKey("players"))
          continue;
        const ConfigValue& playersValue = team["players"];
        if(playersValue.getType() != ConfigValue::LIST)
          continue;

        const ListConfigValue& players = playersValue;
        for(int i = 0, len = players.length(); i < len; ++i)
        {
          const ConfigValue& playersValue = players[i];
          if(playersValue.getType() != ConfigValue::PLAIN)
            continue;
          if(((const PlainConfigValue&)playersValue).str() == robot)
          {
            playerNumber = i + 1;
            team["number"] >> teamNumber;
            teamPort = 10001 + teamNumber * 100;
            team["location"] >> location;
            std::string entryName;
            team["color"] >> entryName;
            if(entryName == "blue")
              teamColor = TEAM_BLUE;
            else if(entryName == "red")
              teamColor = TEAM_RED;
            else
              ASSERT(false);
            loadedSettingsFromFile = true;
            goto loadedTeamsCfg;
          }
        }
      }
loadedTeamsCfg:
      ;
    }
  }
  catch(std::invalid_argument& e)
  {
    TRACE("%s", e.what());
    return false;
  }
  catch(invalid_key& e)
  {
    TRACE("%s", e.what());
    return false;
  }
#endif

  // overwrite settings with values from settings.cfg
  try
  {
    ConfigMap cm;
    if(cm.read("settings.cfg", false, &ConfigMap::printOnErr) >= 0)
    {
      cm.setFlags(cm.getFlags() | ConfigMap::READONLY);
      std::string entryName;

      cm["model"] >> entryName;
      if(entryName == "nao")
        model = nao;
      else
        ASSERT(false);
      cm["teamNumber"] >> teamNumber;
      cm["teamPort"] >> teamPort;
      cm["teamColor"] >> entryName;
      if(entryName == "blue")
        teamColor = TEAM_BLUE;
      else if(entryName == "red")
        teamColor = TEAM_RED;
      else
        ASSERT(false);
      cm["playerNumber"] >> playerNumber;
      cm["location"] >> location;
      loadedSettingsFromFile = true;
    }
  }
  catch(std::invalid_argument& e)
  {
    TRACE("%s", e.what());
    return false;
  }
  catch(invalid_key& e)
  {
    TRACE("%s", e.what());
    return false;
  }
  if(!loadedSettingsFromFile)
  {
    TRACE("Could not load settings for robot \"%s\" from teams.cfg or settings.cfg", robot.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", teamColor == TEAM_BLUE ? "blue" : "red");
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
#endif
  return true;
}

std::string Settings::expandHostFilename(const std::string& file) const
{
  return std::string("Hosts/") + SystemCall::getHostName() + "/" + file;
}

std::string Settings::expandLocationFilename(const std::string& file) const
{
  return std::string("Locations/") + location + "/" + file;
}

std::string Settings::expandRobotFilename(const std::string& file) const
{
  return std::string("Robots/") + robot + "/" + file;
}

std::string Settings::expandHostLocationFilename(const std::string& file) const
{
  return std::string("Locations/") + location + "/Hosts/" + SystemCall::getHostName() + "/" + file;
}

std::string Settings::expandRobotLocationFilename(const std::string& file) const
{
  return std::string("Locations/") + location + "/Robots/" + robot + "/" + file;
}

std::string Settings::expandModelFilename(const std::string& file) const
{
  return std::string("Robots/") + "Nao" + "/" + file;
}
