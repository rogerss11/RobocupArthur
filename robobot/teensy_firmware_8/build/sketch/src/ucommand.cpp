#line 1 "/home/local/svn/robobot/teensy_firmware_8/src/ucommand.cpp"
/***************************************************************************
 *   Copyright (C) 2019-2024 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 * 
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

// adjust REV_MINOR to save new SVN revision number
#define REV_MINOR 5
// revision (REV_ID) moved to teensy_firmware.ino file
//#define REV_ID "$Id: ucommand.cpp 1146 2025-02-11 19:23:22Z jcan $"

#include <malloc.h>
#include <ADC.h>
#include "IntervalTimer.h"
#include "main.h"
#include "ueeconfig.h"
#include "ucommand.h"
// #include "ucontrol.h"
#include "uusb.h"
#include "urobot.h"

#include "uencoder.h"
#include "umotor.h"
#include "uad.h"
#include "ucurrent.h"
#include "ulinesensor.h"
#include "uirdist.h"
#include "ulog.h"
#include "uservo.h"
#include "uimu2.h"
// #include "usound.h"
#include "udisplay.h"
// #include "uusbhost.h"

UCommand command;


void UCommand::setup()
{
  addPublistItem("ver", "get version 'version SVN_rev.x date time'");
  usb.addSubscriptionService(this);
}
/**
 * Get SVN revision number */
uint16_t UCommand::getRevisionNumber()
{
  const char * p1 = getRevisionString();
  p1 = strstr(p1, ".ino");
  if (p1 == nullptr)
    return 888;
  else
    return strtol(&p1[4], NULL, 10) * 10 + REV_MINOR;
}

/**
 * Get SVN revision number */
const char * UCommand::getCompileDate()
{
  const char * p1 = getRevisionString();
  p1 = strstr(p1, ".ino");
  if (p1 != nullptr)
  {
    p1+=4; // skip the '.cpp'
    strtol(p1, (char**)&p1, 10); // skip the version number
    p1++; // next is the date
    strncpy(compileDate, p1, 20);
    compileDate[20] = '\0';
    return compileDate;
  }
  else
    return "no date";
}

////////////////////////////////////////////

void UCommand::sendStatusVersion()
{
  const int MRL = 500;
  char reply[MRL];
  snprintf(reply, MRL, "version %.1f %d %s\r\n", (float)getRevisionNumber() / 10.0, robot.robotHWversion, getCompileDate());
  usb.send(reply);
}


bool UCommand::decode(const char * buf)
{
  bool used = true;
  if (strncmp(buf, "help", 4) == 0)
  { // send on-line help options
    usb.sendAllHelp();
    // eeConfig has no subscriptions, so no virtual help
//     eeConfig.sendHelp();
  }
  else if (strncmp(buf, "leave", 5) == 0)
  { // host are leaving - stop subscriptions
    usb.usbIsUp = false;
    usb.stopAllSubscriptions();
  }
  else if (strncmp(buf, "reboot", 6) == 0)
  { // reboot now
    SRC_GPR5 = 0x0BAD00F1;
    SCB_AIRCR = 0x05FA0004;
    while (1) ;
  }
  else
    used = false;
  return used;
}

// parse a user command and execute it, or print an error message
//
void UCommand::parse_and_execute_command(char * buf)
{ // command may be preceded by 'robot' or 'teensy' or robot type
  if (strncmp(buf, "robot ", 6) == 0)
  {
    buf += 6; // move pointer 6 characters forward
    usb.send("# discarding the 'robot' part\n");
    while (*buf == ' ')
      buf++;
  }
  else if (strncmp(buf, "teensy ", 7) == 0)
  {
    buf += 7; // move pointer 7 characters forward
    while (*buf == ' ')
      buf++;
    usb.send("# discarding the 'teensy' part\n");
  }
  else if (strncmp(buf, "regbot ", 7) == 0)
  {
    buf += 7; // move pointer 7 characters forward
    while (*buf == ' ')
      buf++;
    usb.send("# discarding the 'regbot' part\n");
  }
  //
  bool used = usb.decodeAll(buf);
  //
  if (not used)
  {
    usb.sendInfoAsCommentWithTime("Unhandled message", buf);
  }
}

/////////////////////////////////////////////////

void UCommand::tick()
{
  // subscription service
}

void UCommand::sendData(int item)
{ // called from subscribeTick()
  if (item == 0)
    sendStatusVersion();
}

void UCommand::sendHelp()
{
  const int MRL = 320;
  char reply[MRL];
  snprintf(reply, MRL, "# Command information for %s %s ------- \r\n", robot.deviceName, robot.getRobotName());
  usb.send(reply);
  snprintf(reply, MRL, "# -- \thelp \tThis help text.\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tleave \tStop all subscriptions.\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \treboot \tReboot the Teensy processor.\r\n");
  usb.send(reply);
}
