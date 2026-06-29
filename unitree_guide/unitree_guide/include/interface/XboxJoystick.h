/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef XBOXJOYSTICK_H
#define XBOXJOYSTICK_H

#include <pthread.h>
#include <iostream>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"
#include "message/xbox_joystick.h"
#include <linux/input.h>
#include <linux/joystick.h>

class XboxJoystick : public CmdPanel {
public:
    XboxJoystick();
    ~XboxJoystick();

private:
    static void* runJoystick(void *arg);
    void* run(void *arg);
    void receiveHandle();
    int xbox_open(const char *file_name);
    int xbox_map_read(int xbox_fd, xbox_map_t *map);
    void xbox_close(int xbox_fd);

    pthread_t _tid;
    int JoystickFd;
    bool _running;
    xbox_map_t joystickData;

};


#endif  // XBOXJOYSTICK_H
