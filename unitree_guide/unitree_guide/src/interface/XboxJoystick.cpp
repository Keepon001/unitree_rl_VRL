/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/XboxJoystick.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

XboxJoystick::XboxJoystick() : _running(true) {
    userCmd = UserCommand::NONE;
    userValue.setZero();
    memset(&joystickData, 0, sizeof(joystickData));
    JoystickFd = xbox_open("/dev/input/js0");
    if (JoystickFd < 0) {
        _running = false;
        return;
    }

    pthread_create(&_tid, NULL, runJoystick, (void*)this);
}

XboxJoystick::~XboxJoystick() {
    _running = false;
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    xbox_close(JoystickFd);
}

void* XboxJoystick::runJoystick(void *arg) {
    return ((XboxJoystick*)arg)->run(NULL);
}

void* XboxJoystick::run(void *arg) {
    while (_running) {
        int len = xbox_map_read(JoystickFd, &joystickData);
        if (len < 0) {
            usleep(10 * 1000);
            continue;
        }
        receiveHandle();
        usleep(1000);  // Delay to prevent high CPU usage
    }
    return NULL;
}

void XboxJoystick::receiveHandle() {
    if (((int)joystickData.lb == 1) && ((int)joystickData.b == 1)) {
        userCmd = UserCommand::L2_B;
    } else if (((int)joystickData.lb == 1) && ((int)joystickData.a == 1)) {
        userCmd = UserCommand::L2_A;
    } else if (((int)joystickData.lb == 1) && ((int)joystickData.x == 1)) {
        userCmd = UserCommand::L2_X;
    } else if (((int)joystickData.lb == 1) && ((int)joystickData.y == 1)) {
        userCmd = UserCommand::START;
        std::cout<<"START"<< std::endl;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if (((int)joystickData.lb == 1) && ((int)joystickData.y == 1)) {
        userCmd = UserCommand::L2_Y;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else if (((int)joystickData.rb == 1) && ((int)joystickData.x == 1)) {
        userCmd = UserCommand::L1_X;
    } else if (((int)joystickData.rb == 1) && ((int)joystickData.a == 1)) {
        userCmd = UserCommand::L1_A;
    } else if (((int)joystickData.rb == 1) && ((int)joystickData.y == 1)) {
        userCmd = UserCommand::L1_Y;
    } 

    userValue.L2 = killZeroOffset(joystickData.lt / 32767.0f, 0.08);
    userValue.lx = killZeroOffset(joystickData.lx / 32767.0f, 0.08);
    userValue.ly = -killZeroOffset(joystickData.ly / 32767.0f, 0.08);
    userValue.rx = killZeroOffset(joystickData.ry / 32767.0f, 0.08);
    userValue.ry = -killZeroOffset(joystickData.rt / 32767.0f, 0.08);
    
    // Print the joystick data for debugging
    // printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d select:%d start:%d lo:%d ro:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
    //        joystickData.time, joystickData.a, joystickData.b, joystickData.x, joystickData.y,
    //        joystickData.lb, joystickData.rb, joystickData.select, joystickData.start,
    //        joystickData.lo, joystickData.ro, joystickData.xx, joystickData.yy,
    //        joystickData.lx, joystickData.ly, joystickData.rx, joystickData.ry,
    //        joystickData.lt, joystickData.rt);
    // std::cout<<userValue.lx<< std::endl;
    // fflush(stdout);
}

int XboxJoystick::xbox_open(const char *file_name) {
    int xbox_fd = open(file_name, O_RDONLY);
    if (xbox_fd < 0) {
        perror("open");
        return -1;
    }
    return xbox_fd;
}

int XboxJoystick::xbox_map_read(int xbox_fd, xbox_map_t *map) {
    int len;
    struct js_event js;

    len = read(xbox_fd, &js, sizeof(struct js_event));
    if (len < 0) {
        perror("read");
        return -1;
    }

    map->time = js.time;
    if (js.type == JS_EVENT_BUTTON) {
        switch (js.number) {
            case XBOX_BUTTON_A: map->a = js.value; break;
            case XBOX_BUTTON_B: map->b = js.value; break;
            case XBOX_BUTTON_X: map->x = js.value; break;
            case XBOX_BUTTON_Y: map->y = js.value; break;
            case XBOX_BUTTON_LB: map->lb = js.value; break;
            case XBOX_BUTTON_RB: map->rb = js.value; break;
            case XBOX_BUTTON_SELECT: map->select = js.value; break;
            case XBOX_BUTTON_START: map->start = js.value; break;
            case XBOX_BUTTON_HOME: map->home = js.value; break;
            // case XBOX_BUTTON_LO: map->lo = js.value; break;
            // case XBOX_BUTTON_RO: map->ro = js.value; break;
            default: break;
        }
    } else if (js.type == JS_EVENT_AXIS) {
        switch (js.number) {
            case XBOX_AXIS_LX: map->lx = js.value; break;
            case XBOX_AXIS_LY: map->ly = js.value; break;
            case XBOX_AXIS_RX: map->rx = js.value; break;
            case XBOX_AXIS_RY: map->ry = js.value; break;
            case XBOX_AXIS_LT: map->lt = js.value; break;
            case XBOX_AXIS_RT: map->rt = js.value; break;
            case XBOX_AXIS_XX: map->xx = js.value; break;
            case XBOX_AXIS_YY: map->yy = js.value; break;
            default: break;
        }
    }
    return len;
}

void XboxJoystick::xbox_close(int xbox_fd) {
    close(xbox_fd);
}
