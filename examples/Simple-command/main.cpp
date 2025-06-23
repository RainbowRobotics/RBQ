#include <iostream>
#include <signal.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <memory>
#include <termios.h>
#include <thread>

#include "rbq/Api.h"

int __IS_WORKING;

RBQ_API* rbqApi;

struct termios old_tio, new_tio;
bool keyboard_initialized = false;

void init_keyboard()
{
    if (keyboard_initialized) return;
    
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;

    new_tio.c_lflag &= ~(ICANON | ECHO);
    
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    keyboard_initialized = true;
}

void cleanup_keyboard()
{
    if (keyboard_initialized) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        keyboard_initialized = false;
    }
}

char check_keyboard()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) {
        return c;
    }
    return 0;
}

void CatchSignals(int _signal)
{
    std::cout << "Received signal: " << _signal << std::endl;
    switch(_signal)
    {
    case SIGHUP:
    case SIGINT:
    case SIGTERM:
    case SIGKILL:
    case SIGSEGV:
        __IS_WORKING = false;
        std::_Exit(_signal);
        break;
    };
}

int main(int argc, char *argv[])
{
    std::cout << "Starting " << APP_NAME << "..." << std::endl;

    init_keyboard();

    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);
    signal(SIGINT, CatchSignals);
    signal(SIGHUP, CatchSignals);
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        std::cerr << "Failed to lock memory" << std::endl;
        return 1;
    }
    std::cout << APP_NAME << " initializing..." << std::endl;

    try {
        rbqApi = new RBQ_API(21);
        if (!rbqApi) {
            std::cerr << "Failed to create RBQ_API" << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Initialization error: " << e.what() << std::endl;
        return 1;
    }
    __IS_WORKING = true;

    // TODO: cout all the key bindings
    // e.g.

    std::cout << "Press q for exit" << std::endl;
    std::cout << "Press 1 for Sit" << std::endl;
    std::cout << "Press 2 for Stance" << std::endl;
    std::cout << "Press 3 for Walk" << std::endl;
    std::cout << "Press 4 for Stairs" << std::endl;
    std::cout << "Press 5 for Running" << std::endl;
    
    

    while(__IS_WORKING && rbqApi)
    {
        char key = check_keyboard();

        bool joyCommandUpdated = false;

        switch(key) {
            case 'q':
                __IS_WORKING = false;
                break;
            case '1':
                std::cout << "Sit command received." << std::endl;
                rbqApi->motion.ground();
                break;
            case '2':
                std::cout << "Stance command received." << std::endl;   
                rbqApi->motion.ready();
                break;
            case '3':
                std::cout << "Walk command received." << std::endl;
                rbqApi->motion.walk();
                break;
            case '4':
                std::cout << "Stairs command received." << std::endl;
                rbqApi->motion.stairs();
                break;
            case '5':
                std::cout << "Running command received." << std::endl;
                rbqApi->motion.run();
                break;
            case 'w':
                joyCommandUpdated = true;
                rbqApi->gamepad.setLeftJogX(1.0f);
                break;
            case 's':
                joyCommandUpdated = true;
                rbqApi->gamepad.setLeftJogX(-1.0f);
                break;
            case 'a':
                joyCommandUpdated = true;
                rbqApi->gamepad.setLeftJogY(-1.0f);
                break;
            case 'd':
                joyCommandUpdated = true;
                rbqApi->gamepad.setLeftJogY(1.0f);
                break;
        }

        if(joyCommandUpdated) {
           rbqApi->gamepad.sendCommand();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz

    }
    std::cout << APP_NAME << " is terminating normally" << std::endl;
    cleanup_keyboard();
    return 0;
}
