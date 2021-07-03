#ifndef CommandHandling_hpp
#define CommandHandling_hpp

/* External libraries */
#include <Arduino.h>

/* Definitions */
#define MAXIMUM_ALLOWED_COMMANDS (100)

enum CommandSpecifiers
{
    /* Needed due to 'toFloat' return 0 upon failure' */
    INVALID_SPECIFIER = 0,
    MOTOR_CONTROLLER,
    ACCELEROMETER,
    COMMAND_HANDLING,
    NUM_COMMAND_SPECIFIERS
};

struct AllowedCommands
{
    String commands[MAXIMUM_ALLOWED_COMMANDS];
    int numCommands;
};

/* Function declarations */
String parseCommandLine();

#endif
