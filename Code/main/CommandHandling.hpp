#ifndef CommandHandling_hpp
#define CommandHandling_hpp

/* External libraries */
#include <Arduino.h>

/* Internal libraries */

/* Definitions */
#define MAXIMUM_ALLOWED_COMMANDS (5)
#define MAXIMUM_COMMAND_SIZE (2)

typedef void (*CommandFunctionCallback)(const String commandParameters[], const int numParameters);

struct Command
{
    int numParameters;
    String commandParameters[MAXIMUM_COMMAND_SIZE];
    String commandName;
};

struct CommandFunction
{
    String commandName;
    CommandFunctionCallback callbackFunction;
};

enum CommandSpecifiers
{
    /* Needed due to 'toInt' returning 0 upon failure' */
    INVALID_SPECIFIER = 0,
    MOTOR_CONTROLLER,
    EKF_FILTER,
    ACCELEROMETER,
    COMMAND_HANDLING,
    NUM_COMMAND_SPECIFIERS
};

struct AllowedCommands
{
    CommandFunction commands[MAXIMUM_ALLOWED_COMMANDS];
    int numCommands;
};

/* Function declarations */
void setupCommandHandler(void);
void parseCommandLine(String input);

#endif
