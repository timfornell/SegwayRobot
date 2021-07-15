#ifndef CommandHandling_hpp
#define CommandHandling_hpp

/* External libraries */
#include <Arduino.h>

/* Definitions */
#define MAXIMUM_ALLOWED_COMMANDS (5)
#define MAXIMUM_COMMAND_SIZE (20)

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
    /* Needed due to 'toInt' return 0 upon failure' */
    INVALID_SPECIFIER = 0,
    MOTOR_CONTROLLER,
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
