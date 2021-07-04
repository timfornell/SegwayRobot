/* External libraries */
#include <DebugLog.h>

/* Local libraries */
#include "CommandHandling.hpp"
#include "MotorController.hpp"

/* Static variables */
static AllowedCommands allowedCommands[NUM_COMMAND_SPECIFIERS - 1]; // First value is 'Invalid'

/* Static function declarations */
static Command parseCommand(String commandString, char separator, int index);
static boolean runCommand(const int commandSpecifier, const Command *const command);
static boolean checkCommandSpecifier(const int commandSpecifier);

/* Static function definitions */
static Command parseCommand(String commandString, char separator)
{
    Command command;
    command.size = 0;
    int index = 0;
    boolean commandStartFound = false;

    for (int i = 0; i < commandString.length(); i++)
    {
        if (!commandStartFound)
        {
            commandStartFound = (commandString[i] == ':') ? true : false;
        }
        else
        {
            if (commandString[i] == separator)
            {
                index += 1;
                command.size += 1;
                continue;
            }
            else
            {
                command.size += (command.size == 0) ? 1 : 0;
                command.command[index] += commandString[i];
            }
        }
    }

    return command;
}

static boolean runCommand(const int commandSpecifier, const Command *const commandArray)
{
    boolean commandFound = false;
    const String commandName = commandArray->command[0];
    AllowedCommands *const commands = &allowedCommands[commandSpecifier];

    for(int i = 0; i < commands->numCommands; i++)
    {
        CommandFunction *const commandFunction = &commands->commands[i];
        if (commandName == commandFunction->commandName)
        {
            LOG_VERBOSE("Found command to run.");
            commandFound = true;
            commandFunction->callbackFunction(commandArray);
            break;
        }
    }

    return commandFound;
}

static boolean checkCommandSpecifier(const int commandSpecifier)
{
    boolean validSpecifier = false;

    // This value should always be equal to the number of options in the 'else if' statement below
    const int numCommandSpecifiers = 3;

    if (numCommandSpecifiers + 1 != NUM_COMMAND_SPECIFIERS)
    {
        LOG_ERROR("Number of command specifiers is not correct, commands will not work.");
    }
    else if (commandSpecifier == MOTOR_CONTROLLER ||
             commandSpecifier == ACCELEROMETER ||
             commandSpecifier == COMMAND_HANDLING)
    {
        LOG_VERBOSE("Specifier is valid.");
        validSpecifier = true;
    }

    return validSpecifier;
}

/* Funcion definitions */
void setupCommandHandler(const DebugLogLevel debugLogLevel)
{
    // Setup debug level
    LOG_SET_LEVEL(DebugLogLevel::VERBOSE);

    /* Setup allowed commands */
    allowedCommands[MOTOR_CONTROLLER].commands[0] = {"K", &setControllerParameter_K};
    allowedCommands[MOTOR_CONTROLLER].commands[1] = {"Ti", &setControllerParameter_Ti};
    allowedCommands[MOTOR_CONTROLLER].commands[2] = {"Td", &setControllerParameter_Td};
    allowedCommands[MOTOR_CONTROLLER].numCommands = 3;

    allowedCommands[ACCELEROMETER].numCommands = 0;
    allowedCommands[COMMAND_HANDLING].numCommands = 0;
}

String parseCommandLine(void)
{
    /*
     * This is a very basic and ugly way to parse commands from the serial interface.
     * Commands should be on the form '<specifier>:<command>,<optional parameters>'
     */

    String string = Serial.readStringUntil('\n');
    LOG_VERBOSE("Command received: ");
    LOG_VERBOSE(string);

    /* The shortest allowed command is two characters long, e.g. 'commandSpecifier:' */
    if (string.length() > 1)
    {
        const int commandSpecifier = string.substring(0).toInt();
        LOG_VERBOSE("Parse command...");
        const Command command = parseCommand(string.substring(1, string.length() - 1), ',');
        LOG_VERBOSE("Check command specifier...");
        const boolean validCommandSpecifier = checkCommandSpecifier(commandSpecifier);

        if (validCommandSpecifier && command.size > 0)
        {
            LOG_VERBOSE("Run command...");
            const boolean commandWasRun = runCommand(commandSpecifier, &command);
            if (!commandWasRun)
            {
                LOG_WARNING("Could not run command: ");
                LOG_WARNING(string);
            }

            LOG_VERBOSE("Command ran successfully.");
        }
        else
        {
            LOG_WARNING("Failed to parse command...");
            LOG_WARNING("Commands should be on the form '<specifier>:<command>,<optional parameters>'.");
        }
    }
}
