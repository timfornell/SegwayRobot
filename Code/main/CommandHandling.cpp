/* External libraries */

/* Local libraries */
#include "CommandHandling.hpp"
#include "MotorController.hpp"

/* Static variables */
static AllowedCommands allowedCommands[NUM_COMMAND_SPECIFIERS - 1]; // First value is 'Invalid'

/* Static function declarations */
static boolean getCommandNameAndParameters(const String commandParameters, Command &command);
static boolean runCommand(const int commandSpecifier, const String commandParameters);
static boolean checkCommandSpecifier(const int commandSpecifier);

/* Static function definitions */
static boolean getCommandNameAndParameters(const String commandParameters, Command &command)
{
    boolean successfullyParsedCommand = false;

    /* The command name starts after the ':' and ends at the first ',' */
    int colonIndex = commandParameters.indexOf(':');
    int commaIndex = commandParameters.indexOf(',');
    command.numParameters = 0;
    
    if (colonIndex > -1)
    {
        Serial.print("Colon found. Command name: ");
        command.commandName = commaIndex > -1 ? commandParameters.substring(colonIndex + 1, commaIndex) : 
                                                commandParameters.substring(colonIndex + 1, commandParameters.length());
        Serial.println(command.commandName);
        
        const String parameters = commandParameters.substring(commaIndex + 1);
        Serial.print("Parameters: ");
        Serial.println(parameters);
        
        // Reset commaIndex
        commaIndex = 0;
        while (commaIndex > -1)
        {
            const String substring = parameters.substring(commaIndex);
            Serial.print("Searching string: ");
            Serial.println(substring);

            const int nextComma = substring.indexOf(',');
            Serial.print("Previous comma: ");
            Serial.println(commaIndex);
            Serial.print("Next comma: ");
            Serial.println(nextComma);

            if (nextComma > 0)
            {
                const String parameter = substring.substring(0, nextComma );
                Serial.print("Found parameter: ");
                Serial.println(parameter);

                command.commandParameters[command.numParameters] = parameter;
                command.numParameters += 1;
                commaIndex += nextComma + 1;
            }
            else
            {
                // The string won't end with a comma
                if (substring.length() > 0)
                {
                    Serial.print("Adding last parameter: ");
                    Serial.println(substring);
                    command.commandParameters[command.numParameters] = substring;
                    command.numParameters += 1;
                }

                break;
            }
        }
        
        successfullyParsedCommand = true;
    }
    else
    {
        Serial.print("No colon found:");
        Serial.print(colonIndex);
    }

    return successfullyParsedCommand;
}

static boolean runCommand(const int commandSpecifier, const String commandParameters)
{
    AllowedCommands *const commands = &allowedCommands[commandSpecifier];
    boolean commandFound = false;
    Command command;

    boolean commandParsed = getCommandNameAndParameters(commandParameters, command);

    if (commandParsed)
    {
        for(int i = 0; i < commands->numCommands; i++)
        {
            CommandFunction *const commandFunction = &commands->commands[i];
            if (command.commandName == commandFunction->commandName)
            {
                Serial.println("Found command to run.");
                commandFound = true;
                commandFunction->callbackFunction(command.commandParameters, command.numParameters);
                break;
            }
        }
    }

    return commandFound;
}

static boolean checkCommandSpecifier(const int commandSpecifier)
{
    boolean validSpecifier = false;

    // This value should always be equal to the number of options in the 'else if' statement below
    const int numCommandSpecifiers = 3;

    if (numCommandSpecifiers + 1 != static_cast<int>(NUM_COMMAND_SPECIFIERS))
    {
        Serial.println("Number of command specifiers is not correct, commands will not work.");
    }
    else if (commandSpecifier == static_cast<int>(MOTOR_CONTROLLER) ||
             commandSpecifier == static_cast<int>(ACCELEROMETER) ||
             commandSpecifier == static_cast<int>(COMMAND_HANDLING))
    {
        Serial.println("Specifier is valid.");
        validSpecifier = true;
    }

    return validSpecifier;
}

/* Funcion definitions */
void setupCommandHandler(void)
{
    /* Setup allowed commands */
    allowedCommands[MOTOR_CONTROLLER].commands[0] = {"K", &setControllerParameter_K};
    allowedCommands[MOTOR_CONTROLLER].commands[1] = {"Ti", &setControllerParameter_Ti};
    allowedCommands[MOTOR_CONTROLLER].commands[2] = {"Td", &setControllerParameter_Td};
    allowedCommands[MOTOR_CONTROLLER].numCommands = 3;

    allowedCommands[ACCELEROMETER].numCommands = 0;
    allowedCommands[COMMAND_HANDLING].numCommands = 0;
    
    Serial.println("CommandHandler setup finished.");
}

void parseCommandLine(String input)
{
    /*
     * This is a very basic and ugly way to parse commands from the serial interface.
     * Commands should be on the form '<specifier>:<command>,<optional parameters>'
     */

    /* The shortest allowed command is two characters long, e.g. 'commandSpecifier:' */
    if (input.length() > 1)
    {
        const int commandSpecifier = input.substring(0).toInt();
        Serial.print("Command specifier: ");
        Serial.println(commandSpecifier);
 
        Serial.println("Check command specifier...");
        const boolean validCommandSpecifier = checkCommandSpecifier(commandSpecifier);

        if (validCommandSpecifier)
        {
            Serial.println("Command specifier was valid.");
            String substring = input.substring(1, input.length());
            Serial.print("Call runCommand with: ");
            Serial.println(substring);
            const boolean successfull = runCommand(commandSpecifier, substring);
        }
        else
        {
            Serial.println("Command specifier was not valid.");
        }
    }

    Serial.println("Finished parsing command line.");
}
