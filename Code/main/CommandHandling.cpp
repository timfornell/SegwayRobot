/* Local libraries */
#include "CommandHandling.hpp"
#include "MotorController.hpp"

/* Static variables */
static AllowedCommands allowedCommands[NUM_COMMAND_SPECIFIERS - 1]; // First value is 'Invalid'

/* Static function declarations */
static String splitStringWithSeparator(String data, char separator, int index);

/* Static function definitions */
static String splitStringWithSeparator(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for(int i = 0; i <= maxIndex && found <= index; i++){
    if(data.charAt(i) == separator || i == maxIndex){
        found++;
        strIndex[0] = strIndex[1] + 1;
        strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/* Funcion definitions */
void setupCommandHandler(void)
{
    Serial.begin(9600);

    /* Setup allowed commands */
    allowedCommands[MOTOR_CONTROLLER].commands[0] = "K";
    allowedCommands[MOTOR_CONTROLLER].commands[1] = "Ti";
    allowedCommands[MOTOR_CONTROLLER].commands[2] = "Td";
    allowedCommands[MOTOR_CONTROLLER].numCommands = 3;

    allowedCommands[ACCELEROMETER].numCommands = 0;
    allowedCommands[COMMAND_HANDLING].numCommands = 0;
}

String parseCommandLine(void)
{
    String string = Serial.readStringUntil('\n');

    if (string.length() > 0)
    {   
        const float cmdSpecifier = string.substring(0).toFloat();

        if (cmdSpecifier == MOTOR_CONTROLLER)
        {
            
        }
        else if (cmdSpecifier == ACCELEROMETER)
        {
            // Nothing
        }
        else if (cmdSpecifier == COMMAND_HANDLING)
        {
            // Nothing
        }
    }   
}
