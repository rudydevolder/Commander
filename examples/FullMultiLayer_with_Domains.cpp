/*Commander example - multi object, multi layer command structure:
 * Demonstrates how to use many commander objects each with its own command array to create a layerd command structure
 * Top level commands can be invoked by typing them
 * Lower commands can be directly invoked from the top layer by typing the top command followed by the lower command
 * For example 'get help' calls the 'help' function in the command set called 'get'
 * Lower command structures can be entered by typing their command, and then the lower level commands can be directly invoked
 * An exit command will return control to the top level
 * For example, 'get' will transfer control to the 'get' command set. 'help' will then call the help function for the 'get' commands.
 */

/* This example is a more advanced example from the above:
   It demonstrates the flexibility to work with a structured table of variables to report and fill in, 
   There's a group of general sub-commands (Volt, Amp, Speed) that repeatedly come back in several master-commands:
   - The trick is: I used the same general sub-commands over & over in 5 different master-commands setting and reading 
   values depending on the mastercommand (domain) 
   - Also, some are Read-only variables because they contain statistics like min and max, but they can be reset in group. 


  More idea's for 2 motor controllers:
  ======================================

                    =============
                    === To Do ===
                    =============
  
  Extra menus:
  =============

  select
    all     // Motor 1 & 2  => read/write parameters to all motors
    m1      // Motor 1      =>  only read/write parameters to motor 1
    m2      // Motor 2
    * Remarque:     show selected motor in command prompt

  report
    off
    on
    all
    stat
    oob     // only report when out of bounds

  break
      1234  // set analog breaking, 0 = digital break
  

*/




 #include "Arduino.h"
 #include <Commander.h>
 //include "uTimerLib.h"


extern const commandList_t  masterCommands[];   //forward declare the master command list
extern const uint16_t       numOfMasterCmds;    //A forward declaration so the compiler knows we are going to initialise this variable properly later
extern const commandList_t  Gen_Sub_Cmds[]; 	//forward declare the set command list
extern const uint16_t       numOfGen_Sub_Cmds;	//A forward declaration so the compiler knows we are going to initialise this variable properly later
extern const commandList_t  reportCommands[];
extern const uint16_t       numOfReportCmds;

portSettings_t savedSettings;                   //Create a portSettings_t variable to store Commander settings in when we switch to a different command object.

//Declare all the commander objects
Commander masterCmd;
Commander Gen_Param_Cmd;
Commander reportCmd;


//Create a pointer to a commander object, and point it to the master object
//We will change this later to point to a sub command object when we transfer control.
Commander* activeCommander = &masterCmd;

enum TypesOfDomain {Set, Read, SetMin, Min, SetMax, Max};
enum TypesOfVar   {Volt, Amp, Rpm};
//enum TypesOfCommands

//Variables we can set or get
bool  reporting     = false;
bool  Past1second   = false;
int   currentDOM    = 0;

const int MaxDomains = 6;       //{Set   , Read  , SetMin, Min   , SetMax, Max   };
int   SpeedVar[MaxDomains]      = {0     , 1     , 2     , 3     , 4     , 5     }; //Just an example of some default values to start with
float VoltageVar[MaxDomains]    = {10.99 , 4.5   , 5.5   , 6.5   , 12.56 , 12.88 }; // 
float AmperageVar[MaxDomains]   = {0.12  , 7.9   , 8.0   , 9.9   , 10.12 , 12.34 }; //
const char DomainsDescriptor[MaxDomains][12] = {"Set        ",
                                              "Actual     ", // = "read"
                                              "Set MInimum", // = Limit
                                              "Act.MInimum", // = Actual minimum
                                              "Set MAximum", // = Limit
                                              "Act.Maximum"};// = Actual maximum
const int MaxUnit = 3;
const char Units[MaxUnit][10] = {"Volt", "Amps", " Rpm"};

unsigned long millis_idle       = 0;
unsigned long reporting_period  = 1000;
unsigned long idle_period       = 10000; // At start

//==============================================================================================================================================================================================
// MASTER-commands:     =============================================================
bool reportHandler(Commander &Cmdr);
bool setHandler(Commander &Cmdr);   //Set parameters
bool readHandler(Commander &Cmdr);  //Read current parameters
bool setminHandler(Commander &Cmdr);//To set a minimum limit
bool setmaxHandler(Commander &Cmdr);//To set a maximum limit
bool minHandler(Commander &Cmdr);
bool maxHandler(Commander &Cmdr);
// General commands:    =============================================================
bool exit(Commander &Cmdr);         //Go back to main-menu
// report-commands:     =============================================================
bool reportON(Commander &Cmdr);
bool reportOFF(Commander &Cmdr);
// general-commands:    =============================================================
bool VoltVariable(Commander &Cmdr);
bool AmpVariable(Commander &Cmdr);
bool Speed(Commander &Cmdr);
bool DomReset(Commander &Cmdr);
//==============================================================================================================================================================================================
//All commands for 'master':
const commandList_t masterCommands[] = {
    {"set",         setHandler,          "'s'   => set    [speed/volt/amp]"},   {"s",   setHandler, "-"},
    {"act",         readHandler,         "'a'   => actual [speed/volt/amp]"},   {"a",   readHandler, "-"},
    {"setmin",      setminHandler,       "'smi' => set min[speed/volt/amp]"},   {"smi", setminHandler, "-"},
    {"min",         minHandler,          "'ami' => min    [speed/volt/amp]"},   {"ami", minHandler, "-"},
    {"setmax",      setmaxHandler,       "'sma' => setmax [speed/volt/amp]"},   {"sma", setmaxHandler, "-"},
    {"max",         maxHandler,          "'ama' => max    [speed/volt/amp]"},   {"ama", maxHandler, "-"},
    {"report",      reportHandler,       "'r'   => report [#seconds]"},         {"r",   reportHandler, "-"},
};
const uint16_t numOfMasterCmds = sizeof(masterCommands); //initialise the numOfMasterCmds variable after creating the masterCommands[] array - numOfMasterCmds Set indicates the length of the array
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool reportHandler(Commander &Cmdr){
    Cmdr.print("Report ");        
    Cmdr.println("");
    bool retVal = 0;
    if(Cmdr.hasPayload()){ 
      int delay = 1;
      if (!Cmdr.getInt(delay)) {
          retVal  = reportCmd.feed(Cmdr);
      } else {
          Serial.print("Delay: "); Serial.print(delay); Serial.println(" seconds");
          reporting_period  = delay * 1000;
          reporting = true;
      }

    }else{
      savedSettings = reportCmd.portSettings();                 //backup the port setting from the commander we are about to transfer control to.
      activeCommander = &reportCmd;                             //Set the Commander pointer to the sub commander 1 object
      activeCommander->transfer(Cmdr);                          //Transfer this Cmdr to the new active commander
    }
    return retVal;
}
//==============================================================================================================================================================================================
//All Sub commands:
bool Gen_Sub_Cmds_Handler(Commander &Cmdr){    
    bool    retVal = 0;
    switch (currentDOM) { 
        case Set:          Gen_Param_Cmd.commanderName = "Set";             break;
        case Read:         Gen_Param_Cmd.commanderName = "Actual";          break;
        case SetMin:       Gen_Param_Cmd.commanderName = "Set min limit";   break;
        case Min:          Gen_Param_Cmd.commanderName = "Actual min";      break;
        case SetMax:       Gen_Param_Cmd.commanderName = "Set max limit";   break;
        case Max:          Gen_Param_Cmd.commanderName = "Actual max";      break;
        default:           break;
    }
    if(Cmdr.hasPayload()){  //Cmdr.println("handing payload to get command object");
        Gen_Param_Cmd.print(Gen_Param_Cmd.commanderName);         Gen_Param_Cmd.print(" ");
        retVal  = Gen_Param_Cmd.feed(Cmdr);
    }else{                  //Cmdr.println("handing control to get command object");
        savedSettings = Gen_Param_Cmd.portSettings();           //backup the port setting from the commander we are about to transfer control to.
        activeCommander = &Gen_Param_Cmd;                       //Set the Commander pointer to the sub commander 1 object
        activeCommander->transfer(Cmdr);                        //Transfer this Cmdr to the new active commander
    }
    return retVal;
}
bool setHandler(Commander &Cmdr){
    currentDOM = Set;   
    return Gen_Sub_Cmds_Handler(Cmdr);
}
bool readHandler(Commander &Cmdr){
    currentDOM = Read;       
    return Gen_Sub_Cmds_Handler(Cmdr);
}
bool setminHandler(Commander &Cmdr){
    currentDOM = SetMin;        
    return Gen_Sub_Cmds_Handler(Cmdr);
}
bool minHandler(Commander &Cmdr){
    currentDOM = Min;        
    return Gen_Sub_Cmds_Handler(Cmdr);
}
bool setmaxHandler(Commander &Cmdr){
    currentDOM = SetMax;       
    return Gen_Sub_Cmds_Handler(Cmdr);
}
bool maxHandler(Commander &Cmdr){
    currentDOM = Max;       
    return Gen_Sub_Cmds_Handler(Cmdr);
}

bool exit(Commander &Cmdr){
    Cmdr.println("Back to 'Main menu'"); 
    activeCommander = &masterCmd;     //Set the Commander pointer to the sub commander 1 object
    activeCommander->transfer(Cmdr);  //Transfer this Cmdr to the new active commander
    Cmdr.portSettings(savedSettings); //Transfer this Cmdr to the new active commander
    return 0;
}

//==============================================================================================================================================================================================
//All commands for 'report':
const commandList_t reportCommands[] = {
    {"on",      reportON,         "reporting ON"},
    {"off",     reportOFF,        "reporting OFF"},
    {"exit",    exit,             "Exit to main-menu"},
    {"x",       exit,             "-"}
};
const uint16_t numOfReportCmds = sizeof(reportCommands);
//--------------------------------------------------------------------------------------------
bool reportON(Commander &Cmdr){
    Cmdr.println("reporting ON ");
    reporting = true;
    return 0;
}
bool reportOFF(Commander &Cmdr){
    Cmdr.println("reporting OFF ");
    reporting = false;
    return 0;
}
//==============================================================================================================================================================================================
//All general sub-commands:
const commandList_t Gen_Sub_Cmds[] = {
    {"volt",      VoltVariable, "'v' => Volts"},                                {"v",         VoltVariable, "-"},
    {"amp",       AmpVariable,  "'a' => Amps"},                                 {"a",         AmpVariable,  "-"},
    {"speed",     Speed,        "'r' => Rpm"},                                  {"r",         Speed,        "-"},
    {"reset",     DomReset,     "'R' => Reset all values within this domain"},  {"R",         DomReset,      "-"},
    {"exit",      exit,         "'x' => go to main menu"},                      {"x",         exit,         "-"},
};
const uint16_t numOfGen_Sub_Cmds = sizeof(Gen_Sub_Cmds); //initialise the numOfGen_Sub_Cmds variable after creating the Gen_Sub_Cmds[] array - numOfGen_Sub_Cmds now indicates the length of the array
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool VoltVariable(Commander &Cmdr){
    bool retVal = 0;
    float f;
    if (Cmdr.getFloat(f)) {
        if ((currentDOM == Read)|(currentDOM == Min)|(currentDOM == Max)) {
            Gen_Param_Cmd.print(" => read only value!!!  ");            retVal = 1;
        } else {
            VoltageVar[currentDOM] = f;
        }
    }
    Cmdr.print(Units[Volt]);                Cmdr.print(" = ");
    Cmdr.print(VoltageVar[currentDOM]);     Cmdr.println(" V");
    return retVal;
}
bool AmpVariable(Commander &Cmdr){
    bool retVal = 0;
    float f;
    if (Cmdr.getFloat(f)) {
        if ((currentDOM == Read)|(currentDOM == Min)|(currentDOM == Max)) {
            Gen_Param_Cmd.print(" => read only value!!!  ");            retVal = 1;
        } else {
            AmperageVar[currentDOM] = f;
        }
    }        
    Cmdr.print(Units[Amp]);                 Cmdr.print(" = ");
    Cmdr.print(AmperageVar[currentDOM]);    Cmdr.println(" A");
    return retVal;
}
bool Speed(Commander &Cmdr){
    bool retVal = 0;
    int i;
    if (Cmdr.getInt(i)) {
        if ((currentDOM == Read)|(currentDOM == Min)|(currentDOM == Max)) {
            Gen_Param_Cmd.print(" => read only value!!!  ");            retVal = 1;
        } else {
            SpeedVar[currentDOM] = i;
        }
    }
    Cmdr.print(Units[Rpm]);                 Cmdr.print(" = ");
    Cmdr.print(SpeedVar[currentDOM]);       Cmdr.println(" Rpm");
    return retVal;
}
bool DomReset(Commander &Cmdr){
    VoltageVar[currentDOM] = 0;       AmperageVar[currentDOM] = 0;      SpeedVar[currentDOM] = 0;      
    Cmdr.println(" Values within this domain have been reset to '0'");
    return 0;
}
//==============================================================================================================================================================================================

//REPORT ---------------------------------------------------------------------------
void report_print() {  //Periodically executed in the main loop when reporting is active
    char buf[80];
    Serial.println();
    Serial.println("----------------------------------------------------------------");
    Serial.print("           ");
    for (int j=0; j<MaxUnit; j++){
        Serial.print("            "); Serial.print(Units[j]); 
    } 
    Serial.println();
    Serial.println("----------------------------------------------------------------");
    
    for (int i = 0; i < MaxDomains; i++) { //Set, Read, SetMin, Min, SetMax, Max
        Serial.print(DomainsDescriptor[i]); //Serial.print(masterCommands[i].commandString);
        for (int j=0; j<MaxUnit; j++){
            Serial.print("         ");
            switch (j) {
            case Volt:  sprintf(buf, "%6.2f V", VoltageVar[i]);     Serial.print(buf);                break;
            case Amp:   sprintf(buf, "%6.2f A", AmperageVar[i]);    Serial.print(buf);                break;
            case Rpm:   sprintf(buf, "%4d rpm", SpeedVar[i]);       Serial.print(buf);                break;
            default:                                                                                  break;
            }
        }
        Serial.println();
    }
    Serial.println("----------------------------------------------------------------");
}

//SETUP ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  //initialise the commander objects
  masterCmd.        begin(&Serial, masterCommands,      numOfMasterCmds);
  Gen_Param_Cmd.    begin(&Serial, Gen_Sub_Cmds,    	numOfGen_Sub_Cmds);
  reportCmd.        begin(&Serial, reportCommands,      numOfReportCmds);

  //give each one a name
  masterCmd.        commanderName   = "Main menu";
  Gen_Param_Cmd.    commanderName   = "statistic ";
  reportCmd.        commanderName   = "report";
     
  //enable multi commander mode for all command objects:
  masterCmd.multiCommander(true);     //enable multicommander so command prompts work properly with multiple commander objects
  Gen_Param_Cmd.multiCommander(true);     
  reportCmd.multiCommander(true);

  masterCmd.commandPrompt(ON);        //enable command prompts for the master command object
  masterCmd.echo(true);               //We only need to do this for the master because the settings will be copied to any sub command object we use
  while(!Serial){;}                   //wait for a serial port to open
  Serial.println("Hello: Type 'help' to get help");
  //print the command prompt
  masterCmd.printCommandPrompt();

}

//MAIN LOOP ---------------------------------------------------------------------------
void loop() {
  //Call the update function using the activeCommander pointer
  //This will call the update() function on whatever commander object it is currently pointing to.
  if (reporting) {
      if (Serial.available()) { //After receiving data: set idle period to 5 seconds
          millis_idle = millis();     //reporting_period = idle_period;
      } else {
          if ( (millis() - millis_idle) > reporting_period ) { //Do this every period of time
              report_print();
              activeCommander->printCommandPrompt();
              millis_idle = millis(); //reporting_period = 2500;
          }
      }
  }
  activeCommander->update();

}
