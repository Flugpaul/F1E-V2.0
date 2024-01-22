/******************************************************************************************************************************************
F1E_2.0 - 11: 
Einstellung der Kreisflugfunktion 
Servo-Reverse auf -1 gesetzt, da Hitec HS-40 andersrum läuft als die bisherigen China-Servos
Umstellung der Interruppt Pins: 
2: Programming request
3: Circle function
6: Direction button 
7: Servo Reverse
8: RDT interrupt

F1E_2.0 - 10: Sprechende Texte Oled eingebaut
Servo-Reverse auf -1 gesetzt, da das neue Standard-Servo Hitec HS-40 andersrum läuft als die bisherigen China-Servos

F1E_2.0 - 09: Endspurt: Endlich soll sich das F1E-Steuerungsservo bewegen! 
PID-Wert "fest" im EEPROM festlegen
int GetF1E_PID (struct ParameterStruct MyVar) Ermittelt den Korrektur-Faktor
void F1E_Servo_Move (struct ParameterStruct MyVar)   Bewegt das Servo in die Korrektur-Richtung  




F1E_2.0 - 08: Ergänzung um
    - seriell Einlesen - keine 0-Belegung falls Seriel.timeout kommt
      funktioniert
    - Interrupt für DT
      funktioniert mit Werten aus dem Variablen-Struct
      void DT_Servo_Move () . Muss leider mit globalen Variablen arbeiten
    - Variablen für Programmversions-Variante  - für die unterschiedliche Belegung von Pins. 
    - Oled aktiviert
    
    
 
    
1E_2.0 - 07: Ergänzung um
    - Weitere Parameter
    - Trimmung, liest Trimmung aus - im angegebenen Wertebereich
    - Servorevers, Liest Memory, aber schaut auch auf der Hardware/Bord nach. Ist der Jumper gesetzt, wird der Memory Wert "negiert", ansonsten bleibt er bestehen.  
    int F1E_Get_Current_Bearing (struct ParameterStruct &MyVar)  // liefert den Int-Wert der Ausrichtung des Sensors
    int F1E_Get_Wanted_Bearing (struct ParameterStruct &MyVar)  // Liest die gewüncht Richtung bei Drückenm des festgelegten Tasters,
    int  F1E_Get_Wanted_Trim (struct ParameterStruct &MyVar)
    int CheckServoRevers(struct ParameterStruct MyVar)

    Programmier Taster auf ExtraPin gelegt (AddInterruptPin)
    


F1E_2.0 - 06: Ergänzung um
    - DT-Servo, Nutzung der Default-Werte
    - Unterscheidung veränderliche/unveränderliche Parameter
    - Sensorerkennung
    - Kalibierung
    - Fehler "Hängenbleiben" entfernt: Problem war, wenn die serielle Schnittstelle nicht erkannt wurde, blieb das Programm in der Endlosschleife "While (!Serial)"
  

F1E_2.0 - 05: Verändern eines Servo-Wertes und Ansteurung des Servos   klappt : 
Servo.h und Servo.Write genutzt

F1E_2.0 - 04: Verändern der Parameter über die Serielle Schnittstelle als Testlauf für spätere Änderungen. 
    bool CheckNeedforProgramming (struct ParameterStruct &MyVar) // Fragt, ob Programmierwunsch besteht
    void ChangeValuesSerial (struct ParameterStruct &MyVar) // Ändert die Variablen über die serielle Schnittstelle
    void PrintValuesSerial (struct ParameterStruct &MyVar) // Zeigt die aktuellen Werte des Structs an
    void InitPins (struct ParameterStruct &MyVar) // setzt die Pins gem. Vorgabe aus dem Struct
    void GetSerialIntValue(int &VarValue, String VarText) Schreibt auf eine Int-Variable den neuen eingelesenen Wert und zeigt ihn auf der seriellen Schnittstell an

Xiao_I_am_alive(); // Sicherstellen, dass der Prozessor noch lebt
  

F1E_2.0 - 03: Übergabe Struct Parameter via Reference! 
    void InitVariables(struct ParameterStruct &MyVar) // liest die Variablen aus dem Flashspeicher

Klappt! Der Inhalt wird in der Funktion 
InitVariables aus dem Flash Speicher ausgelesen oder beim allerersten Neustart des setup mit den defaultwerten ausgegeben

gestestete Funktionen:
void InitVariables(struct ParameterStruct &MyVar) 




 ******************************************************************************************************************************************/
//==================== Section for Gobal variables=============================

// some global variables
 int counter = 0; 
 int DelayTime = 500;
 char TmpTxt[100];

// variables for RDT-Servo / Interrupt  
int interrupt_DT_Pos;

// Variables for direction change / F1E_Servo_Move ()
int current_Bearing = 180;                          // INT-Variable for current heading
int wanted_Bearing = 180;                           // INT-Variable for desired heading
int delta_Bearing;                                  // INT-(+-)Variable for deviation from desired heading

//PID-correction
int abs_delta_Bearing = 0;
int abs_delta_Bearing_n1 = 0; // default value deviation one measurement before
int abs_delta_Bearing_n2 = 0; // default value deviation two measurements before

//PID-correction

int PID_Output;

//Servo turn direction
int ServoRevers; 
//==================== Section for Gobal variables=============================

// All needed stuff for I2C and Sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// all needed Stuff/Lib. for OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Defaults for the BNO055-Sensor
// Set the delayTime between fresh samples /
#define BNO055_SAMPLERATE_delayTime_MS (100)
// Check I2C device address from BNO055 and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#include <Servo.h> // use of the PWM-library 
Servo ServoF1E; //Servo for F1E-Steering
Servo ServoDT; //Servo for DT-Release
  /* Initialise the servo */

// Needed Stuff for EEPROM 
// Use 0-2. Larger for more debugging messages
#define FLASH_DEBUG       0
#include <FlashStorage_SAMD.h>
const int WRITTEN_SIGNATURE = 0xBEEFDEED;

//Struct-Container for Init-Variables
struct ParameterStruct
{
  char RevText[100];
  int ProgVersion;        //Zähler für die Programmversion 
  int ProgVariant;        //Zähler für die jeweile Variante der aktuellen Programm-Version
  int Switch_1;           //Button for direction and programming
  int ServoPinF1E;    	  //PWM Pin for F1E Servo
  int ServoPosF1E;        //Pos for F1E Servo
  int ServoPinDT;         //PWM Pin for DT Servo
  int ServoPosDT;         //Pos for DT Servo without Interrupt/standBy      
  int ServoPosDT_Interrupt;//Pos for DT Servo by Interrupt 
  int ServoReversePin;    //Pin for ServoRevers indication
  int ServoReverse;       //Value =1 if no ServoRevers desired, 0 for Reverse desired
  int InterruptPin;       //Pin for Interrupt (DT)    
  int AddInterruptPin;    //Pin for additional Interrupt (e.g. Circle)  
  int ProgrammingRequestPin; //Pin for programming request  (e.g. Programming) 
  int TrimPotPin;         //Pin for trim adjustment
  int AddPotPin;          //Pin for additional adjustments
  int MaxTrim;            //Max Value for accelaration of trim position
  int CenterGrad;         //Middle Positon of Servo 
  int MinGrad;            //minimum servo position (45° standard Servo, 60° Cha/KST Servo)
  int MaxGrad;            //maximum servo position (135° standard Servo, 120° Cha/KST Servo)
  float PID_Kp;           //variable for PID-dumping ("proportional")
  float PID_Ki;           //variable for PID-dumping ("integral")
  float PID_Kd;           //variable for PID-dumping ("differential")
  int UpdateFlashCounter; //Counter for amount of calls after last MP flash
} ;

//Instance for the Struct-Container
ParameterStruct F1E_Variables;

  
void setup()
{
 Serial.begin(115200);

 //while (!Serial);
  
 // #### Initialize Variables
 InitVariables(F1E_Variables);
 // info about exixting MP
 Xiao_I_am_alive();
// Initialize OLED Display

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(1, 1);
  display.setTextSize(2);
  display.println(F1E_Variables.RevText);
  display.print("Ver:");
  display.print(F1E_Variables.ProgVersion);
  display.print("/");
  display.println(F1E_Variables.ProgVariant);
  display.display();
  delay(1000);

 Serial.println("===============================================================");
 Serial.println("Restart Nr. "); 
 Serial.println(F1E_Variables.UpdateFlashCounter);
 Serial.println("===============================================================");
 
 PrintValuesSerial(F1E_Variables);
 // #### Init necessary Pins on MP with current Variables
 InitPins(F1E_Variables);

 // #### Move the F1E Servo to defined default position in Variable-Struct
 ServoF1E.write(F1E_Variables.MaxGrad);
 //ServoDT.write(F1E_Variables.ServoPosDT);
 ServoDT.write(F1E_Variables.ServoPosDT_Interrupt);
 
 
 // #### Check for required modifcation of Variable-Values
 if (!CheckNeedforProgramming(F1E_Variables))
  ChangeValuesSerial (F1E_Variables);
    
 PrintValuesSerial(F1E_Variables);
  //Init all with new Values
   InitPins(F1E_Variables);
   ServoRevers = CheckServoRevers(F1E_Variables);
   Serial.print ("ServoReverse set to: ");
   Serial.println (ServoRevers);


  //RDT set to Interupt modus, we have to use a global variable
  interrupt_DT_Pos= F1E_Variables.ServoPosDT_Interrupt;
  attachInterrupt(digitalPinToInterrupt(F1E_Variables.InterruptPin), DT_Servo_Move, LOW);
  
  /* Initialise the sensor */
  Serial.println("Search for BNO055 sensor");
  delay(500);
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    /* display.clearDisplay();
      display.println("No BNO055 detected");
      display.display();
    */
    Oprintln ("No BNO055 detected");
    Serial.println ("No BNO055 detected");
    while (1);
  }
  Oprintln ("BNO055 detected");
  Serial.println("BNO055 detected");
  
  // Check calibration status in a loop. End setup only, when calibration is sufficient
  //Calibrate
  Oprintln ("Calibration started");

  while (Analyse_CalStatus())
  {
    Serial.println("Not yet sufficiant calibrated");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(DelayTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(DelayTime);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println ("Calibration ended!");
  Oprintln ("Calibration ended!");
  delay(1000);
  Oprintln ("Fly a Max!");
  delay(1000);

// Check Servo Reverse 
ServoRevers=CheckServoRevers(F1E_Variables);
Serial.print ("ServoReverse set to: ");
Serial.println (ServoRevers);
if (ServoRevers==1)
Oprintln ("ServoReverse is enabled");


// Move the Servos into default positions  
 ServoF1E.write(F1E_Variables.ServoPosF1E);
 ServoDT.write(F1E_Variables.ServoPosDT);


current_Bearing = F1E_Get_Current_Bearing (F1E_Variables);
Serial.print("current_Bearing: ");
Serial.println(current_Bearing);
wanted_Bearing = current_Bearing;

F1E_Get_Wanted_Trim (F1E_Variables);
F1E_Servo_Move (F1E_Variables);

}

void loop()
{

Serial.print(counter);
counter++;
Serial.println("------------------------------------------");
 
//Decide Cicle or heading 
// Circle demand will be set by the AddInterruptPin
if(!digitalRead(F1E_Variables.AddInterruptPin))   
  {
  //Circle Trim is demanded
  F1E_Get_and_Move_To_Wanted_Circle_Trim(F1E_Variables);
  Serial.print("Circle function is enabled!");
  Oprintln ("Circle function is enabled!");
  }
else
  {
  //Normal heading flight is demanded
  current_Bearing = F1E_Get_Current_Bearing (F1E_Variables);
  Serial.print("current_Bearing: ");
  Serial.println(current_Bearing);
  wanted_Bearing = F1E_Get_Wanted_Bearing (F1E_Variables);
  Serial.print("wanted_Bearing: ");
  Serial.println(wanted_Bearing);  
  F1E_Servo_Move (F1E_Variables);
  }
}

void InitVariables(struct ParameterStruct &MyVar)
{

//  Serial.println(BOARD_NAME);
//  Serial.println(FLASH_STORAGE_SAMD_VERSION);
//  Serial.print("EEPROM length: ");
//  Serial.println(EEPROM.length());
  // Check signature at address 0
  int signature;
  uint16_t storedAddress = 0;

 EEPROM.get(storedAddress, signature);

  // If the EEPROM is empty then no WRITTEN_SIGNATURE

  if (signature == WRITTEN_SIGNATURE)
  {
    EEPROM.get(storedAddress + sizeof(signature), MyVar);

    // ...and finally save everything into emulated-EEPROM
    EEPROM.put(storedAddress + sizeof(signature), MyVar);

    if (!EEPROM.getCommitASAP())
    {
      Serial.println("CommitASAP not set. Need commit()");
      EEPROM.commit();
    }
  }
  else
  {
    Serial.println("EEPROM is empty, writing WRITTEN_SIGNATURE and default data:");
    EEPROM.put(storedAddress, WRITTEN_SIGNATURE);
   
    
    //Defintion of default-values!
    //==================================================== 
    String name = "F1E-2.0";
    name.toCharArray(MyVar.RevText, 100);
    MyVar.ProgVersion=2;
    MyVar.ProgVariant=1;   //1 for RDT Version
    MyVar.UpdateFlashCounter=0;   
    MyVar.ServoPinF1E=10;
    MyVar.ServoPosF1E=90;
    MyVar.ServoPinDT=9;
    MyVar.ServoPosDT=90;
    MyVar.ServoPosDT_Interrupt=180;
    MyVar.Switch_1= 6;
    MyVar.ServoReversePin=7;
    MyVar.ServoReverse=-1;  // Value 1 or -1 
    MyVar.InterruptPin=8;
    MyVar.AddInterruptPin=3;
    MyVar.ProgrammingRequestPin=2;
    MyVar.TrimPotPin=A0;
    MyVar.MaxTrim=40;
    MyVar.CenterGrad=90;
    MyVar.MinGrad=60;
    MyVar.MaxGrad=120;
    MyVar.AddPotPin=A1;
    MyVar.PID_Kp=0.5;
    MyVar.PID_Ki=0.05;
    MyVar.PID_Kd=0.25;
     //====================================================
  
     // ...and finally save everything into emulated-EEPROM
    EEPROM.put(storedAddress + sizeof(signature), MyVar);

    if (!EEPROM.getCommitASAP())
    {
      Serial.println("CommitASAP not set. Need commit()");
      EEPROM.commit();
    }
  }
}
bool CheckNeedforProgramming(struct ParameterStruct &MyVar)
{
  if (!digitalRead(MyVar.ProgrammingRequestPin))
  {
  Serial.print ("Programming-Request on Pin ");
   Oprintln ("Programming-Request!");
  Serial.println (MyVar.ProgrammingRequestPin);
  return (0);  // Bedingung erfüllt
  }
  else
  {
  Serial.println ("No-Request!");
  return (1);  // Bedingung nicht erfüllt   
  }
}
  
void ChangeValuesSerial (struct ParameterStruct &MyVar)
{
  // Check signature at address 0
  int signature;
  int userInput;
  int OldValue;
  const int bufferSize = 10; // Größe des Eing    abepuffers
  char inputBuffer[bufferSize]; // Eingabepuffer

  uint16_t storedAddress = 0;
  Serial.println("EEPROM -  write new Data:");

  EEPROM.put(storedAddress, WRITTEN_SIGNATURE);

    Serial.setTimeout(10000); // Timeout, if there is no input on serial, default time 10 s
    Serial.println(MyVar.RevText);
    MyVar.ProgVariant=GetSerialIntValue(MyVar.ProgVariant,"MyVar.ProgVariant");
    //MyVar.Switch_1=GetSerialIntValue(MyVar.Switch_1,"MyVar.Switch_1");
    MyVar.ServoPosF1E=GetSerialIntValue(MyVar.ServoPosF1E,"MyVar.ServoPosF1E");
    //MyVar.ServoPinF1E=GetSerialIntValue(MyVar.ServoPinF1E,"MyVar.ServoPinF1E");
    MyVar.ServoPosDT=GetSerialIntValue(MyVar.ServoPosDT,"MyVar.ServoPosDT");
    MyVar.ServoPosDT_Interrupt=GetSerialIntValue(MyVar.ServoPosDT_Interrupt,"MyVar.ServoPosDT_Interrupt");
    //MyVar.ServoPinDT=GetSerialIntValue(MyVar.ServoPinDT,"MyVar.ServoPinDT");
    MyVar.MaxTrim=GetSerialIntValue(MyVar.MaxTrim,"MyVar.MaxTrim");
    MyVar.MinGrad=GetSerialIntValue(MyVar.MinGrad,"MyVar.MinGrad");
    MyVar.MaxGrad=GetSerialIntValue(MyVar.MaxGrad,"MyVar.MinGrad");
    MyVar.ServoReverse=GetSerialIntValue(MyVar.ServoReverse,"MyVar.ServoReverse");
  
  //Store request? Some Problems with use of the struct
    //Serial.print("Store Data? (Y=1/N=0) ?:");
    //userInput = Serial.parseInt();
    //if (userInput == 1) 
     // {
      // ...and finally save everything into emulated-EEPROM
      // Increase the counter
      MyVar.UpdateFlashCounter=MyVar.UpdateFlashCounter+1;
      EEPROM.put(storedAddress + sizeof(signature), MyVar);
      Serial.println("Data stored");
     


      if (!EEPROM.getCommitASAP())
        {
        Serial.println("CommitASAP not set. Need commit()");
        EEPROM.commit();
        }
      
 //     }
  
 //   else
 //     {
 //     Serial.println("No Data stored");
 //     }



  }
void InitPins(struct ParameterStruct &MyVar)
{
   // Set the switch-Buttons - button default High
  pinMode(MyVar.Switch_1, INPUT);          // define button type
  digitalWrite(MyVar.Switch_1, HIGH);      // button default High 
   // Set the Interrupt Pin - Pin default High
  pinMode(MyVar.ServoReversePin, INPUT);          // define button type
  digitalWrite(MyVar.ServoReversePin, HIGH);      // button default High 
   // Set the Interrupt Pin - Pin default High
  pinMode(MyVar.InterruptPin, INPUT);          // define button type
  digitalWrite(MyVar.InterruptPin, HIGH);      // button default High 
  // Set the Interrupt Pin - Pin default High
  pinMode(MyVar.AddInterruptPin, INPUT);          // define button type
  digitalWrite(MyVar.AddInterruptPin, HIGH);      // button default High 
  // Set the prgramming Pin - Pin default High 
  pinMode(MyVar.ProgrammingRequestPin, INPUT);          // define button type
  digitalWrite(MyVar.ProgrammingRequestPin, HIGH);      // button default High 
 
  // Initialise the servos
  ServoF1E.attach(MyVar.ServoPinF1E);    // connect Servo F1E.Switch_1, HIGH); 
  ServoDT.attach(MyVar.ServoPinDT);    // connect Servo F1E.Switch_1, HIGH);  
}

void Xiao_I_am_alive()
{
  int Xiao_zaehler =0;
  while(Xiao_zaehler < 10)
  {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);  
  Xiao_zaehler++;
  }
}

void PrintValuesSerial (struct ParameterStruct &MyVar)
{
   // Print a list of the current data inserted.
    Serial.println("List of Variables set: ");
    Serial.print("RevText : ");
    Serial.println(MyVar.RevText);
    Serial.print("ProgVersion : ");
    Serial.println(MyVar.ProgVersion);
    Serial.print("ProgVariant : ");
    Serial.println(MyVar.ProgVariant);
    Serial.print("UpdateCounter : ");
    Serial.println(MyVar.UpdateFlashCounter);
    Serial.print("Switch_1 : ");
    Serial.println(MyVar.Switch_1);
    Serial.print("ServoPosF1E : ");
    Serial.println(MyVar.ServoPosF1E);
    Serial.print("ServoPinF1E : ");
    Serial.println(MyVar.ServoPinF1E);
    Serial.print("ServoPosDT : ");
    Serial.println(MyVar.ServoPosDT);
    Serial.print("ServoPosDT_Interrupt : ");
    Serial.println(MyVar.ServoPosDT_Interrupt);
    Serial.print("ServoPinDT : ");
    Serial.println(MyVar.ServoPinDT);
    Serial.print("ServoReversePin: ");
    Serial.println(MyVar.ServoReversePin);
    Serial.print("ServoReverse: ");
    Serial.println(MyVar.ServoReverse);
    Serial.print("InterruptPin: ");
    Serial.println(MyVar.InterruptPin);
    Serial.print("AddInterruptPin: ");
    Serial.println(MyVar.AddInterruptPin);
    Serial.print("ProgrammingRequestPin: ");
    Serial.println(MyVar.ProgrammingRequestPin);
    Serial.print("TrimPotPin: ");
    Serial.println(MyVar.TrimPotPin);
    Serial.print("AddPotPin: ");
    Serial.println(MyVar.AddPotPin);
    Serial.print("MaxTrim: ");
    Serial.println(MyVar.MaxTrim);
    Serial.print("CenterGrad: ");
    Serial.println(MyVar.CenterGrad);
    Serial.print("MinGrad: ");
    Serial.println(MyVar.MinGrad);
    Serial.print("MaxGrad: ");
    Serial.println(MyVar.MaxGrad);
    Serial.print("PID_Kp: ");
    Serial.println(MyVar.PID_Kp);
    Serial.print("PID_Ki: ");
    Serial.println(MyVar.PID_Ki);
    Serial.print("PID_Kd: ");
    Serial.println(MyVar.PID_Kd);
    
}

int GetSerialIntValue(int VarValue, String VarText)
{
  //Function for set Int-Value with value read from serial port. If nothing is done, onlx CR the old value will remain
  const int bufferSize = 10; // Size of buffer
  int amountcharacters; // amount of return characters from seriell input
  char inputBuffer[bufferSize]; // buffer 
  int OldValue=VarValue;
  Serial.print(VarText);
  Serial.print(" (");
  Serial.print(VarValue);
  Serial.print("): ");
  //Check for "nothing done" or only CR / timeout or "Enter" without new value
  amountcharacters = Serial.readBytesUntil('\n', inputBuffer, bufferSize);
  if (amountcharacters == 0)
  {
    VarValue = OldValue;
    Serial.println("No change, Timeout, old value will resume ");
  }
  else
  {
  if (inputBuffer[0]!= 13) 
    {
      VarValue = atoi(inputBuffer);
      Serial.println("new Value entered");
    }
    else
    {
      VarValue = OldValue;
      Serial.println("No change, Enter-key as only input, old value will resume ");
    }
  }
  Serial.print("New Value: ");
  Serial.println(VarValue);
return (VarValue);
 }

 void MyPrintln (String PrintText)
 {
  display.println(PrintText);
  Serial.println(PrintText);
 }

int Oprintln(char * PrintText)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(5, 10);
  display.println(PrintText);
  display.display();
  return(0);
}

/**************************************************************************/
/*
    Analyse the Calibration status
*/
/**************************************************************************/
bool Analyse_CalStatus ()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // If the sensor is not sufficient calibrated, return 1.
  if (gyro > 2 && mag > 2)  {
    return 0;
  }
  return 1;
}  

/**************************************************************************/
/*
    determine current heading
*/
/**************************************************************************/
int F1E_Get_Current_Bearing (struct ParameterStruct &MyVar)
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int heading = euler.x(); // transform to int

  /* Display the floating point data */
  //Serial.print("X: ");
  //Serial.print(euler.x());
  //Serial.print(" Y: ");
  //Serial.print(euler.y());
  //Serial.print(" Z: ");
  //Serial.print(euler.z());
  //Serial.print("\t\t");

  // print the heading, pitch and roll    Serial.print("Orientation: ");
  //Serial.print(heading);
  //Serial.print(", ");
  //Serial.print(current_Bearing);
  //Serial.print(", ");
  //Serial.print(wanted_Bearing);
  //Serial.print(", ");
  //Serial.print(pitch);
  //Serial.print(", ");
  //Serial.println(roll);
  return (heading);
}
/**************************************************************************/
/*
    Determine the desired bearing
*/
/**************************************************************************/
int F1E_Get_Wanted_Bearing (struct ParameterStruct &MyVar)
{
  if (!digitalRead(MyVar.Switch_1))
  {
    // Check if there is a change necessary for the middle position
    //
    F1E_Get_Wanted_Trim (MyVar);

    // new wanted bearing
    wanted_Bearing = current_Bearing;
  }
  return(wanted_Bearing);
}

/**************************************************************************/
/*
    Determine the desired trim
*/
/**************************************************************************/
int  F1E_Get_Wanted_Trim (struct ParameterStruct &MyVar)
{
  static int pwmOld;
  // Check if there is a change necessary for the middle position
  int sensor = analogRead(MyVar.TrimPotPin);
  int minGrad = MyVar.MinGrad;
  int maxGrad = MyVar.MaxGrad;
  int pwm_F1E = map(sensor, 0, 1023, 0, MyVar.MaxTrim) - MyVar.MaxTrim / 2;

  // new wanted trim
  MyVar.CenterGrad = MyVar.CenterGrad - (pwm_F1E-pwmOld);
  // adjust min and max Grad
  MyVar.MinGrad = MyVar.MinGrad - (pwm_F1E-pwmOld);
  MyVar.MaxGrad = MyVar.MaxGrad - (pwm_F1E-pwmOld);
  pwmOld = pwm_F1E;
Serial.print("pwm_F1E: ");
Serial.println(pwm_F1E);
  return (pwm_F1E);
}
//Change serve-direction if there is a hardware-junper plugged in (Struct is only read, copy transfered)
int CheckServoRevers(struct ParameterStruct MyVar)
{
  int intvalue; 
  if (!digitalRead(MyVar.ServoReversePin))
  {
    Serial.println("ServoRevers Jumper set");
    intvalue = -1 * MyVar.ServoReverse;
  }
  else
  {
  Serial.println("ServoRevers Jumper not set");
  intvalue = MyVar.ServoReverse;
  }
return (intvalue);
}

void DT_Servo_Move ()
{
Serial.println("RDT received, Interruped happened, DT Servo move to RDT-postion"); 
//Oprintln("RDT received");  There is a mafunction calling the display function. I do not now why. 
ServoDT.write(interrupt_DT_Pos);
return; 
}

int GetF1E_PID (struct ParameterStruct MyVar)
{
  int outputvalue; 
  // Output mit PID - Kp modifizieren
  //PID_Output = PID_Kp * abs_delta_Bearing + PID_Ki * (abs_delta_Bearing + abs_delta_Bearing_n1 + abs_delta_Bearing_n2) + PID_Kd * (abs_delta_Bearing - abs_delta_Bearing_n1);
  outputvalue = MyVar.PID_Kp * abs_delta_Bearing + MyVar.PID_Ki * (abs_delta_Bearing + abs_delta_Bearing_n1 + abs_delta_Bearing_n2) + MyVar.PID_Kd * (abs_delta_Bearing - abs_delta_Bearing_n1);
  return (outputvalue);
  }

  /**************************************************************************/
/*
    Moves the F1E-servo if there is a deviation to desired/wanted direction
*/
/**************************************************************************/
void F1E_Servo_Move (struct ParameterStruct MyVar)
{
  delta_Bearing =  wanted_Bearing - current_Bearing;
  //Modulo for beeing aware to 0/360° change of direction and calculation only the deviation/direction for corretion
  //Serial.print("Modulo degrees change from desired direction: ");

  //Remember to previous delta_bearing - they will be used for PID calculation
  abs_delta_Bearing_n2 = abs_delta_Bearing_n1;
  abs_delta_Bearing_n1 = abs_delta_Bearing;
  abs_delta_Bearing = ((delta_Bearing + 540) % 360) - 180;


  int pid_output=GetF1E_PID(MyVar);
  int servoPos = MyVar.CenterGrad - (MyVar.ServoReverse * pid_output);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(1, 1);
  display.println(servoPos);
  display.setCursor(1, 17);
  display.println(wanted_Bearing);
  display.setCursor(50,17);
  display.println(current_Bearing);
  display.display();
  //delay (10);
  // Limitation of correction
  if (servoPos > MyVar.MaxGrad) {
    servoPos = MyVar.MaxGrad;
  }
  if (servoPos < MyVar.MinGrad) {
    servoPos = MyVar.MinGrad;
  }

  // go to new Servo position
  ServoF1E.write(servoPos);

  return;
}

void F1E_Get_and_Move_To_Wanted_Circle_Trim (struct ParameterStruct MyVar)
  {
    int sensor = analogRead(MyVar.AddPotPin);
    int MappedPwm = map(sensor, 0, 1023, 0, MyVar.MaxGrad-MyVar.MinGrad)+ MyVar.MinGrad; 
    ServoF1E.write(MappedPwm); 
    return;
  }
