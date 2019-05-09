/**
  Besturingssoftware Pingpongpartner
  Naam: PPP_Brain.cpp
  Purpose: Aansturing van elektronische pingpongpartner op basis van te
           ontvangen bluetooth commando's.

  @authors Ben Mechelmans, Thomas Sergeys
  @version 1.0 6/04/2019
*/

/**
  Verloop van Aansturing
    - Instellen van aantal ballen (standaard 10)
    - Instellen van Home positie (standaard [0, 0])
    - Instellen van



*/

// benodigde software libraries
#include <PWMSoft.h>
#include <SoftPWM_timer.h>
#include <AltSoftSerial.h>
#include <math.h>

AltSoftSerial bluetoothSerial; // de bluetoothconnectie (noem dit hoe je wil maar niet Serial, dat is de USB connectie). RX op pin 9 via spanningsdeler, TX op pin 8

// placeholders voor later gedefinieerde functies
void hardwareSetup();
void rgbSet(int R, int G, int B);
void gradientSet();
void ledSet(String color, int value);
void difSet(int value);
void Reload(String action);
void aim(float speed, float angle);
void shoot(float speed, float angle);
void servoSet(int servoPin, float servoAngle);
void motorSet();
float speed();
void calTraj(float speed, float angle);
float calAnglefromSpeed(float distance, float speed);
float calSpeedfromAngle(float distance, float angle);
void calibration();
String getValue(String data, char separator, int index);


// pin defenities voor digitale pinnen
int PWM_M34_PIN = 2;
int RELOAD_PIN = 3;
int DIRECTION_SWITCH_PIN = 4;
int ROTATION_PIN = 5;
int PWM_M2_PIN = 6;
int LED1_PIN = 7;
int TX_PIN = 8;
int RX_PIN = 9;
int PWM_M1_PIN = 10;
int ELEVATION_PIN = 11;

// pin defenities voor analoge pinnen
int PWM_M56_PIN = A1;
int RED_IN_PIN = A4;
int LED2_PIN = A3;
int BLUE_IN_PIN = A2;
int PHOTO_PIN = A5;
int GREEN_IN_PIN = A0;

// definities voor standaardwaarden
int RELOAD_HOME = 0;
int RELOAD_START = 0;
int RELOAD_END = 10;

int ROTATION_HOME = 0;
int ROATION_START = -10;
int ROTATION_END = 10;

int ELEVATION_HOME = 0;
int ELEVATION_START = -5;
int ELEVATION_END = 5;

int PHOTO_THRESHOLD = 850;  // triggerwaarde voor script
int PHOTO_DISTANCE = 0.05;  // afstand tussen phototransistoren
int PHOTO_DELAY = 0.1;  // delays voor stabiliteit

float WHEELRADIUS = 0.01;  // straal van lanceerwielen [m]
int SPEEDFACTOR = 60/(2*WHEELRADIUS*3.1415);  // overgang van [m/s] naar [rmp]

// defenities voor profiles
int DIFFICULTY = 0;
int RELOAD_SPEED[] = {1000, 1000, 2000};  // {current, min, max}
int MAX_AMMOUNT_OF_BALLS = 10;
int CURRENT_AMMOUNT_OF_BALLS = 10;
float HOMEPOSX[] = {0, 0, 0};  // {current, min, max}
float HOMEPOSY[] = {0, 0, 0};  // {current, min, max}
float TARGETX[] = {0, 0, 0};  // {current, min, max}
float TARGETY[] = {2.6, 2.6, 2.6};  // {current, min, max}
float LEFTSPEED = 2;  // [m/s]
float RIGHTSPEED = 2;
float SPINPERCENTAGE[] = {0, 0, 0};  // {current, min, max}

// defenities voor placeholder variables
String command;  // opslag voor bluetoothbericht
float trajArray[3];  // opslag van berekende baan [elevation, rotation, speed]
bool fireFlag = false;  // controls fire protocol


void setup()
{
  Serial.begin(9600);
  Serial.println(F("Program started running!"));

  delay(1000);

  // configuring all settings for connected hardware
  hardwareSetup();
  delay(100);
}


void loop()
{
  // Kijkt of er iets werd verzonden over Bluetooth, ontvangt het en decodeert het ook.
  while (bluetoothSerial.available())
  {
    delay(10);
    char c = bluetoothSerial.read();
    command += c; // voeg karakter c toe aan command string totdat alle verzonden karakters via bluetoothSerial opgeslagen zijn in command
  }

  // Kijkt of er iets werd verzonden over Serial, ontvangt het en decodeert het ook.
  while (Serial.available())
  {
    delay(10);
    char c = Serial.read();
    command += c; // voeg karakter c toe aan command string totdat alle verzonden karakters via bluetoothSerial opgeslagen zijn in command
  }


  if (command.length() > 0)
    // er werd een commando ontvangen!
  {
    Serial.println(F("-------------------------"));
    Serial.print(F("Received command : "));
    Serial.println(command);
    //bluetoothSerial.print(F("confirmation,"));
    //bluetoothSerial.println(command);


    // endpoint for RGB-LED
    //  input : SETRGB/...;...;...
    //          values between 900 and 1023
    if (command.startsWith("SETRGB/"))
    {
      String number =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.

      int R = getValue(number, ';', 0).toInt();
      int G = getValue(number, ';', 1).toInt();
      int B = getValue(number, ';', 2).toInt();

      rgbSet(R, G, B);
    }

    // endpoint for singlecolor leds
    //  input : SETLED/...;....
    //          values are color and value
    //          color is RED or GREEN
    //          value is 0 or 1
    if (command.startsWith("SETLED/"))
    {
      String number =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.

      String color = getValue(number, ';', 0);
      bool value = getValue(number, ';', 1).toInt();

      ledSet(color, value);
    }

    // endpoint for rotation angle
    //  input : SETROTATION/...
    //          value between ROATION_START and ROTATION_END
    if (command.startsWith("SETROTATION/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      servoSet(ROTATION_PIN, value);
    }

    // endpoint for elevation angle
    //  input : SETELEVATION/...
    //          value between ELEVATION_START and ELEVATION_END
    if (command.startsWith("SETELEVATION/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      servoSet(ELEVATION_PIN, value);
    }

    // endpoint for reload angle
    //  input : SETRELOAD/...
    //          value between RELOAD_START and RELOAD_END
    if (command.startsWith("SETRELOAD/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      servoSet(RELOAD_PIN, value);
    }

    // endpoint for changing difficulty level
    //  input : SETDIFFICULTY/...
    //          value between 0 and 3
    if (command.startsWith("SETDIFFICULTY/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      difSet(value);
    }

    // endpoint for setting wait time between reloads
    //  input : SETRELOADSPEED/...
    //          value larger then 0
    if (command.startsWith("SETRELOADSPEED/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      RELOAD_SPEED[0] = value;
    }

    // endpoint for setting left motor speed
    //  input : SETLEFTSPEED/...
    //          value between 0 and 1023
    if (command.startsWith("SETLEFTSPEED/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      LEFTSPEED = value;
    }

    // endpoint for setting right motor speed
    //  input : SETRIGHTSPEED/...
    //          value between 0 and 1023
    if (command.startsWith("SETRIGHTSPEED/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      RIGHTSPEED = value;
    }

    // endpoint for setting spin percentage
    //  input : SETSPIN/
    //          value between -1 and 1 (left and right)
    if (command.startsWith("SETSPIN/"))
    {
      float value =  command.substring(command.indexOf("/") + 1).toFloat(); // splits het commando op na de / om de parameter in te lezen.
      SPINPERCENTAGE[0] = value;
    }

    // endpoint for setting the maximum amount of available balls after a fresch reload
    //  input : SETMAXBALLS/...
    //          value inspecified
    if (command.startsWith("SETMAXBALLS/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      MAX_AMMOUNT_OF_BALLS = value;
      CURRENT_AMMOUNT_OF_BALLS = value;
    }

    // endpoint for setting home position of the PPP
    //  input : SETHOMEPOS/...;...
    //          values are x and y
    //          must stay within pingpong table dimensions
    if (command.startsWith("SETHOMEPOS/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      HOMEPOSX[0] = getValue(value, ';', 0).toFloat();
      HOMEPOSY[0] = getValue(value, ';', 1).toFloat();
    }

    // endpoint for setting target position for the ball to hit
    //  input : SETTARGETPOS/...;...
    //          values are x and y
    //          must stay within pingpong table dimensions
    if (command.startsWith("SETTARGETPOS/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      TARGETX[0] = getValue(value, ';', 0).toFloat();
      TARGETY[0] = getValue(value, ';', 1).toFloat();
    }

    // endpoint for calculating the recuired ball trajectory to hit the target
    //  input : CALCULATETRAJECTORY/...;...
    //          values are speed and angle
    //          one gets specified, other (the one to be determined) is -1000
    if (command.startsWith("CALCULATETRAJECTORY/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      int speed = getValue(value, ';', 0).toInt();
      int angle = getValue(value, ';', 1).toInt();

      calTraj(speed, angle);
    }

    // endpoint for performing a reload sequence
    //  input : RELOAD/...
    //          value is 'OPEN' or 'CLOSE'
    if (command.startsWith("RELOAD/"))
    {
      String action =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      Reload(action);
    }

    // endpoint for aiming the PPP at target location
    //  input : AIM/
    if (command.startsWith("AIM/"))
    {
      aim(-1000, 30);
    }

    // endpoint for shooting one projectile (used for testing purposes)
    //  input : SHOOT/
    if (command.startsWith("SHOOT/"))
    {
      shoot(-1000, 30);
    }

    // endpoint for resetting amount of balls (drum gets reloaded)
    //  input : SHOOT/
    if (command.startsWith("BALLS/"))
    {
      CURRENT_AMMOUNT_OF_BALLS = MAX_AMMOUNT_OF_BALLS;
    }

    // endpoint for toggling training session
    //  input : TOGGLEFIRE/
    if (command.startsWith("TOGGLEFIRE/"))
    {
      if (fireFlag == true)
      {
        fireFlag = false;
      }
      else if (fireFlag == false)
      {
        fireFlag = true;
      }
      Serial.print(F("fireflag set to ["));
      Serial.print(fireFlag);
      Serial.println("]");
    }

    // endpoint for testing mechanical limits
    //  input : CALIBRATE/
    if (command.startsWith("CALIBRATE/"))
    {
      calibration();
    }

    if (command.startsWith("GETENDSTOPS/"))
    {
      String endstops = String(ROATION_START) + "," +  String(ROTATION_HOME) + "," + String(ROTATION_END) + ","
                + String(ELEVATION_START) + "," + String(ELEVATION_HOME) + "," + String(ELEVATION_END) + ","
                + String(RELOAD_START) + "," + String(RELOAD_HOME) + "," + String(RELOAD_END);
      Serial.println(endstops);
      bluetoothSerial.println(endstops);
    }

    if (command.startsWith("GETTRAININGDETAILS/"))
    {
      String info = String(DIFFICULTY) + "," +  String(MAX_AMMOUNT_OF_BALLS) + ","
                + String(CURRENT_AMMOUNT_OF_BALLS) + "," + String(trajArray[2]);
      Serial.println(info);
      bluetoothSerial.println(info);
    }

    command = "";  // cleaning input stream
  }

  // main pipeline
  if (fireFlag == true)
  {
    if (CURRENT_AMMOUNT_OF_BALLS != 0)
    {
      gradientSet();
      //HOMEPOSX[0] = random(HOMEPOSX[1], HOMEPOSX[2]);  // not used -> PPP unable to move
      //HOMEPOSY[0] = random(HOMEPOSY[1], HOMEPOSY[2]);

      // random target location
      TARGETX[0] = random(TARGETX[1], TARGETX[2]);
      TARGETY[0] = random(TARGETY[1], TARGETY[2]);
      // random reload speed
      RELOAD_SPEED[0] = random(RELOAD_SPEED[1], RELOAD_SPEED[2]);

      // shoot ball at 45° angle
      shoot(-1000, 45);

      CURRENT_AMMOUNT_OF_BALLS --;
      Serial.print(F("Balls remaining : "));
      Serial.println(CURRENT_AMMOUNT_OF_BALLS);

      String info = String(DIFFICULTY) + "," +  String(MAX_AMMOUNT_OF_BALLS) + ","
                    + String(CURRENT_AMMOUNT_OF_BALLS) + "," + String(trajArray[2]);;
      Serial.println(info);
      bluetoothSerial.println(info);
    }
    else
    {
      Serial.println(F("Out of balls! A reload is needed."));
      fireFlag = false;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
//        Functies                                                            //
////////////////////////////////////////////////////////////////////////////////


/**
  Functie bedoeld om alle verbonden hardware te resetten en terug te brengen
  naar hun home posities / standaardstatus.

  @return None
*/

void hardwareSetup()
{
  Serial.println(F("Starting with hardware preperation ..."));
  delay(10);

  Serial.println(F("        Starting SoftPWM ..."));
  SoftPWMBegin();
  delay(10);

  Serial.println(F("        Starting bluetooth connection endpoint ..."));
  bluetoothSerial.begin(9600);
  delay(10);

  Serial.println(F("        Resetting all LED's ..."));
  analogWrite(LED1_PIN, 1000);
  analogWrite(LED2_PIN, 1000);
  digitalWrite(RED_IN_PIN, HIGH);
  digitalWrite(BLUE_IN_PIN, HIGH);
  digitalWrite(GREEN_IN_PIN, HIGH);
  delay(10);

  Serial.println(F("        Homing all servo's ..."));
  servoSet(RELOAD_PIN, RELOAD_HOME);
  servoSet(ROTATION_PIN, ROTATION_HOME);
  servoSet(ELEVATION_PIN, ELEVATION_HOME);
  delay(10);

  Serial.println(F("    Preperation Done!"));
}



/**
  Functie bedoeld om de moeilijkheidsgraad van de PPP aan te passen.
  Verandert globale variabelen zoals snelheid en willekeurigheidsgraad naargelang
  meegegeven moeilijkheidswaarde.

  @param value moeilijkheidswaarde
    0 = [makkelijk]
    1 = [normaal]
    2 = [moeilijk]
    3 = [extreem]
  @return None
*/
void difSet(int value)
{

  if (value == 0)
  {
    Serial.println(F("Difficulty set to [makkelijk]"));
    DIFFICULTY = 0;
    RELOAD_SPEED[1] = 4000;  // min
    RELOAD_SPEED[2] = 4000;  // max
    MAX_AMMOUNT_OF_BALLS = 5;
    TARGETX[1] = 0;  // left
    TARGETX[2] = 0;  // right
    TARGETY[1] = 2.6;  // down
    TARGETY[2] = 2.6;  // up
    SPINPERCENTAGE[1] = 0;  // min
    SPINPERCENTAGE[2] = 0;  // max
  }

  if (value == 1)
  {
    Serial.println(F("Difficulty set to [normaal]"));
    DIFFICULTY = 1;
    RELOAD_SPEED[1] = 2000;  // min
    RELOAD_SPEED[2] = 4000;  // max
    MAX_AMMOUNT_OF_BALLS = 10;
    TARGETX[1] = 0;  // left
    TARGETX[2] = 0;  // right
    TARGETY[1] = 2;  // down
    TARGETY[2] = 2.6;  // up
    SPINPERCENTAGE[1] = 0;  // min
    SPINPERCENTAGE[2] = 0;  // max
  }

  if (value == 2)
  {
    Serial.println(F("Difficulty set to [moeilijk]"));
    DIFFICULTY = 2;
    RELOAD_SPEED[1] = 1000;  // min
    RELOAD_SPEED[2] = 4000;  // max
    MAX_AMMOUNT_OF_BALLS = 10;
    TARGETX[1] = -80;  // left
    TARGETX[2] = 80;  // right
    TARGETY[1] = 2;  // down
    TARGETY[2] = 2.6;  // up
    SPINPERCENTAGE[1] = 0;  // min
    SPINPERCENTAGE[2] = 0;  // max
  }

  if (value == 3)
  {
    Serial.println(F("Difficulty set to [extreem]"));
    DIFFICULTY = 3;
    RELOAD_SPEED[1] = 1000;  // min
    RELOAD_SPEED[2] = 2000;  // max
    MAX_AMMOUNT_OF_BALLS = 10;
    TARGETX[1] = -80;  // left
    TARGETX[2] = 80;  // right
    TARGETY[1] = 2;  // down
    TARGETY[2] = 2.6;  // up
    SPINPERCENTAGE[1] = -0.2;  // min
    SPINPERCENTAGE[2] = 0.2;  // max
  }
}



/**
  Functie om verbonden RGB-LED van kleur te veranderen

  @param R roodwaarde [900-1023]
  @param G groenwaarde [900-1023]
  @param B blauwwaarde [900-1023]
  @return None
*/
void rgbSet(int R, int G, int B)
{
  Serial.println(F("Setting RGB values ..."));
  Serial.print(F("        R = "));
  Serial.println(R);
  Serial.print(F("        G = "));
  Serial.println(G);
  Serial.print(F("        B = "));
  Serial.println(B);


  R = map(R, 0, 255, 1023, 0);
  G = map(G, 0, 255, 1023, 0);
  B = map(B, 0, 255, 1023, 0);

  analogWrite(RED_IN_PIN, R);
  analogWrite(GREEN_IN_PIN, G);
  analogWrite(BLUE_IN_PIN, B);

  Serial.println(F("    Done!"));
}

void gradientSet()
{
  float greenValue = 220 + (float(CURRENT_AMMOUNT_OF_BALLS)/float(MAX_AMMOUNT_OF_BALLS))*30;
  float redValue = 180 + (1-float(CURRENT_AMMOUNT_OF_BALLS)/float(MAX_AMMOUNT_OF_BALLS))*70;
  rgbSet(redValue, greenValue, 0);
}


/**
  Functie om verbonden éénkleurleds te bedienen.

  @param color ledkleur : RED / GREEN
  @param value ledstatus : 0 / 1
  @return None
*/
void ledSet(String color, int value)
{
  Serial.print(F("Setting LED with color "));

  value = map(value, 0, 1, 1, 0);

  if (color == "RED")
  {
    analogWrite(LED1_PIN, value*1000);
    Serial.print(color);
    Serial.print(F(" to "));
    Serial.println(value);
  }
  else
  {
    analogWrite(LED2_PIN, value*1000);
    Serial.print(color);
    Serial.print(F(" to "));
    Serial.println(value);
  }
}

/**
  Functie ter bediening van laadarm om de pingpong ballen tussen de DC motoren
    te duwen

  @param action opdracht voor laadarm
    "OPEN" - opent laadmechanisme
    "CLOSE" - sluit laadmechanisme & duwt bal tussen DC motoren
  @return None
*/
void Reload(String action)
{
  Serial.print(F("Reload ..."));

  if (action == "OPEN")
  {
    Serial.println(F("        Open"));
    servoSet(RELOAD_PIN, RELOAD_START);
  }
  if (action == "CLOSE")
  {
    Serial.println(F("        Close"));
    servoSet(RELOAD_PIN, RELOAD_END);
  }
}


/**
  Functie om PPP volledig te richten.

  @return None
*/
void aim(float speed, float angle)
{
  // defining all needed parameters of trajectory
  calTraj(speed, angle);
  float elevation_angle = trajArray[0];
  float rotation_angle = trajArray[1];
  float motor_speed = trajArray[2];

  // moving in position
  servoSet(ELEVATION_PIN, elevation_angle);
  servoSet(ROTATION_PIN, rotation_angle);
  LEFTSPEED = (1 + SPINPERCENTAGE[0])*motor_speed;
  RIGHTSPEED = (1 - SPINPERCENTAGE[0])*motor_speed;
}

/**
  Pipeline functie voor het afschieten van een pingpong bal. Opent en sluit
    achtereenvolgens het laadmechanisme

  @return None
*/
void shoot(float speed, float angle)
{
  aim(speed, angle);
  motorSet();

  Reload("OPEN");
  Serial.print(F("Reload Delay = "));
  Serial.println(RELOAD_SPEED[0]);
  delay(RELOAD_SPEED[0]);
  Reload("CLOSE");
  float measured_speed = 3;//speed();
  bluetoothSerial.println("speed/" + String(measured_speed));

  delay(1000);

  // stopping motors
  LEFTSPEED = 0;
  RIGHTSPEED = 0;
  motorSet();
}

/**
  Functie om servomotoren te plaatsen op een gespecifieerde hoek.

  @param servoPin aansluitingspin van de te bedienen servo
  @param servoAngle hoek waarop servo moet geplaatst worden
            [-60, 60] ! oppassen met mechanische limieten

  @return None
*/
void servoSet(int servoPin, float servoAngle)
{
  Serial.println(F("Setting servo to angle ..."));
  Serial.print(F("        servoPin = "));
  Serial.println(servoPin);
  Serial.print(F("        servoAngle = "));
  Serial.println(servoAngle);

  float valuePWM = map(servoAngle, -60, 60, 8, 37);
  Serial.print(F("        valuePWM = "));
  Serial.println(valuePWM);

  if ((valuePWM <= 37) && (valuePWM >= 8))
  {
    SoftPWMSet(servoPin, valuePWM);
    Serial.println(F("    Done!"));
  }
  else
  {
    Serial.print(F("    ERR - "));
    Serial.print(valuePWM);
    Serial.print(F(" is geen geldige waarde voor een servomotor!"));
  }
}

/**
  Functie om globale snelheidswaarde toe te passen op de DC motoren.

  @return None
*/
void motorSet()
{
  Serial.println(F("Running DC-motors up to speed ..."));
  Serial.print(F("        Left motor: "));
  Serial.println(LEFTSPEED);
  Serial.print(F("        Right motor: "));
  Serial.println(RIGHTSPEED);
  SoftPWMSet(PWM_M1_PIN, LEFTSPEED*SPEEDFACTOR);
  SoftPWMSet(PWM_M2_PIN, RIGHTSPEED*SPEEDFACTOR);
  Serial.println(F("    Done!"));
}

/**
  Functie om de snelheid van een afgeschoten pingpong bal te berekenen.

  @return snelheid
*/
float speed()
{
  // Meet de snelheid
  int photoVal = analogRead(PHOTO_PIN);
  int timeout_1 = millis();
  int timeout_2 = millis();
  while (timeout_2-timeout_1<3000) {

    if (int(photoVal) > PHOTO_THRESHOLD)
    {
      int time_begin = millis();
      while (int(photoVal) > PHOTO_THRESHOLD)
      {
        int timeout_2 = millis();
        photoVal = analogRead(PHOTO_PIN);
        delay(PHOTO_DELAY);
        if (timeout_2-timeout_1>3000);
          break;
      }

      while (int(photoVal) < PHOTO_THRESHOLD)
      {
        int timeout_2 = millis();
        photoVal = analogRead(PHOTO_PIN);
        delay(PHOTO_DELAY);
        if (timeout_2-timeout_1>3000);
          break;
      }

      int time_end = millis();
      while (int(photoVal) > PHOTO_THRESHOLD)
      {
        int timeout_2 = millis();
        photoVal = analogRead(PHOTO_PIN);
        delay(PHOTO_DELAY);
        if (timeout_2-timeout_1>3000);
          break;
      }
      float time_difference = float( time_end - time_begin ) ;
      float snelheid = float( PHOTO_DISTANCE/time_difference ) ;
      Serial.println(snelheid);
      return snelheid;
    }
    long timeout_2 = millis();
  }
}

/**
  Functie ter berekening van alle nog onbekende variabelen van het baltraject

  @param speed vereiste balsnelheid
    -1000 -> onbepaald
  @param angle vereiste elevatiehoek
    -1000 -> onbepaald
  @return None
*/
void calTraj(float speed, float elevation_angle)
{
  Serial.println(F("Calculating ball trajectory ..."));

  // calculating geometrie
  float deltaX = TARGETX[0] - (HOMEPOSX[0]);
  float deltaY = TARGETY[0] - (HOMEPOSY[0]);
  float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
  float rotation_angle = atan(deltaX / deltaY)*180/3.1415;

  // printing given information
  Serial.print(F("        speed : "));
  Serial.println(speed);
  Serial.print(F("        elevation_angle : "));
  Serial.println(elevation_angle);
  Serial.print(F("        rotate_angle : "));
  Serial.println(rotation_angle);
  Serial.print(F("        deltaX : "));
  Serial.println(deltaX);
  Serial.print(F("        deltaY : "));
  Serial.println(deltaY);
  Serial.print(F("        distance : "));
  Serial.println(distance);

  // calculating missing variables with mathematical model (mupad)
  //   if -1000 is given, seen as unknown
  float new_speed;
  float new_elevation_angle;

  if (speed == -1000)
  {
    new_speed = calSpeedfromAngle(distance, elevation_angle);
    new_elevation_angle = elevation_angle;
  }
  if (elevation_angle == -1000)
  {
    new_speed = speed;
    new_elevation_angle = calAnglefromSpeed(distance, speed);
  }

  // printing findings
  Serial.print(F("        return ["));
  Serial.print(new_elevation_angle);
  Serial.print(F(", "));
  Serial.print(rotation_angle);
  Serial.print(F(", "));
  Serial.print(new_speed);
  Serial.println(F("]"));

  trajArray[0] = new_elevation_angle;
  trajArray[1] = rotation_angle;
  trajArray[2] = new_speed;
}

float calAnglefromSpeed(float distance, float speed)
{
  float new_angle = (57.29577951*acos((0.9449649771*distance)/speed))*180/3.1415;
  return new_angle;
}

float calSpeedfromAngle(float distance, float angle)
{
  float new_speed = ((0.9449649771*distance)/cos(angle));
  return new_speed;
}


/**
  Functie bedoeld om alle verbonden hardware te testen. Gaat alle mechanische
  limieten af ter controle dat de machine tijdens operatie zichzelf niet
  beschadigd. Ook visuele elementen worden getest.

  @return None
*/
void calibration()
{
  Serial.println(F("Calibration started ..."));
  delay(1000);

  Serial.println(F("--> moving servo 1 between end positions..."));
  servoSet(RELOAD_PIN, RELOAD_END);
  delay(1000);
  servoSet(RELOAD_PIN, RELOAD_HOME);
  delay(2000);

  Serial.println(F("--> moving servo 2 between end positions..."));
  servoSet(ROTATION_PIN, ROTATION_END);
  delay(1000);
  servoSet(ROTATION_PIN, ROTATION_HOME);
  delay(2000);

  Serial.println(F("--> moving servo 3 between end positions..."));
  servoSet(ELEVATION_PIN, ELEVATION_END);
  delay(1000);
  servoSet(ELEVATION_PIN, ELEVATION_HOME);
  delay(2000);

  Serial.println(F("--> getting motor 1 up to spead ..."));
  LEFTSPEED = 1023;
  motorSet();
  delay(1000);
  LEFTSPEED = 0;
  motorSet();
  delay(2000);

  Serial.println(F("--> getting motor 2 up to spead ..."));
  RIGHTSPEED = 1023;
  motorSet();
  delay(1000);
  RIGHTSPEED = 0;
  motorSet();
  delay(2000);

  Serial.println(F("--> putting RED-led on and off ..."));
  ledSet("RED", 1);
  delay(1000);
  ledSet("RED", 0);
  delay(2000);

  Serial.println(F("--> putting GREEN-led on and off ..."));
  ledSet("GREEN", 1);
  delay(1000);
  ledSet("GREEN", 0);
  delay(2000);

  Serial.println(F("--> going through RGB colors ..."));
  rgbSet(1023, 0, 0);
  delay(1000);
  rgbSet(0, 1023, 0);
  delay(1000);
  rgbSet(0, 0, 1023);
  delay(1000);
  rgbSet(0, 0, 0);
  delay(2000);

}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
