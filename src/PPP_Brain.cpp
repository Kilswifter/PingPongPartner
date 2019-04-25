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


#include <PWMSoft.h>
#include <SoftPWM_timer.h>
#include <AltSoftSerial.h>
#include <math.h>

AltSoftSerial bluetoothSerial; // de bluetoothconnectie (noem dit hoe je wil maar niet Serial, dat is de USB connectie). RX op pin 9 via spanningsdeler, TX op pin 8
void hardwareSetup();
void rgbSet(int R, int G, int B);
void difSet(int value);
//void dirSet(int value);
void Reload(String action);
void shoot(int speed, int angle);
void servoSet(int servoPin, int servoAngle);
void motorSet();
void speed();
void calTraj(int speed, int angle);
float calAnglefromSpeed(int distance, int speed);
float calSpeedfromAngle(int distance, int angle);
void calibration();


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
int PWM_M56_PIN = A0;
int RED_IN_PIN = A1;
int LED2_PIN = A2;
int BLUE_IN_PIN = A3;
int PHOTO_PIN = A4;
int GREEN_IN_PIN = A5;

// definities voor standaardwaarden
int RELOAD_HOME = 0;
int RELOAD_END = 60;
int ROTATION_HOME = 0;
int ROTATION_END = 60;
int ELEVATION_HOME= 0;
int ELEVATION_END = 60;

int PHOTO_THRESHOLD = 850;  // triggerwaarde voor script
int PHOTO_DISTANCE = 0.05;  // afstand tussen phototransistoren
int PHOTO_DELAY = 0.1;  // delays voor stabiliteit

int SPEEDFACTOR = 1000;

// defenities voor profiles
int DIFFICULTY = 0;
int RELOAD_SPEED[] = {1000, 1000, 2000};
int MAX_AMMOUNT_OF_BALLS = 10;
int CURRENT_AMMOUNT_OF_BALLS = 10;
//  int SERVO_SPEED = 400;
float HOMEPOSX[] = {0, 0, 0};
float HOMEPOSY[] = {0, 0, 0};
float TARGETX[] = {0, 0, 0};
float TARGETY[] = {2.6, 2.6, 2.6};
int LEFTSPEED = 3000;
int RIGHTSPEED = 3000;

// defenities voor placeholder variables
String command;  // opslag voor bluetoothbericht
float trajArray[3];
bool fireFlag = false;


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
  //Kijkt of er iets werd verzonden over Bluetooth, ontvangt het en decodeert het ook.
  while (bluetoothSerial.available())
  {
    delay(10);
    char c = bluetoothSerial.read();
    command += c; // voeg karakter c toe aan command string totdat alle verzonden karakters via bluetoothSerial opgeslagen zijn in command
  }

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
    bluetoothSerial.print(F("confirmation : "));
    bluetoothSerial.println(command);

    if (command.startsWith("SETRGB/"))
    {
      String number =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      int R = (number.substring(0, 2)).toInt();
      int G = (number.substring(3, 6)).toInt();
      int B = (number.substring(7, 10)).toInt();

      rgbSet(R, G, B);
    }

    if (command.startsWith("SETROTATION/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      servoSet(ROTATION_PIN, value);
    }

    if (command.startsWith("SETELEVATION/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      servoSet(ELEVATION_PIN, value);
    }

    if (command.startsWith("SETRELOAD/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      servoSet(RELOAD_PIN, value);
    }

    if (command.startsWith("SETDIFFICULTY/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      difSet(value);
    }

    if (command.startsWith("SETRELOADSPEED/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      RELOAD_SPEED[0] = value;
    }

    if (command.startsWith("SETMAXBALLS/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      MAX_AMMOUNT_OF_BALLS = value;
      CURRENT_AMMOUNT_OF_BALLS = value;
    }

    if (command.startsWith("SETHOMEPOS/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      HOMEPOSX[0] = (value.substring(0, 1)).toInt();
      HOMEPOSY[0] = (value.substring(1, 2)).toInt();
    }

    if (command.startsWith("SETTARGETPOS/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      TARGETX[0] = (value.substring(0, 1)).toInt();
      TARGETY[0] = (value.substring(1, 2)).toInt();
    }

    if (command.startsWith("CALCULATETRAJECTORY/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      int speed = (value.substring(0, command.indexOf("/"))).toInt();
      int angle = (value.substring(value.indexOf("/") + 1)).toInt();

      calTraj(speed, angle);
    }

    if (command.startsWith("RELOAD/"))
    {
      String action =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      Reload(action);
    }

    if (command.startsWith("SHOOT/"))
    {
      shoot(-1000, 30);
    }

    if (command.startsWith("BALLS/"))
    {
      CURRENT_AMMOUNT_OF_BALLS = MAX_AMMOUNT_OF_BALLS;
    }

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

    if (command.startsWith("CALIBRATE/"))
    {
      calibration();
    }

    command = "";
  }

  // main pipeline
  if (fireFlag == true)
  {
    if (CURRENT_AMMOUNT_OF_BALLS != 0)
    {
      HOMEPOSX[0] = random(HOMEPOSX[1], HOMEPOSX[2]);
      HOMEPOSY[0] = random(HOMEPOSY[1], HOMEPOSY[2]);
      TARGETX[0] = random(TARGETX[1], TARGETX[2]);
      TARGETY[0] = random(TARGETY[1], TARGETY[2]);
      RELOAD_SPEED[0] = random(RELOAD_SPEED[1], RELOAD_SPEED[2]);

      shoot(-1000, 45);

      CURRENT_AMMOUNT_OF_BALLS --;
      Serial.print(F("Balls remaining : "));
      Serial.println(CURRENT_AMMOUNT_OF_BALLS);
    }
    else
    {
      fireFlag = false;
    }
  }
}


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
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(RED_IN_PIN, LOW);
  digitalWrite(BLUE_IN_PIN, LOW);
  digitalWrite(GREEN_IN_PIN, LOW);
  delay(10);

  Serial.println(F("        Homing all servo's ..."));
  servoSet(RELOAD_PIN, RELOAD_END);
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
    CURRENT_AMMOUNT_OF_BALLS = 5;
    TARGETX[1] = 0;
    TARGETX[2] = 0;
    TARGETY[1] = 2.6;
    TARGETY[2] = 2.6;
  }

  if (value == 1)
  {
    Serial.println(F("Difficulty set to [normaal]"));
    DIFFICULTY = 1;
    RELOAD_SPEED[1] = 2000;  // min
    RELOAD_SPEED[2] = 4000;  // max
    MAX_AMMOUNT_OF_BALLS = 10;
    TARGETX[1] = 0;
    TARGETX[2] = 0;
    TARGETY[1] = 2;
    TARGETY[2] = 2.6;
  }

  if (value == 2)
  {
    Serial.println(F("Difficulty set to [moeilijk]"));
    DIFFICULTY = 2;
    RELOAD_SPEED[1] = 1000;  // min
    RELOAD_SPEED[2] = 4000;  // max
    MAX_AMMOUNT_OF_BALLS = 10;
    TARGETX[1] = -80;
    TARGETX[2] = 80;
    TARGETY[1] = 2;
    TARGETY[2] = 2.6;
  }

  if (value == 3)
  {
    Serial.println(F("Difficulty set to [extreem]"));
    DIFFICULTY = 3;
    RELOAD_SPEED[1] = 1000;  // min
    RELOAD_SPEED[2] = 2000;  // max
    MAX_AMMOUNT_OF_BALLS = 10;
    TARGETX[1] = -80;
    TARGETX[2] = 80;
    TARGETY[1] = 2;
    TARGETY[2] = 2.6;
  }
}



/**
  Functie om verbonden RGB-LED van kleur te veranderen

  @param R roodwaarde [0-1023]
  @param G groenwaarde [0-1023]
  @param B blauwwaarde [0-1023]
  @return None
*/
void rgbSet(int R, int G, int B)
{
  Serial.println(F("Setting RGB values ..."));
  Serial.print(F("        R = "));
  Serial.println(R);
  Serial.print(F(" G = "));
  Serial.println(G);
  Serial.print(F(" B = "));
  Serial.println(B);

  SoftPWMSet(RED_IN_PIN, R);
  SoftPWMSet(GREEN_IN_PIN, G);
  SoftPWMSet(BLUE_IN_PIN, B);

  Serial.println(F("    Done!"));
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
    servoSet(RELOAD_PIN, RELOAD_HOME);
  }
  if (action == "CLOSE")
  {
    Serial.println(F("        Close"));
    servoSet(RELOAD_PIN, RELOAD_END);
  }
}

/**
  Pipeline functie voor het afschieten van een pingpong bal. Opent en sluit
    achtereenvolgens het laadmechanisme

  @return None
*/
void shoot(int speed, int angle)
{
  // defining all needed parameters of trajectory
  calTraj(speed, angle);
  float elevation_angle = trajArray[0];
  float rotation_angle = trajArray[1];
  float motor_speed = trajArray[2];

  // moving in position
  servoSet(ELEVATION_PIN, elevation_angle);
  servoSet(ROTATION_PIN, rotation_angle);
  LEFTSPEED = motor_speed;
  RIGHTSPEED = motor_speed;
  motorSet();

  Reload("OPEN");
  Serial.print(F("Reload Delay = "));
  Serial.println(RELOAD_SPEED[0]);
  delay(RELOAD_SPEED[0]);
  Reload("CLOSE");

  delay(1000);

  LEFTSPEED = 0;
  RIGHTSPEED = 0;
  motorSet();
}

void servoSet(int servoPin, int servoAngle)
{
  Serial.println(F("Setting servo to angle ..."));
  Serial.print(F("        servoPin = "));
  Serial.println(servoPin);
  Serial.print(F("        servoAngle = "));
  Serial.println(servoAngle);

  int valuePWM = map(servoAngle, -60, 60, 8, 37);
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
void speed()
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
void calTraj(int speed, int elevation_angle)
{
  Serial.println(F("Calculating ball trajectory ..."));

  // calculating geometrie
  float deltaX = TARGETX[0] - int(HOMEPOSX[0]);
  float deltaY = TARGETY[0] - int(HOMEPOSY[0]);
  float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
  float rotation_angle = atan(deltaY / deltaX);

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

float calAnglefromSpeed(int distance, int speed)
{
  float new_angle = (57.29577951*acos((0.9449649771*distance)/speed));
  return new_angle;
}

float calSpeedfromAngle(int distance, int angle)
{
  float new_speed = ((0.9449649771*distance)/cos(angle));
  return new_speed;
}

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
  SoftPWMSet(PWM_M1_PIN,37);
  delay(1000);
  SoftPWMSet(PWM_M1_PIN,8);
  delay(2000);

  Serial.println(F("--> getting motor 2 up to spead ..."));
  SoftPWMSet(PWM_M2_PIN,37);
  delay(1000);
  SoftPWMSet(PWM_M2_PIN,8);
  delay(2000);
}
