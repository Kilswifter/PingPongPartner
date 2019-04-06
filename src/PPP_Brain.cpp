/**
  Besturingssoftware Pingpongpartner
  Naam: PPP_Brain.cpp
  Purpose: Aansturing van elektronische pingpongpartner op basis van te
           ontvangen bluetooth commando's.

  @authors Ben Mechelmans, Thomas Sergeys
  @version 1.0 6/04/2019
*/

#include <PWMSoft.h>
#include <SoftPWM_timer.h>
#include <AltSoftSerial.h>

AltSoftSerial bluetoothSerial; // de bluetoothconnectie (noem dit hoe je wil maar niet Serial, dat is de USB connectie). RX op pin 9 via spanningsdeler, TX op pin 8
void hardwareSetup();
void rgbSet(int R, int G, int B);
void difSet(int value);
void dirSet(int value);
void Reload(String action);
void Shoot();
void servoSet(int servoPin, int servoAngle);
void speed();


// pin defenities voor digitale pinnen
int PWM_M34_PIN = 2;
int PWM_S1_PIN = 3;
int DIRECTION_SWITCH_PIN = 4;
int PWM_S2_PIN = 5;
int PWM_M2_PIN = 6;
int LED1_PIN = 7;
int TX_PIN = 8;
int RX_PIN = 9;
int PWM_M1_PIN = 10;
int PWM_S3_PIN = 11;

// pin defenities voor analoge pinnen
int PWM_M56_PIN = A0;
int RED_IN_PIN = A1;
int LED2_PIN = A2;
int BLUE_IN_PIN = A3;
int PHOTO_PIN = A4;
int GREEN_IN_PIN = A5;

// definities voor standaardwaarden
int PWM_S1_HOME = 0;
int PWM_S1_END = 10;
int PWM_S2_HOME = 0;
int PWM_S3_HOME = 0;

int SERVOMIN = 8;
int SERVOMAX = 37;

int PHOTO_THRESHOLD = 850;  // triggerwaarde voor script
int PHOTO_DISTANCE = 0.05;  // afstand tussen phototransistoren
int PHOTO_DELAY = 0.1;  // delays voor stabiliteit

// defenities voor profiles
int reloadProfile[] = {1, 2, 3, 4};
int ITEMS_PER_PROFILE = 1;
int DIFFICULTY = 0;
int RELOAD_SPEED = 10;
int MAX_AMMOUNT_OF_BALLS = 10;
int CURRENT_AMMOUNT_OF_BALLS = 10;
//int SERVO_SPEED = 400;
int HOMEPOSX = 0;
int HOMEPOSY = 0;
int TARGETX = 0;
int TARGETY = 0;
int LEFTSPEED = 30;
int RIGHTSPEED = 30;


// defenities voor placeholder variables
unsigned long time_begin, time_end;
float snelheid, time_difference;
String command;  // opslag voor bluetoothbericht


void setup() {
  Serial.begin(9600);
  Serial.println("Program started running!");

  delay(1000);

  // configuring all settings for connected hardware
  hardwareSetup();
  delay(100);
}




void loop() {
  //Kijkt of er iets werd verzonden over Bluetooth, ontvangt het en decodeert het ook.
  while (bluetoothSerial.available())
  {
    delay(10);
    char c = bluetoothSerial.read();
    command += c; // voeg karakter c toe aan command string totdat alle verzonden karakters via bluetoothSerial opgeslagen zijn in command
  }


  if (command.length() > 0)
    // er werd een commando ontvangen!
  {
    //Serial.println(command);
    //bluetoothSerial.println(command);

    if (command.startsWith("SETRGB/"))
    {
      String number =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      int R = (number.substring(0, 2)).toInt();
      int G = (number.substring(3, 6)).toInt();
      int B = (number.substring(7, 10)).toInt();
      rgbSet(R, G, B);

      bluetoothSerial.print(R);
      bluetoothSerial.print(" ");
      bluetoothSerial.print(G);
      bluetoothSerial.print(" ");
      bluetoothSerial.println(B);
      delay(5);
    }

    if (command.startsWith("SETDIFFICULTY/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      difSet(value);
    }

    if (command.startsWith("SETDIRECTION/"))
    {
      int value =  command.substring(command.indexOf("/") + 1).toInt(); // splits het commando op na de / om de parameter in te lezen.
      dirSet(value);
    }

    if (command.startsWith("SETHOMELOCATION/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      int X = (value.substring(0, 1)).toInt();
      int Y = (value.substring(1, 2)).toInt();
      HOMEPOSX = X;
      HOMEPOSY = Y;
    }

    if (command.startsWith("SETTARGETLOCATION/"))
    {
      String value =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      int X = (value.substring(0, 1)).toInt();
      int Y = (value.substring(1, 2)).toInt();
      TARGETX = X;
      TARGETY = Y;
    }

    if (command.startsWith("RELOAD/"))
    {
      String action =  command.substring(command.indexOf("/") + 1); // splits het commando op na de / om de parameter in te lezen.
      Reload(action);
    }

    if (command.startsWith("SHOOT/"))
    {
      Shoot();
    }
  }


}


void hardwareSetup()
{
  Serial.println("Starting with hardware preperation ...");
  delay(10);

  Serial.println("        Starting SoftPWM ...");
  SoftPWMBegin();
  delay(10);

  Serial.println("        Starting bluetooth connection endpoint ...");
  bluetoothSerial.begin(9600);
  delay(10);

  Serial.println("        Resetting all LED's ...");
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(RED_IN_PIN, LOW);
  digitalWrite(BLUE_IN_PIN, LOW);
  digitalWrite(GREEN_IN_PIN, LOW);
  delay(10);

  Serial.println("        Homing all servo's ...");
  servoSet(PWM_S1_PIN, PWM_S1_HOME);
  servoSet(PWM_S2_PIN, PWM_S2_HOME);
  servoSet(PWM_S3_PIN, PWM_S3_HOME);
  delay(10);

  Serial.println("    Preperation Done!");
}

void dirSet(int value)
{
  switch (value)
  {
    case 0:
      Serial.println("linksachter");
      SoftPWMSet(PWM_M1_PIN,37);//motoren activeren
      SoftPWMSet(PWM_M2_PIN,37);
      servoSet(PWM_S1_PIN,-30); //De richting naar links
      servoSet(PWM_S2_PIN, 60); //Het balletje naar voor duwen
      speed();
      servoSet(PWM_S2_PIN,-60); // Nieuw balletje laten vallen
      CURRENT_AMMOUNT_OF_BALLS -- ;

    case 1:
      Serial.println("rechtsachter");
      SoftPWMSet(PWM_M1_PIN,37);//motoren activeren
      SoftPWMSet(PWM_M2_PIN,37);
      servoSet(PWM_S1_PIN,30); //De richting naar rechts
      servoSet(PWM_S2_PIN, 60); //Het balletje naar voor duwen
      speed();
      servoSet(PWM_S2_PIN,-60); // Nieuw balletje laten vallen
      CURRENT_AMMOUNT_OF_BALLS -- ;

    case 2:
      Serial.println("linksvoor");
      SoftPWMSet(PWM_M1_PIN,20);//motoren activeren
      SoftPWMSet(PWM_M2_PIN,20);
      servoSet(PWM_S1_PIN,-30); //De richting naar links
      servoSet(PWM_S2_PIN, 60); //Het balletje naar voor duwen
      speed();
      servoSet(PWM_S2_PIN,-60); // Nieuw balletje laten vallen
      CURRENT_AMMOUNT_OF_BALLS -- ;

    case 3:
      Serial.println("rechtsvoor");
      SoftPWMSet(PWM_M1_PIN,37);//motoren activeren
      SoftPWMSet(PWM_M2_PIN,37);
      servoSet(PWM_S1_PIN,30); //De richting naar rechts
      servoSet(PWM_S2_PIN, 60); //Het balletje naar voor duwen
      speed();
      servoSet(PWM_S2_PIN,-60); // Nieuw balletje laten vallen
      CURRENT_AMMOUNT_OF_BALLS -- ;

  }
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
  switch (value)
  {
    case 0:
      Serial.println("Difficulty set to [makkelijk]");
      Serial.println("        RELOAD_SPEED = " );
      RELOAD_SPEED = 1;
      while (CURRENT_AMMOUNT_OF_BALLS > 0) {
        int Direction = random(0,3);
        dirSet(Direction);
        delay(20);
      }


    case 1:
      Serial.println("Difficulty set to [normaal]");
      Serial.println("        RELOAD_SPEED = " + 2);
      RELOAD_SPEED = 2;
      while (CURRENT_AMMOUNT_OF_BALLS > 0) {
        int Direction = random(0,3);
        dirSet(Direction);
        delay(30);
      }

    case 2:
      Serial.println("Difficulty set to [moeilijk]");
      Serial.println("        RELOAD_SPEED = " + 3);
      RELOAD_SPEED = 3;
      while (CURRENT_AMMOUNT_OF_BALLS > 0) {
        int Direction = random(0,3);
        dirSet(Direction);
        delay(40);
      }

    case 3:
      Serial.println("Difficulty set to [extreem]");
      Serial.println("        RELOAD_SPEED = " + 4);
      // tijd tussen ballen is het kortst
      RELOAD_SPEED = 4;
      while (CURRENT_AMMOUNT_OF_BALLS > 0) {
        int Direction = random(0,3);
        dirSet(Direction);
        delay(50);
      }
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
  Serial.println("Setting RGB values ...");
  Serial.print("        R = ");
  Serial.println(R);
  Serial.print(" G = ");
  Serial.println(G);
  Serial.print(" B = ");
  Serial.println(B);

  SoftPWMSet(RED_IN_PIN, R);
  SoftPWMSet(GREEN_IN_PIN, G);
  SoftPWMSet(BLUE_IN_PIN, B);
  Serial.println("    Done!");
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
  Serial.print("Reload ...");

  if (action == "OPEN")
  {
    Serial.println("        Open");
    servoSet(PWM_S1_PIN, PWM_S1_HOME);
  }
  if (action == "CLOSE")
  {
    Serial.println("        Close");
    servoSet(PWM_S1_PIN, PWM_S1_END);
  }
}

/**
  Pipeline functie voor het afschieten van een pingpong bal. Opent en sluit
    achtereenvolgens het laadmechanisme

  @return None
*/
void Shoot()
{
  Reload("OPEN");
  delay(1000);
  Reload("CLOSE");
}

void servoSet(int servoPin, int servoAngle)
{
  Serial.println("Setting servo to angle ...");
  Serial.print("        servoPin = ");
  Serial.println(servoPin);
  Serial.print("        servoAngle = ");
  Serial.println(servoAngle);

  int valuePWM = map(servoAngle, -60, 60, 8, 37);
  Serial.print("        valuePWM = ");
  Serial.println(valuePWM);

  if ((valuePWM <= SERVOMAX) && (valuePWM >= SERVOMIN))
  {
    SoftPWMSet(servoPin, valuePWM);
    Serial.println("    Done!");
  }
  else
  {
    Serial.print("    ERR - ");
    Serial.print(valuePWM);
    Serial.print(" is geen geldige waarde voor een servomotor!");
  }
}

/**
  Functie om globale snelheidswaarde toe te passen op de DC motoren.

  @return None
*/
void motorSet()
{
  Serial.println("Running DC-motors up to speed ...");
  Serial.print("        Left motor: ");
  Serial.println(LEFTSPEED);
  Serial.print("        Right motor: ");
  Serial.println(RIGHTSPEED);
  SoftPWMSet(PWM_S1_PIN, LEFTSPEED);
  SoftPWMSet(PWM_S2_PIN, RIGHTSPEED);
  Serial.println("    Done!");

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
      time_begin = millis();
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

      time_end = millis();
      while (int(photoVal) > PHOTO_THRESHOLD)
      {
        int timeout_2 = millis();
        photoVal = analogRead(PHOTO_PIN);
        delay(PHOTO_DELAY);
        if (timeout_2-timeout_1>3000);
          break;
      }
      time_difference = float( time_end - time_begin ) ;
      snelheid = float( PHOTO_DISTANCE/time_difference ) ;
      Serial.println(snelheid);
      return snelheid
    }
    timeout_2 = millis();
  }
}
