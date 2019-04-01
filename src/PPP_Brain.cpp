#include <PWMSoft.h>
#include <SoftPWM_timer.h>
#include <AltSoftSerial.h>

AltSoftSerial bluetoothSerial; // de bluetoothconnectie (noem dit hoe je wil maar niet Serial, dat is de USB connectie). RX op pin 9 via spanningsdeler, TX op pin 8
void hardwareSetup();
void servoSet(int servoPin, int servoAngle);


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
int PWM_S2_HOME = 0;
int PWM_S3_HOME = 0;

int SERVOMIN = 8;
int SERVOMAX = 37;

int DIFFICULTY = 0;
int RELOAD_SPEED = 10;

int PHOTO_THRESHOLD = 850;  // triggerwaarde voor script
int PHOTO_DISTANCE = 0.05;  // afstand tussen phototransistoren
int PHOTO_DELAY = 0.1;  // delays voor stabiliteit

// defenities voor placeholder variables
unsigned long time_begin, time_end;
float snelheid, time_difference;
String command;  // opslag voor bluetoothbericht


void setup() {
  Serial.begin(9600);
  Serial.print("Program started running!");

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
    if (command.startsWith("SETDIFFICULTY/"))
    {}
  }


  // De Gsm stuurt een waarde van 0 tot 7 naar de bluetooth module




  switch (DIFFICULTY)
  {
    case 0:
    //tijd tussen ballen is het langst
      Serial.println("makkelijk");
      RELOAD_SPEED = 1;

    case 1:
      Serial.println("normaal");
      RELOAD_SPEED = 2;

    case 2:
      Serial.println("moeilijk");
      RELOAD_SPEED = 3;

    case 3:
      Serial.println("extreem");
      // tijd tussen ballen is het kortst
      RELOAD_SPEED = 4;

    case 4 :
      // links voor

      break;
    case 5 :
      // links achter

      break;
    case 6 :
      // rechts voor

      break;
    case 7 :
      // rechts achter

      break;

    }
}


void hardwareSetup()
{
  Serial.print("Starting with hardware preperation ...");
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



void servoSet(int servoPin, int servoAngle)
{
  int valuePWM = map(servoAngle, -60, 60, 8, 37);
  SoftPWMSet(servoPin, valuePWM);
}


void speed()
{
  // Meet de snelheid
  int photoVal = analogRead(PHOTO_PIN);

  if (int(photoVal) > PHOTO_THRESHOLD)
  {
    time_begin = millis();
    while (int(photoVal) > PHOTO_THRESHOLD)
    {
      photoVal = analogRead(PHOTO_PIN);
      delay(PHOTO_DELAY);
    }
    while (int(photoVal) < PHOTO_THRESHOLD)
    {
      photoVal = analogRead(PHOTO_PIN);
      delay(PHOTO_DELAY);
    }
    time_end = millis();
    while (int(photoVal) > PHOTO_THRESHOLD)
    {
      photoVal = analogRead(PHOTO_PIN);
      delay(PHOTO_DELAY);
    }
    time_difference = float( time_end - time_begin ) ;
    snelheid = float( PHOTO_DISTANCE/time_difference ) ;
    Serial.println(snelheid);
  }
}
