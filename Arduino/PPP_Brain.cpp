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

// snelheidssensor
int Treshold = 850;
int Distance = 0.05;
unsigned long time_begin, time_end;
float Snelheid, Timedifference;



void setup() {
  Serial.begin(9600);
  Serial.print("Program started running!");

  delay(1000);

  // configuring all settings for connected hardware
  hardwareSetup();



}

void loop() {
  // put your main code here, to run repeatedly:
  // De Gsm stuurt een waarde van 0 tot 7 naar de bluetooth module

  switch (DIFFICULTY)
  {
    case 0:
    //tijd tussen ballen is het langst
      Serial.println("makkelijk");
      RELOAD_SPEED = 1

      break ;
    case 1:
      Serial.println("normaal");
      RELOAD_SPEED = 2

      break;

    case 2:
      Serial.println("moeilijk");
      RELOAD_SPEED = 3

      break;
    case 3:
      Serial.println("extreem");
      // tijd tussen ballen is het kortst
      RELOAD_SPEED = 4
      break;

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
  Serial.print("Starting with preperation ...");
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
  // Meet de snelheid
  Val = analogRead(PHOTO_PIN);

  if (int(Val) > Treshold)
  {
    time_begin = millis();
    while (int(Val) > Treshold)
    {
      val = analogRead(PHOTO_PIN);
      delay(0.1);
    }
    while (int(Val) < Treshold)
    {
      Val = analogRead(PHOTO_PIN);
      delay(0.1);
    }
    time_end = millis();
    while (int(Val) > Treshold)
    {
      Val = analogRead(PHOTO_PIN);
      delay(0.1);
    }
    Timedifference = float((time_end - time_begin)) ;
    Snelheid = float( Distance/Timedifference) ;
    Serial.println(Snelheid);

}
