#include <PWMSoft.h>
#include <SoftPWM_timer.h>
#include <AltSoftSerial.h>

AltSoftSerial bluetoothSerial; // de bluetoothconnectie (noem dit hoe je wil maar niet Serial, dat is de USB connectie). RX op pin 9 via spanningsdeler, TX op pin 8

// pin defenities voor digitale pinnen
int PWM-M3&4_PIN = 2;
int PWM-S1_PIN = 3;
int DIRECTION-SWITCH_PIN = 4;
int PWM-S2_PIN = 5;
int PWM-M2_PIN = 6;
int LED1_PIN = 7;
int TX_PIN = 8;
int RX_PIN = 9;
int PWM-M1_PIN = 10;
int PWM-S3_PIN = 11;

// pin defenities voor analoge pinnen
int PWM-M5&6_PIN = A0;
int RED-IN_PIN = A1;
int LED2_PIN = A2;
int BLUE-IN_PIN = A3;
int PHOTO_PIN = A4;
int GREEN-IN_PIN = A5;

// definities voor standaardwaarden
int PWM-S1_HOME = 0;
int PWM-S2_HOME = 0;
int PWM-S3_HOME = 0;



void setup() {
  Serial.begin(9600);
  Serial.print("Program started running!");

  delay(1000);

  // configuring all settings for connected hardware
  hardwareSetup();



}

void loop() {
  // put your main code here, to run repeatedly:

}


void hardwareSetup()
{
  Serial.print("Starting with preperation ...")
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
  digitalWrite(RED-IN_PIN, LOW);
  digitalWrite(BLUE-IN_PIN, LOW);
  digitalWrite(GREEN-IN_PIN, LOW);
  delay(10);

  Serial.println("        Homing all servo's ...");
  servoSet(PWM-S1_PIN, PWM-S1_HOME);
  servoSet(PWM-S2_PIN, PWM-S2_HOME);
  servoSet(PWM-S3_PIN, PWM-S3_HOME);
  delay(10);

  Serial.println("    Preperation Done!");
}



void servoSet(servoPin, angle)
{
  valuePWM = map(angle, -60, 60, 8, 37)
  SoftPWMSet(servoPin, valuePWM
}
