
#define IR_pin A1
#define motorA_pin_en 5
#define motorB_pin_en 6

#define motorA_encoder_pin 2
#define motorB_encoder_pin 3

#define in1_pin 12
#define in2_pin 11
#define in3_pin 10
#define in4_pin 9

#define Rtrig_pin 4
#define Recho_pin 7
#define Ltrig_pin 13
#define Lecho_pin 8

#define RIR_pin A2
#define LIR_pin A0

int distanceL;
int distanceR;
volatile int encoderACounter = 0;
volatile int encoderBCounter = 0;


double leftSpeed = 83;
double rightSpeed = 80;

bool flag = false;

unsigned long prevTime, currentTime;
unsigned long counter = 0;
int interval = 100;

void setup()
{

  pinMode(motorA_pin_en, OUTPUT);
  pinMode(motorB_pin_en, OUTPUT);

  pinMode(in1_pin, OUTPUT);
  pinMode(in2_pin, OUTPUT);
  pinMode(in3_pin, OUTPUT);
  pinMode(in4_pin, OUTPUT);

  pinMode(motorA_encoder_pin, INPUT);//Read from Encoder PINS __|¯¯|__|¯¯|__|¯¯|__|¯¯|
  pinMode(motorB_encoder_pin, INPUT);

  //Ultrasonics sensors
  pinMode(Rtrig_pin, OUTPUT);
  pinMode(Ltrig_pin, OUTPUT);
  pinMode(Recho_pin, INPUT);
  pinMode(Lecho_pin, INPUT);

  ////////// TO MAKE SURE MOTORS ARE OFF INITIALY////////////
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, LOW);
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, LOW);
  ///////////////////////////////////////////////////////////

  attachInterrupt(digitalPinToInterrupt(motorA_encoder_pin), motorARevCount, RISING);
  //attachInterrupt(digitalPinToInterrupt(motorB_encoder_pin), motorBRevCount, RISING);


  prevTime = millis();
  Serial.begin(9600);
}


void loop()
{
  if (millis() - prevTime >= interval) {
    if(flag){
        encoderACounter = 0;
        while(encoderACounter < 200){
          MoveForward();
        }
        Stop();
      flag = false;
    } else{
      if (analogRead(IR_pin) > 512 && analogRead(LIR_pin) < 512 ) // if there's a wall on left and right move forward
        MoveForwardConstant();
      else if (analogRead(LIR_pin) > 512){
        encoderACounter = 0;
        while(encoderACounter < 70){
          MoveForward();
        }

        Stop();
          encoderACounter = 0;
          while(encoderACounter < 111){
          NightyDegreesLeft();
          }
        Stop();
        flag = true;
      } else if (analogRead(LIR_pin) < 512 && analogRead(IR_pin) < 512 && analogRead(RIR_pin) > 512){
        encoderACounter = 0;
        while(encoderACounter < 10){
          MoveForward();
        }

        Stop();
          encoderACounter = 0;
          while(encoderACounter < 55){
          NightyDegreesRight();
          }
        Stop();
        flag = true;
      }else if(analogRead(LIR_pin) < 512 && analogRead(IR_pin) < 512 && analogRead(RIR_pin) < 512){
        Stop();
          encoderACounter = 0;
          while(encoderACounter < 230){
          NightyDegreesLeft();
          }
        Stop();
        flag = true;
      }    
    }
    prevTime = millis();
  } 

}



void MoveForward() {

  //Coding the HBridge for Moving Forward
  //MOTORA
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  //MOTORB
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  ErrorCorrection();
    //Giving a speed to the motors
  analogWrite(motorA_pin_en, rightSpeed);
  analogWrite(motorB_pin_en, leftSpeed);
}

void MoveForwardConstant() {

  //Coding the HBridge for Moving Forward
  //MOTORA
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  //MOTORB
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  ErrorCorrection();
    //Giving a speed to the motors
  analogWrite(motorA_pin_en, rightSpeed);
  analogWrite(motorB_pin_en, leftSpeed);
  counter = counter +1;
  if (counter == 10) {
    Stop();
    counter = 0;
}
}


void Stop(){

  analogWrite(motorA_pin_en, 0);
  analogWrite(motorB_pin_en, 0);
  delay(50);

}

void ErrorCorrection() {
  //Reading from right Ultra
  digitalWrite(Rtrig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Rtrig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Rtrig_pin, LOW);

  double durationR = pulseIn(Recho_pin, HIGH);
  distanceR = (durationR * .0343) / 2;
  //Reading from LEFT Ultra
  digitalWrite(Ltrig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Ltrig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ltrig_pin, LOW);

  double durationL = pulseIn(Lecho_pin, HIGH);
  distanceL = (durationL * .0343) / 2;
  //MotorA      MotorB
  int error = distanceR - distanceL;// EX R=9  L=12 9-12 = -3

  Serial.println(error);

if (analogRead(LIR_pin) > 512 || analogRead(RIR_pin) > 512){
    rightSpeed = 80 ; leftSpeed = 83; // 137 90
}
else{
  if (error > 5) {
    rightSpeed = 75 ; leftSpeed = 98;
  }
  if(error < -5){
    rightSpeed = 95 ; leftSpeed = 83;    
  }
  if (error == 3 || error == 4 || error == 5){
    rightSpeed= 81; leftSpeed= 92;
  }
  if (error == -3 || error == -4 || error == -5){
    rightSpeed= 90; leftSpeed= 85;
  }
  if(error == 0 || error == 1 || error == 2 || error == -2 || error == -1){
    rightSpeed = 80 ; leftSpeed = 83; // 137 90
  }
}
}

void NightyDegreesLeft() {
  //This Function is to move 90 degrees to the left
  //MOTORA
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  //MOTORB
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);

  //Move for 90 deg
  analogWrite(motorA_pin_en, 80);
  analogWrite(motorB_pin_en, 83);
}

void NightyDegreesRight() {
  //This Function is to move 90 degrees to the left
  //MOTORA
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  //MOTORB
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);

  //Move for 90 deg
  analogWrite(motorA_pin_en, 80);
  analogWrite(motorB_pin_en, 83);
}


void motorARevCount() {
  encoderACounter++;
}


void motorBRevCount() {
  encoderBCounter++;
}


