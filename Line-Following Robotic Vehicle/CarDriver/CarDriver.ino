//IR Emitters//
#define EMITEVEN    61
#define EMITODD     45
#define REFL0       65
#define REFL1       48
#define REFL2       64
#define REFL3       47
#define REFL4       52
#define REFL5       68
#define REFL6       53
#define REFL7       69

// Motor Control //
#define LEFT_NSLP   31  // HIGH to wake up
#define LEFT_DIR    29  // HIGH - Forward
#define LEFT_PWM    40  // HIGH - Speed
#define RIGHT_NSLP  11
#define RIGHT_DIR   30
#define RIGHT_PWM   39

// Time Constants //
#define RC_CHARGE 5
#define RC_DISCHARGE 775

const int receivers[8] = { REFL0, REFL1, REFL2, REFL3,
                           REFL4, REFL5, REFL6, REFL7 };

const double K_P = 3.2;
const double K_D = 1.2;

double speed_L = 250;
double speed_R = 250;

bool turnedAlready = false;
const int SAMPLESIZE = 10;

class queue {
  public:
    queue() {
      for(int i = 0; i < SAMPLESIZE; i++) {
        array[i] = 0;
      }
    }
    double pushpop(double newVal) {
      double temp = array[pointer];
      array[pointer] = newVal;
      if(++pointer == 10)
        pointer = 0;
      return temp;
    }
  private:
    double array[SAMPLESIZE];
    int pointer = 0;
};

void sampleIR(int readings[8]) {
  for(int i = 0; i < 8; i++) {
    pinMode(receivers[i], OUTPUT);
    digitalWrite(receivers[i], HIGH);
  }
  
  delayMicroseconds(RC_CHARGE);

  for(int i = 0; i < 8; i++) {
    pinMode(receivers[i], INPUT);
  }

  delayMicroseconds(RC_DISCHARGE);
  
  for(int i = 0; i < 8; i++)  // 1 means black detected
    readings[7-i] = digitalRead(receivers[i]);
//  dump(readings);
}

void dump(int readings[8]) {
  for(int i = 0; i < 8; i++) {
    Serial.print(readings[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void brake() {
  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(LEFT_DIR, HIGH);
  analogWrite(LEFT_PWM, 0);
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(RIGHT_PWM, 0);
}

void turn(int dutycycle) {
  int turnSpeed = (dutycycle*255)/1000;
  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(LEFT_DIR, HIGH);
  analogWrite(LEFT_PWM, turnSpeed);
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(RIGHT_PWM, turnSpeed);
  delay(265);
}

void drive(int dutycycle, bool isRight) { // out of 1000
  int driveSpeed = (dutycycle*255)/1000;
  if(isRight) {
    digitalWrite(RIGHT_DIR, LOW);
    analogWrite(RIGHT_PWM, driveSpeed);
  } else {
    digitalWrite(LEFT_DIR, LOW);
    analogWrite(LEFT_PWM, driveSpeed);
  }

}

void setup() {
  // Initialize IR Pins
  pinMode(EMITEVEN, OUTPUT);
  pinMode(EMITODD, OUTPUT);
  digitalWrite(EMITEVEN, HIGH);
  digitalWrite(EMITODD, HIGH);
  
  pinMode(REFL0, INPUT);
  pinMode(REFL1, INPUT);
  pinMode(REFL2, INPUT);
  pinMode(REFL3, INPUT);
  pinMode(REFL4, INPUT);
  pinMode(REFL5, INPUT);
  pinMode(REFL6, INPUT);
  pinMode(REFL7, INPUT);

  // Initialize Motor Pins
  pinMode(LEFT_NSLP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(RIGHT_NSLP, HIGH);
  
//  Serial.begin(9600);
//  Serial.print("begin");
//  Serial.print(REFL0);
  delay(1000);
}

queue pastErrors;
int readings[8];

void loop() {

  drive(speed_R, true);
  drive(speed_L, false);   
  
  sampleIR(readings);
  double error_Horiz = // positive value = need to turn right
       -9 * readings[1] - 
       4 * readings[2] -  1 * readings[3] + 
       1 * readings[4] +  4 * readings[5] + 
       9 * readings[6]; 

  if(readings[5] == 0 && readings[6] == 1) 
    error_Horiz += 12;
  if(readings[2] == 0 && readings[1] == 1) 
    error_Horiz -= 12;

  double brakeCheck = readings[0] + readings[1] +
                   readings[2] + readings[3] +
                   readings[4] + readings[5] +
                   readings[6] + readings[7];
                   
  double errorDiff = error_Horiz - pastErrors.pushpop(error_Horiz);
 
  if(brakeCheck >= 8) {
    if(!turnedAlready) {
      turnedAlready = true;
      brake();
      turn(1000);
      speed_R = 125;
      speed_L = 125;
      drive(speed_R, true);
      drive(speed_L, false); 
      while(1) {
        sampleIR(readings);
        if(readings[3] == 1 && readings[4] == 1)
          break;
      }
      drive(100, true);
      drive(100, false); 
      delay(300);

      // stop
    } else {
      while(1) {
          drive(0, true);
          drive(0, false); 
      }
    }
  }
  else {
   speed_R -= error_Horiz * K_P;
   speed_R -= errorDiff * K_D;
   speed_L += error_Horiz * K_P; 
   speed_L += errorDiff * K_D; 
  }

  if(error_Horiz > 0) {
    speed_R -= error_Horiz * K_P;
  }
  else if (error_Horiz < 0) {
    speed_L += error_Horiz * K_P; 
  }

  if(errorDiff > 0) {
    speed_L += errorDiff * K_D; 
  }
  else if(errorDiff < 0) {
    speed_R -= errorDiff * K_D;
  }
  
  if(error_Horiz <= 0) {
    speed_R = 400;
    speed_L = 400;
  }

  speed_R = constrain(speed_R, 100, 450);
  speed_L = constrain(speed_L, 100, 450);  
            
}
