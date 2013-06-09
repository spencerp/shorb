/* 
=====Gyro Setup:=====
VDD - Arduino 3.3v (Brown)
GND - Arduino GND (Red)
INT - Arduino digital pin 2 (Blue)
FSYNC - leave unconnected
SCL - Arduino SCL (dedicated pin or Analog 5) (Yellow)
SDA - Arduino SDA (dedicated pin or Analog 4) (Green)
VIO - Arduino 3.3v (Orange)
CLK - leave unconnected
ASCL - leave unconnected
ASDA - leave unconnected



=====IR Setup:=====
White - A0
Red - 5v
Black - GND



====RX Setup:====
CH1 - 12 (Bank)
CH2 - 11 (Forward)
CH3 - 10 (Initialize iff>1050)...iff SWB flipped... Also Gyro speed.
CH4 - 9 (Manual control iff>1470) In Autonomous, VRA *Be sure to keep below half. SWA flipped in for Manual
CH5 - 8  VRB
CH6 - BLANK
***On furthest right pin***
***ASWA flipped forward induced differential***
BAT - 5v/GND


====Servo/Motor Setup:=====
***White Wires to Control Input on Arduino
Fore Servo - 6
Aft Servo - 5
Fore Motor - 3
Aft Motor - 3
***Red HIGH, Black GND
*/


#include "Wire.h"

//Servo Library
#include <Servo.h>

// PID Control Library
#include <PID_v1.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
MPU6050 mpu;


// Filter
#include <SignalFilter.h>

SignalFilter RXFilter1; // Ch1
SignalFilter RXFilter2; // Ch2
//SignalFilter RXFilter3;
//SignalFilter RXFilter4;
SignalFilter IRFilter;


// Output x,y,z Quaternions from Calculation on DMP
#define OUTPUT_READABLE_YAWPITCHROLL

// Accel compensated for orientation
//#define OUTPUT_READABLE_WORLDACCEL

// Outpute relative acceleration according to Sensor x,y,z
#define OUTPUT_READABLE_REALACCEL



#define LED_PIN 13
bool blinkState = false;


// Setting up bluetooth communication
#include <GraphSeries.h>


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// PID vars
double BankReference, BankInput, BankOutput;
double AccelReference, AccelInput, AccelOutput;
//Specify the links and initial tuning parameters
PID Bank(&BankInput, &BankOutput, &BankReference,1,0,.15, DIRECT); //Set Kp, Ki,Kd here
PID Accel(&AccelInput, &AccelOutput, &AccelReference,1,0,.15, DIRECT);  //** Actually BANK!!!!!
int AccelX; // Linear acceleration in forward direction.
double yaw, pitch, roll;


// Servo Vars
Servo ForeServo;
Servo AftServo;
int Servoval1;
int Servoval2;
Servo Motors;



// parameters
int MaxAccelDeg = 25; // Defines Maximum acceleration angle of pendulum gyros (Between 0 and 45
int MaxBankDeg = 25;
int MassTot = 5; //Kg
int MassPend = 1; //Kg
int MaxAccel;

// IR Sensor vars
double IRdist;
double IRhist;
double IRvel;
int c1=0;
double sum=0;
double dt=1000;
double dx;


// RX/TX vars
int ch1 = 1000;
int ch2 = 1000;
int ch3 = 1020;
int ch4 = 1500;
int ch5;
int ch6;
int GyroSpeed = 0;


// Ramp time variables
int t1 = 0;
float t2 = 0;
float t3 = 0;

// Run-time variables
int c = 0;
float temp;
int filterAvg1[50];
int filterAvg2[50];

String printt;
String printtt;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready (Commented is Serial Debugging)
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());       // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/ 
    Serial.println(F("\nFlip SWB Switch to Begin Initialization"));
    while(pulseIn(10, HIGH)<1050){
      Serial.println(pulseIn(10, HIGH));
    }            //(Initialize iff>1300)...iff SWB flipped
    //while (Serial.available() && Serial.read());

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // initialize the variables we're linked to
    BankReference = 0;
    BankInput = ypr[3]; // Chooses Roll Direction
    AccelReference = 0;
    AccelInput = ypr[2]; // Chooses Pitch Direction

    // turn the PID on
    Bank.SetMode(AUTOMATIC);
    Accel.SetMode(AUTOMATIC);
    Bank.SetOutputLimits(-90, 90);
    Accel.SetOutputLimits(-90, 90);
    
    // initialize RX
    pinMode(12, INPUT);
    pinMode(11, INPUT);
    pinMode(10, INPUT);
    pinMode(9, INPUT);
    pinMode(8, INPUT);
    pinMode(3, OUTPUT);
    //pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    //bluetooth data transfer
    pinMode(0,INPUT);
    pinMode(1,OUTPUT);
   
    
    // Initialize gyro control servos
    ForeServo.attach(6);
    AftServo.attach(5);
    ForeServo.write(90); // Sets to midpoint
    AftServo.write(90);
    Motors.attach(3);
    Motors.write(0);
    
    // Initialize Filters
    RXFilter1.begin();
    RXFilter1.setFilter('m');
    RXFilter1.setOrder(2);
    RXFilter2.begin();
    RXFilter2.setFilter('m');
    RXFilter2.setOrder(2);
    /*RXFilter3.begin();
    RXFilter3.setFilter('m');
    RXFilter3.setOrder(2);
    Filter.begin();
    Filter.setFilter('b');
    Filter.setOrder(1);
    Filter.begin();
    Filter.setFilter('b');
    Filter.setOrder(1);
    */
    IRFilter.begin();
    IRFilter.setFilter('m');
    IRFilter.setOrder(2);
    
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    analogWrite(1,100);
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // .
        // Check non-vital channels every 25 comps
        if(c>25){
          temp=ch3;
          ch3 = pulseIn(10, HIGH, 10000); //(Initialize iff>1300)...iff SWB flipped.. **and gyro speed
          if(ch3==0){
            ch3=temp;}
            
          /*temp=ch4;
          ch4 = pulseIn(9, HIGH, 10000); // Manual iff>1490
          if(ch4==0){
            ch4=temp;}*/
        }
        else{
          c++;
        }
        
        // Check Soft Reset Trigger
  /*      if(ch3<1010 && ch3!=0){
          digitalWrite(LED_PIN,LOW);
          while(GyroSpeed>50){
            GyroSpeed=GyroSpeed-1;
             analogWrite(3,GyroSpeed); 
             analogWrite(4,GyroSpeed);
             delay(100);
          }
          softReset();    //SoftReset
        }
        
        */
        
        
        // Average the distances
        if(c1>5){
          IRdist=sum/4;
          IRdist = 65*pow(IRdist, -1.10);
          // These values from own curve fit. Will have to be adjusted for reflectivity of ball.
          c1=0;
          sum=0;
          dt = (millis() - t2);
          dx = (IRdist-IRhist);
          IRvel=-1*(dx)*1000/(dt); // cm/s
          IRhist=IRdist;
          t2 = millis();
        }
        else{
          temp=(double)analogRead(A0)*0.0048828125;
          sum=sum+temp;
          c1=c1+1;
        }
       
        
        
        
        //if(ch4>1450){
        // ======================================================
        // ===           MANUAL CONTROL ALGORITHMS            ===
        // ======================================================
        // Collect RX Signals
        temp=ch1;
        ch1 = pulseIn(12, HIGH, 10000); //bank
        if(ch1==0){
          ch1=temp;}
        temp=ch2;
        ch2 = pulseIn(11, HIGH, 10000); //forward
        if(ch2==0){
          ch2=temp;}

        //Apply Filters
       // ch1 = SMA(ch1,filterAvg1,50);//Simple Moving Average
       // ch2 = SMA(ch2,filterAvg2,50);
        ch1=RXFilter1.run(ch1);
        ch2=RXFilter2.run(ch2);
        

       // Serial.println(ch1);

        
        AccelReference = map(ch1,1000,2000,-5*MaxBankDeg,5*MaxBankDeg); // Map channel for Bank angle
        BankReference = map(ch2,1000,2000,-5*MaxAccelDeg,5*MaxAccelDeg); // Map channel for Accel rate
        
        
        /* Autonomous Override of Manual Control
        if(IRdist<20 | (millis()-t3)<2000){ //Delay 2 seconds
          AccelReference = (int) -MaxAccelDeg/2;
          BankReference = (int) MaxBankDeg;
          if(IRdist<20){t3 = millis();}
        }
        */
        
        // Aaron:
        //***Find way to reinitialize gyro/accel (reset zeros)
        //***Can use Ch5 (VRB) to change PID vars... Setup for PID val.switch
        
        //}
        
        
        
        
        
        
        
        
        
        
        /* else{  // If Ch4 (SwitchA) is flipped ***VRA must be lower than half (left).
        // ==========================================================
        // ===           AUTONOMOUS CONTROL ALGORITHMS            ===
        // ==========================================================
        
        // **JOHAN**
        // **SPENCER**
        // **NOAH**
        
        
        // Can use ch4 = VRA knob once in autonomous mode
        // Can use ch5 = VRB knob all the time
        
        //Variable to work with: IRdist, IRvel, roll, pitch, yaw
        //Define BankReference,[-MaxBankDeg,MaxBankDeg] and AccelReference,[-MaxAccelDeg,MaxAccelDeg] using autonomous algorithms.
        
        
        Serial.println("Autonomous");
        
        
        //Create front end in "Processing" to display collected data from Bluetooth Serial Extension
        
        /*Determine acceleration using MassTot and MassPend using MaxAccel (In setup)
        use conditional v=srqt(2*as*d) to determine eminent collision
        set some factor of safety, f
        
        
        } */
        
        
        
        
        
        
        
        
        
        
        
        
        
        
 
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        // ==============================================================
        // ===           CONTROL ALGORITHMS AND MAP TO SERVOS         ===
        // ==============================================================
        
        //====Gyro Speed Control=====
        temp = map(ch3,1200,2000,35,180);
        
        // Nice Code to Ramp Up Speeds
        if(GyroSpeed==temp){
          t1=millis();
        }
        else if(GyroSpeed>temp){
          GyroSpeed=temp;
          t1=millis();
        }
        else if((millis()-t1)>10){
          GyroSpeed = GyroSpeed++; // Increment Speed by 1000/1000/255=1/255 per second
          t1=millis();
        }
        
        //analogWrite(3,GyroSpeed); // Write to both gyro motors via esc
        //analogWrite(4,GyroSpeed); // Write to both gyro motors  **Pin 4 not PWM
        Motors.write(GyroSpeed); // Control with this rather than PWM above...
        
        //Serial.println(ch3);
        
        //=====Bank PID=====
        BankInput = -roll; // -90 to 90
        Bank.Compute(); // Between -90 and 90
        
        
        //=====Accel PID====
        //AccelX = sqrt(aaReal.x^2 + aaReal.y^2); //Change x/y/z based on mounting.
        //This is the linear acceleration in forward direction
        AccelInput = -pitch;
        Accel.Compute();
        

        
        //=====Differencing Gyro Servo Output====== Differencing and Coalescing BankOutput and AccelOutput
        // Differencing function: IN(accel(-90to90), bank(-90to90)) : OUT(Diff[(0-180),(0-180)])
        //int Servoval1 = coalesce(AccelOutput, BankOutput,0);
        //int Servoval2 = coalesce(AccelOutput, BankOutput,1);
        
        
        Servoval1=AccelOutput/2+BankOutput;
        Servoval2=-AccelOutput/2+BankOutput;
        // ** Between -135 and 135!
        
        Servoval1=(Servoval1+135)*120/270;
        Servoval2=(Servoval2+135)*120/270;
        // Between 0 and 160
        
        
        
        
        
        // Protect Servos
        if(Servoval1>120){
          Servoval1=120;
        }
        else if(Servoval1<0){
          Servoval1=0;
        }
        if(Servoval2>120){
          Servoval2=120;
        }
        else if(Servoval2<0){
          Servoval2=0;
        }
        
        
        ForeServo.write(Servoval1); // Between 0 and 180 *160
        AftServo.write(Servoval2); // Between 0 and 180 (Centered at 90)* 160 and 80
        
        /*char buffer[20];
        IRdist=(int)IRdist;
        roll=(int)roll;
        Serial.println(IRdist);
        Serial.println(roll);
        sprintf(buffer,"%i,%i",roll,IRdist);
        Serial.println(buffer);
        IRdist=(long)IRdist;
        roll=(long)roll;
        printt = String(roll,DEC);
        printtt = String(IRdist,DEC);
        printt = String(printt + printtt);
        Serial.println(printt);*/
        
        
        String toprint = "";
        toprint += (int)IRdist;  
         toprint += ',';
        toprint += (int)roll;
        
        Serial.println(toprint);
        

        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .
        // .

        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw=ypr[0] * 180/M_PI;
            pitch=ypr[1] * 180/M_PI;
            roll=ypr[2] * 180/M_PI;
        #endif
        
        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            /*Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);*/
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}







// Subfunctions

/*void softReset(){
  asm volatile ("  jmp 0");
}

int coalesce(int a, int b, int index){  // Differencing a and b and returns diff1 and diff2 as the two inputs
  int Differenced[1];
  Differenced[0] = a + b;   // Between -180 and 180???
  Differenced[1] = -a + b;
  if(abs(Differenced[0])>60){  //Check rails
    if(Differenced[0]>0){
      Differenced[1]=Differenced[1]-(Differenced[0]-60);
      Differenced[0]=60;
    }
    else{
      Differenced[1]=Differenced[1]-(Differenced[0]+60);  // -(????
      Differenced[0]=-60;
    }
  }
  else if(abs(Differenced[1])>60){  //Check rails
    if(Differenced[1]>0){
      Differenced[0]=Differenced[0]-(Differenced[1]-60);
      Differenced[1]=60;
    }
    else{
      Differenced[0]=Differenced[0]-(Differenced[1]+60);
      Differenced[1]=-60;
    }
  }
  Differenced[0]=90+Differenced[0]/2;
  Differenced[1]=90+Differenced[1]/2; // So 45 contribution from each
  return Differenced[index];
}
//======== Simple Moving Average Filter =========//    
int SMA(int ch, int a[], int num_elements)
{
   int i, avg, sum=0;
   for (i=0; i<(num_elements-1); i++) {
     a[i] = a[i+1];
   }
   a[num_elements-1] = ch;
   
   for (i=0; i<num_elements; i++)
   {
	 sum = sum + a[i];
   }
   
   avg = sum/num_elements;
   
   return(avg);
}


void ramp(int valuefrom, int valto){
  if(valfrom==valto){}
  else(
}


*/
