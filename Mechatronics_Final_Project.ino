
//Includes
#include <Wire.h>
#include <PID_v1.h>
#include <ServoEasing.hpp>

//Ultrasonic sensor pins:
#define echoPin 2
#define trigPin 11

//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

 //Variables for accelerometer
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

//Final angle variables:
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;


//PID Tuning variables
double setPoint = 180;
double Kp = 52; //Proportional gain
double Kd = 1.8; //Derivative gain
double Ki = 99; //Integral gain

double input, output; //Initialization variables (location used below)

//Ultrasonic sensor variables
float dist;
long duration;
int distance;
int d;


//Initializing the PID algorithm
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

//Motor Pins
const int MotorPin1 = 9;
const int MotorPin2 = 6;
const int MotorPin3 = 5;
const int MotorPin4 = 3;

//Servos
ServoEasing pan;
//ServoEasing tilt;
int xpos = 90; //initial position of pan servo
int tiltPos = 90; //initial position of tilt servo 

void setup() {
  //Start I2C communication
  Wire.begin();
  //Sets up the MPU-6050 using the function below.                                                     
  setup(); 
  
  //Read the raw acc and gyro data from the MPU-6050 1000 times in order to prepare for actual use. (Gets offset)                                      
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
  
  // Start Serial Monitor                                                 
  Serial.begin(115200);

  //Sets the easing for the two servos. 
  pan.setEasingType(EASE_QUARTIC_IN_OUT);
  tilt.setEasingType(EASE_CUBIC_IN_OUT);
  //Attaches the servos to the corresponding pins. 
  //Note that only one servo can be used with ultrasonic sensor. 
  pan.attach(10);
  tilt.attach(11);
  //Eases to the initial x position (set to 0) in increments of 20 degrees. 
  pan.easeTo(xpos, 20);

  //Sets up the pin modes for the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, OUTPUT);

}

void loop() {
  
  // Get data from MPU-6050
  readData();

  //Uses the function below to balance the robot based on the IMU readings. 
  balancebot(angle_pitch_output);   
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  
  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;  
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians                                
  angle_roll += gyro_y * 0.0000611; 
                                     
  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  //Accelerometer angle calculations
  
  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      (Using the pitch angle in this case)
  //angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  //angle_roll_acc -= 0.0;                                               

  if(set_gyro_angles){ 
  
  //If the IMU has been running 
  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle                      
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; 
    //Correct the drift of the gyro roll angle with the accelerometer roll angle    
    //angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{ 
    //IMU has just started  
    //Set the gyro pitch angle equal to the accelerometer pitch angle                                                           
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle                                       
    //angle_roll = angle_roll_acc;
    //Set the IMU started flag                                       
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  //Take 90% of the output roll value and add 10% of the raw roll value 
 // angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1; 
  delay(50);
  
  // Print to Serial Monitor for debugging  
  Serial.print(" | Angle  = "); Serial.println(angle_pitch_output);

  //Uncomment for ultrasonic sensor use. 
  //Calculates the distance to the nearest object. 
//        d = getDistance();
        //If the object is close,
//      if(d < 20) {
          //Stop moving, move backwards, look left and right, turn, and start moving again
//        halt();
//        delay(100);
//        backward();
//        delay(150);
//        halt();
//        delay(100);
//        panRight();
//        int distanceRight = getDistance();
//        delay(100);
//        int distanceLeft = getDistance();
//        panLeft();
//        delay(300);
//        if(d >= distanceLeft) {
//          right();
//          halt();
//        }
//        else {
//          left();
//          halt();
//        }
//      }


}

void setup(){

  //Activate the MPU-6050
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
                                              
}


void readData(){ 

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);

  //Gets accelerometer and gyroscope data
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}

//Turns the robot right
void right() {
  digitalWrite(MotorPin1, HIGH);
  digitalWrite(MotorPin2, LOW);
  digitalWrite(MotorPin3, LOW);
  digitalWrite(MotorPin4, HIGH);
}

//Turns the robot left
void left() {
  digitalWrite(MotorPin1, LOW);
  digitalWrite(MotorPin2, HIGH);
  digitalWrite(MotorPin3, HIGH);
  digitalWrite(MotorPin4, LOW);
}

//Moves the robot forwards
void forward() {
  digitalWrite(MotorPin1, HIGH);
  digitalWrite(MotorPin2, LOW);
  digitalWrite(MotorPin3, HIGH);
  digitalWrite(MotorPin4, LOW);
}

//Moves the robot backwards
void backward() {
  digitalWrite(MotorPin1, LOW);
  digitalWrite(MotorPin2, HIGH);
  digitalWrite(MotorPin3, LOW);
  digitalWrite(MotorPin4, HIGH);
}

//Stops the robot
void halt() {
  digitalWrite(MotorPin1, LOW);
  digitalWrite(MotorPin2, LOW);
  digitalWrite(MotorPin3, LOW);
  digitalWrite(MotorPin4, LOW);
}

//Function to dynamically balance the robot given some drift angle
void balancebot(double angle){
  //Computes the pid values. 
  pid.Compute();
  //If the angle is greater than or less than |30|, the robot is considered falling. These values can be changed for different performance. 
  if(angle > 30 || angle < -30) {
    //The robot is falling. 
    if(angle > 30) {
      //The robot is falling forwards. 
      forward();
    }
    else if(angle < -30) {
      //The robot is falling backwards. 
      backward();
    }
  }
  else {
    //The robot is not falling. 
    halt();
  }  
}

//Gets the distance to the closest object using the ultrasonic sensor
int getDistance() {
  digitalWrite(trigPin, LOW);
  delay(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = min(duration*0.017, 250);
  return distance;
}

//Turns the head back and forth
void lookAround() {
  pan.easeTo(180, 30);
  delay(15);
  pan.easeTo(0, 30);
  delay(15);
  pan.easeTo(90, 20);
}

//Turns the head right
void panRight() {
  pan.easeTo(180);
}
//Turns the head left
void panLeft() {
  pan.easeTo(0);
}
