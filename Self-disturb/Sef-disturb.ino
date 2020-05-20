#include <PID_v1.h>
const int photores = A0;// LDR input pin
const int photores2 = A2;// LDR input pin
const int pot = A1;        // Potentiometer input pin
const int led = 9;         // LED output pin
const int led2 = 8;         // LED output pin
double lightLevel;         // Indirectly store the light level
double lightLevel2;         // Indirectly store the light level
// Tuning parameters
float Kp = 0;              // Proportional gain
float Ki = 0.2;             // Integral gain
float Kd =0;
float Kp2 = 1;              // Proportional gain
float Ki2 = 0.5;             // Integral gain
float Kd2 =0;
// Differential gain
// Record the set point as well as the controller input and output
double Setpoint, Input, Output,Input2,Output2;
// Create a controller that is linked to the specified Input, Ouput and Setpoint
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint, Kp2, Ki2, Kd2, DIRECT);
const int sampleRate = 1;        // Time interval of the PID control
const long serialPing = 500;     // How often data is recieved from the Arduino
unsigned long now = 0;           // Store the time that has elapsed
unsigned long lastMessage = 0;   // The last time that data was recieved
 
void setup()
 {
 lightLevel = analogRead(photores);                 // Read the set point
 // Arduino has an analogueRead() resolution of 0-1023 and an analogueWrite() resolution of 0-255
 Input = map(lightLevel, 0, 1023, 0, 255);        // Scale the input
 Setpoint = map(analogRead(pot), 0, 1023, 0, 255);  // Scale the set point
 Serial.begin(9600);                                // Initialise serial communications at 9600 bps
 myPID.SetMode(AUTOMATIC);                          // Turn on the PID control
 myPID.SetSampleTime(sampleRate);                   // Assign the sample rate of the control
  myPID2.SetMode(AUTOMATIC);                          // Turn on the PID control
 myPID2.SetSampleTime(sampleRate); 
 Serial.println("Begin");                           // Let the user know that the set up s complete
 lastMessage = millis();                            // Serial data will be recieved relative to this first point
 }
  
void loop()
 {
 Setpoint = map(analogRead(pot), 0, 1023, 0, 255);  // Continue to read and scale the set point
 lightLevel = analogRead(photores);     // Read the light level
  lightLevel2 = analogRead(photores2);                 // Read the light level
 Input = map(lightLevel, 0, 1023, 0, 255);          // Scale the input to the PID
 Input2 = map(lightLevel2, 0, 1023, 0, 255);          // Scale the input to the PID

 myPID.Compute();                                   // Calculates the PID output at a specified sample time
 analogWrite(led, Output); // Power the LED
 myPID2.Compute();
  analogWrite(led2, Output2); // Power the LED

 now = millis();                                    // Keep track of the elapsed time
 if(now - lastMessage > serialPing)                 // If enough time has passed send data
 {
 
 Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print(Input2);
   Serial.print("\t");
 Serial.println(Input);
    Serial.print("\t");
 //Serial.println(Output2);



 // The tuning parameters can be retrieved by the Arduino from the serial monitor: 0,0.5,0 set Ki to 0.5.
 // Commas are ignored by the Serial.parseFloat() command
 
 lastMessage = now;               // Reference the next serial communication to this point
 }
 }
