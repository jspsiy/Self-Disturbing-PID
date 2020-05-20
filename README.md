# Self-Disturbing-PID

#Components:
- 1x potentiometer
- 2x photoresistor
- 2x Red LED

## Description
- This is a project of how disturbance work in PID controllers where two LED outputs are being controlled at the same time conflicts with each other.

- the tunable parameters are inside the ino file kp1,kp2,kd1,kd2,ki1,ki2


## Libraries needed
- PID_v1.h

## Usage 
- Verify and upload to an arduino board in a dark and enclosed environment, maybe a non reflective/ transparent box.

- You may also use the Serial plotter to view the PID behavior.

## Connections:
##To be improved <will provide visual schametic upon request but i believe you'll figure it out on your own based on the code>

- photoresistor1 and photoresistor 2 conencted at A0 and A2 to ground respectively
- potentiometer voltage in pins connected at voltage pin in, and ground pin connected at ground, connect analog pin to A1.
- LED1(+) and LED2 (+) pin connected at digital pin 9 and 8 and the (-) pins are connected to ground.
