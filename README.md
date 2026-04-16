This package is designed to control and test a dynamic impedance ankle that has a Dynamixel servo, quadrature rotary encoder, and force resistive sensor using a STM32 microcontroller. It also contains test data from measuring the stiffness of the linear spring component and the entire ankle assembly in an Instron machine.

ankle_control
This section of the package is responsible for controlling the ankle assembly.

encoder.c: Provides functions to enable the quadrature encoder with a STM32, read the position and velocity of the encoder, and write this information to UART.
force_sensor.c: Provides functions that enable reading ADC from the Tiva's GPIO pins, read the ADC value and convert it to units of force, and write this information over UART.
servo.c: Provides functions to enable UART for writing to a Dynamixel servo using Protocol 2.0, enable writing a goal position, enable reading a current position, toggle the built-in LED, and toggle between Rx and Tx modes.
controller.c: Contains the main loop that, when loaded onto a stm32, continuously reads the force sensor and encoder data to write goal positions to the servo between steps.


This repo contain we have done the in the "Development and validation of the ankle prosthestic leg" . We have develop the Ak60-motor library and control the ankle motion .
The design of our system is given below:![assembly image](https://github.com/user-attachments/assets/5b660cee-9816-4001-9d1e-1da177350b90)
The hardware overview of our system is :
![HardwareoverviewArtboard 10](https://github.com/user-attachments/assets/274d57de-450e-4941-bc93-979bf55568a3)
This is the routing of the circuit.
![image](https://github.com/user-attachments/assets/ce00abb4-a6a8-4103-92dd-b807c1b12391)

This is the explode view of our leg desing
![exploded_view](https://github.com/user-attachments/assets/c3bbbeee-4ae7-49cc-bdea-6c5eb7eb5985)

This is walking tracking of the real human 

![walking](https://github.com/user-attachments/assets/96e04ca8-15ed-4424-8111-9e3d8b4e9166)

And here is the gait animation of ankle robotics leg we able to follow.

![gait_animation](https://github.com/user-attachments/assets/d835901e-075b-4244-8f12-6aa7c9b1546b)
