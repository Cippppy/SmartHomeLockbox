# Smart Home Lockbox
The Smart Home Lockbox is a project created by Christian Cipolletta and Matt Gerace for their final in Intro To Embedded Systems. It is a tool that will allow people to lock their important items. It utilizes Ultrasonic, temperature, CO2, humidity, pressure, and vibration sensors to be aware of external stimuli. The microcontroller, the MSP430FR2355, takes in those inputs and alerts the user to danger using buzzers, LEDs, and an internet webpage. A servo also locks the box if someone is detected.

## Power Supply
* Barrel Jack [For AC/DC Wall Plug]
* 2x 20mm Coin Cell Batteries + Holders
* 2x Linear Regulators, 1 for 5V, 1 for 3.3V

## Sensors
* BME280 Temperature, Humidity, Pressure Sensor Breakout Board [I2C]
* MQ-9B (Sen-17050) CO2 Sensor [Analog]
* MSP430FR2355 On Board Temperature Sensor [Analog]
* HC-SR04 Ultrasonic Sensor [Digital]
* MPU6050 Accelerometer and Gyroscope [I2C]

## Output Peripherals
* Red LED
* Blue LED
* Active Buzzer
* ESP01 Wifi Module [[ThingSpeak]](https://thingspeak.com/channels/2104523)
* Servo

## Where is the project at?
The onboard temperature sensor and ultrasonic sensors are confirmed to be working. The CO2 gives an ouput, but we do not know if it is correct. The batteries are not strong enough to run the circuit, so it is wall power right now. The I2C code doesn't work so the BME280 and MPU6050 aren't working now either. The servos, LEDs, buzzers, and ESP01 all work.

## Project Video
![Smart Home Lockbox Demo](https://user-images.githubusercontent.com/98924381/235828658-0aa0f5fc-a45d-4658-aacc-fe4c0e6af6ec.MOV)
