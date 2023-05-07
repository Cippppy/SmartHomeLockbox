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

## Where is the project at? (5/7/2023)
The onboard temperature sensor and ultrasonic sensors are confirmed to be working. The CO2 gives an ouput, but we do not know if it is correct. The batteries are not strong enough to run the circuit, so it is wall power right now. The I2C code doesn't work so the BME280 and MPU6050 aren't working now either. The servos, LEDs, buzzers, and ESP01 all work. The 3D printed box that will be the actual one that locks was successfully printed and locked by the servo. However, it was unable to be covered because it is too small to fit the full PCB at this time. You can read more on our [project slides](https://docs.google.com/presentation/d/1oNxZCWa-71zp4ZQAXbW0OE4L5BJcd05L6jm5VQH0Wrw/edit?usp=sharing) or our [project report](https://docs.google.com/document/d/1pnApUV6Cqu8zDdYJzm2F5OI0f5RzCBrWJ35Rd4LK8bY/edit?usp=sharing).

## Future Work
The next steps to take to make the project a full success start with remaking the PCB. First, the ESP01 needs to switch its trace. Second, the servo header needs to be retraced as well. Last, there needs to be a new battery choosen to the power the device. On the software side, the main fix is making sure the I2C connections are fixed. Once that is fixed, to make this device low-power it would be a benefit to make the code interrupt based instead of the current polling based system. The final thing to fix would be to make the lock box big enough to encase the PCB and all parts while still being able to be locked and to store items.

## Project Video
https://user-images.githubusercontent.com/98924381/235829279-1a4e9000-3827-43de-8596-dcd38e9c36f0.mp4

# Appendix
## Resources & Datasheets
* BME280 [Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout.pdf), [Arduino Github](https://github.com/adafruit/Adafruit_BME280_Library)
* MPU6050 [Datasheet](https://www.haoyuelectronics.com/Attachment/GY-521/mpu6050.pdf)
* Ultrasonic [Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
* MQ-9B [Datasheet](https://cdn.sparkfun.com/assets/d/f/5/e/2/MQ-9B_Ver1.4__-_Manual.pdf), [Wiring Example](http://wiring.org.co/learning/basics/airqualitymq135.html)
* Servo [Datasheet](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)
* ESP01 [Datasheet](https://www.universal-solder.ca/downloads/esp8266_series_modules_user_manual_en.pdf)
* Updating 3D Model of the Lockbox [OnShape](https://cad.onshape.com/documents/b93b4b42872b4098d45294de/w/dbbdbcc2a9ef1ffa882faf1d/e/f71d34ea460787590961a1fb?renderMode=0&uiState=64553cd0d9ecec32a6cac62c)



