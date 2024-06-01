# t1tan-strik3rs
The Soldier Health Monitoring and Position Tracking System allows the military personnel to track the current GPS position of a soldier and also checks the health status by detecting heartbeat of a soldier in realtime.


## **_Soldiers Health Monitoring and GPS Tracking System_**

#### **Introduction**

Modern warfare demands real-time solutions for soldier safety. Soldiers risk injury and getting lost during missions. Delayed medical attention or search efforts can be deadly and can jeopardize national security. This project proposes a soldier health and location tracking system. Sensors monitor vitals (heart rate, temperature) and location (GPS). Wireless communication transmits data to a Blynk application accessed on the PC. This unit tracks soldier location and health, triggering alerts for abnormal readings. This system can significantly improve soldier safety by ensuring faster medical response for injured personnel and reducing search times for lost soldiers. However, further considerations include data security, low-power sensor technology, and system scalability for large-scale deployment.

#### **Overview**

The Smart Soldier Gear project, integrated with VSD Squadron technology, enhances soldier safety and operational efficiency through advanced wearable systems. The gear includes a pulse sensor (RC-A-4015) to monitor vital health parameters continuously. Any deviations from predefined thresholds trigger emergency alerts. A GPS module tracks the soldier’s real-time location. Data from these sensors are processed by a microcontroller and transmitted wirelessly via the HC12 RF transceiver directly to the Blynk cloud using the Blynk application. This setup enables real-time data visualization on a customized Blynk dashboard, ensuring continuous health and location monitoring and facilitating immediate responses in emergencies. This system significantly improves the situational awareness and operational readiness of soldiers.

#### **Components Required**

- VSD Squadron Mini developement board with CH32V003F4U6 chip with 32-bit RISC-V core based on RV32EC instruction set
- RC-A-4015 Pulse Sensor
- Neo-6M GPS Receiver
- Bread Board
- Jumper Wires

#### **Software Required**

- Python (libraries like folium, gmplot) - [folium](https://pypi.org/project/folium/), [gmplot](https://pypi.org/project/gmplot/) 
- PlatformIO - download from [Here](https://platformio.org/platformio-ide)


#### **Circuit Connection Diagram**

![ckt-diagram](https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/b6ac5f6f-426d-4c1e-9256-e7cd57292ebc)


#### **Table for Pin Connection**

| RC-A-4015 Pulse Sensor | VSD Squadron Mini |
| ---------------------- | ----------------- |
| GND                    | GND               |
| VCC                    | 5V                |
| A0                     | PA1               |

| Neo-6M GPS Receiver | VSD Squadron Mini |
| ------------------- | ----------------- |
| GND                 | GND               |
| VCC                 | 5V                |
| TX                  | PD6 (RX)          |
| RX                  | PD5 (TX)          |

**_Team Members_** - _Bipin Raj C_ , _B Jnyanadeep_ <br>
**_College_** - _RV College of Engineering_ 

## Code

```cpp
#include <ch32v10x_gpio.h>
#include <stdio.h>
#include <stdint.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define PULSE_SENSOR_PIN PA1
#define GPS_RX_PIN PA3
#define GPS_TX_PIN PA2

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX

void delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++); // Adjust this value for your clock speed
}

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable clock for GPIO port A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure pulse sensor pin as input
    GPIO_InitStructure.GPIO_Pin = PULSE_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog input mode
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable clock for GPIO ports for GPS module
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void setup() {
    // Initialize serial communication for debugging
    printf("Pulse Sensor Reading:\n");

    // Initialize GPIO pins
    GPIO_Config();

    // Initialize GPS module
    gpsSerial.begin(9600); // Change baud rate to match your GPS module
}

uint16_t custom_analogRead(uint8_t pin) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_RegularChannelConfig(ADC1, pin, 1, ADC_SampleTime_57Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

void loop() {
    // Read pulse sensor value
    uint16_t pulseValue = custom_analogRead(PULSE_SENSOR_PIN);

    // Print pulse sensor value to console
    printf("Pulse Sensor Value: %d\n", pulseValue);

    // Read GPS data
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                // Display GPS location
                printf("Latitude: %.6f, Longitude: %.6f\n", gps.location.lat(), gps.location.lng());
            }
        }
    }

    // Delay for 1 second
    delay_ms(1000);
}




