# t1tan-strik3rs
The Soldier Health Monitoring and Position Tracking System allows the military personnel to track the current GPS position of a soldier and also checks the health status by detecting heartbeat of a soldier in realtime.


## **_Soldiers Health Monitoring and GPS Tracking System_**

#### **Introduction**

Modern warfare demands real-time solutions for soldier safety. Soldiers risk injury and getting lost during missions. Delayed medical attention or search efforts can be deadly and can jeopardize national security. This project proposes a soldier health and location tracking system. Sensors monitor vitals (heart rate, temperature) and location (GPS). Wireless communication transmits data to a Blynk application accessed on the PC. This unit tracks soldier location and health, triggering alerts for abnormal readings. This system can significantly improve soldier safety by ensuring faster medical response for injured personnel and reducing search times for lost soldiers. However, further considerations include data security, low-power sensor technology, and system scalability for large-scale deployment.

#### **Overview**

The Smart Soldier Gear project, integrated with VSD Squadron technology, enhances soldier safety and operational efficiency through advanced wearable systems. The gear includes a pulse sensor (RC-A-4015) to monitor vital health parameters continuously. Any deviations from predefined thresholds trigger emergency alerts. A GPS module tracks the soldier’s real-time location. Data from these sensors are processed by a microcontroller and transmitted using pyplot. This setup enables real-time data visualization on a map , ensuring continuous location monitoring and facilitating immediate responses in emergencies. This system significantly improves the situational awareness and operational readiness of soldiers.

#### **Components Required**

- VSD Squadron Mini developement board with CH32V003F4U6 chip with 32-bit RISC-V core based on RV32EC instruction set
- RC-A-4015 Pulse Sensor
- Neo-6M GPS Receiver
- Bread Board
- Jumper Wires

#### **Software Required**

- Python (libraries like folium, gmplot) - [folium](https://pypi.org/project/folium/), [gmplot](https://pypi.org/project/gmplot/) 
- Arduino IDE - download from [Here](https://www.arduino.cc/en/software)


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

# Demo

This is the working of our project:

https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/3a7db16f-2dcf-4336-b647-8024336c2ba6

_Advantages: Keeping in mind the ultimate goal of this hackathon, that is fault injection, we believe that this could see it's applications in military wherein soldiers using such monitoring systems can keep the central unit updated about their positions and health vitals and in case of other countries' intelligence agencies trying to interfere with such systems, our aim is to pentest it and find a viable solution to safeguard such systems and keep our defences non-vulnerable and safe._

![image](https://github.com/BipinRajC/t1tan-strik3rs/assets/132117873/f3788fa8-e94d-4b7b-a886-6d6b66aef672) <br>

![image](https://github.com/BipinRajC/t1tan-strik3rs/assets/132117873/ed821786-9597-4420-9c9d-ce5a36fc4b29) <br>



## **_Python code for interfacing gps module and its working_**

```py
import serial
import folium
from folium.plugins import MarkerCluster
from IPython.display import display, clear_output
import time

# Initialize map centered at a default location (e.g., New York City)
map_center = [12 + 50/60 + 41.6/3600, 77 + 39/60 + 51.0/3600]  # 12.84489, 77.66417
mymap = folium.Map(location=map_center, zoom_start=12)

# Create a marker cluster for better performance with many markers
marker_cluster = MarkerCluster().add_to(mymap)

# Function to update map with new GPS coordinates
def update_map(lat, lon):
    folium.Marker([lat, lon]).add_to(marker_cluster)
    mymap.save('gps_map.html')
    # Display the updated map in Jupyter Notebook
    clear_output(wait=True)
    display(mymap._repr_html_())

# Try to initialize the serial connection with Arduino
try:
    ser = serial.Serial('COM3', 9600)  # Adjust 'COM3' based on your Arduino's port
except serial.SerialException as e:
    print(f"Could not open serial port: {e}")
    ser = None

if ser:
    # Main loop to continuously read GPS data and update map
    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode().strip()
                print(f"Received data: {data}")  # Debugging output
                if data.startswith("$GPGGA"):
                    gps_data = data.split(',')
                    if len(gps_data) >= 10 and gps_data[2] and gps_data[4]:  # Check if latitude and longitude fields are not empty
                        try:
                            # Extract latitude and longitude
                            lat_deg = int(gps_data[2][:2])
                            lat_min = float(gps_data[2][2:])
                            lat = lat_deg + (lat_min / 60.0)
                            if gps_data[3] == 'S':
                                lat = -lat

                            lon_deg = int(gps_data[4][:3])
                            lon_min = float(gps_data[4][3:])
                            lon = lon_deg + (lon_min / 60.0)
                            if gps_data[5] == 'W':
                                lon = -lon

                            print(f"Latitude: {lat}, Longitude: {lon}")
                            update_map(lat, lon)
                        except ValueError as e:
                            print(f"Error parsing GPS data: {e}")
                    else:
                        print("Incomplete GPS data received.")
            time.sleep(1)  # Adjust delay as needed to control the update frequency
    except KeyboardInterrupt:
        print("Mapping stopped by user.")
    finally:
        ser.close()
        print("Serial connection closed.")

    # Save the map to an HTML file one last time when the script ends
    mymap.save('gps_map.html')
    print("Map saved as 'gps_map.html'.")
else:
    print("Serial connection was not established. Exiting the program.")
```
## **_Arduino code for pulse sensor interfacing and its working_**

```
// Define the pin where the pulse sensor is connected
const int pulsePin = A0;

// Variable to store the sensor value
int sensorValue = 0;

void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(9600);
}

void loop() {
  // Read the value from the pulse sensor
  sensorValue = analogRead(pulsePin);

  // Print the sensor value to the Serial Monitor and Serial Plotter
  Serial.println(sensorValue);

  // Wait for a short period before reading the value again
  delay(1000); // Adjust the delay as needed
}
```

## **_Fault injection in Pulse Sensor_**

Voltage glitching exploit involves momentarily disrupting the power supply to the microcontroller, causing it to behave unpredictably. 

Explanation: <br>
In  every 10 seconds (millis() % 10000 < 100 condition), a voltage glitch is simulated by setting sensorValue to its maximum possible value (1023 in Arduino's analogread scale).
This simulates a scenario where the microcontroller might experience a transient voltage spike or power disturbance.

##### **_Code for Pulse sensor Fault injection_**

```cpp
int sensorValue=0;
int pulsePin=A0;

void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(9600);
}

void loop() {
  // Simulate voltage glitch every 10 seconds
  if (millis() % 10000 < 100) { // Inject glitch for 100 milliseconds every 10 seconds
    // Simulate voltage glitch by resetting sensorValue to a high value
    sensorValue = 1023; // Max value glitch
  } else {
    // Read the value from the pulse sensor
    sensorValue = analogRead(pulsePin);
  }

  // Print the sensor value to the Serial Monitor and Serial Plotter
  Serial.println(sensorValue);

  // Wait for a short period before reading the value again
  delay(1000); // Adjust the delay as needed
}
```
## **_Demo of Fault injection in Pulse sensor_**

https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/3f373802-433c-4dff-91ea-8ed855ea724b

## **_Securing the Fault in Pulse Sensor_**

The alpha parameter in the EMA filter 
(sensorValue Filtered = alpha * sensorValueRaw + (1 - alpha) * sensorValueFiltered;) 
- The controlling input of the exponential smoothing calculation is defined as the smoothing factor or the smoothing constant.
- Exponential functions are used to assign exponentially decreasing weights over time. 
- Fine-tune alpha based on the expected dynamics of your sensor readings and the severity of voltage faults encountered in your setup. <br>

![image](https://github.com/BipinRajC/t1tan-strik3rs/assets/132117873/e47e66fe-75c5-4a36-88ab-8950a099895a)
<br>

https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/491fcf72-5c7d-4a03-9229-204c9aa7d82a

```cpp
// Define constants for EMA filter
const float alpha = 0.2;  // Smoothing factor (adjust as needed, between 0 and 1)
float sensorValueFiltered = 0;  // Initial filtered sensor value

// Define the pin where the pulse sensor is connected
const int pulsePin = A0;

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
}

void loop() {
  // Read the value from the pulse sensor
  int sensorValueRaw = analogRead(pulsePin);

  // Apply EMA filter
  sensorValueFiltered = alpha * sensorValueRaw + (1 - alpha) * sensorValueFiltered;

  // Print the filtered sensor value to the Serial Monitor and Serial Plotter
  Serial.println(sensorValueFiltered);

  // Wait for a short period before reading the value again
  delay(1000); // Adjust the delay as needed
}
```




**_Team Members_** - _Bipin Raj C_ , _B Jnyanadeep_ <br>
**_College_** - _RV College of Engineering_ 




