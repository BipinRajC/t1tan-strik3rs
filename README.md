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

## **_GPS-module fault injection using faulty Quantum gate logic_**

Faulty Quantum Gate Logic: <br>
Quantum gates are the fundamental building blocks of quantum circuits, analogous to classical logic gates in digital circuits. They operate on qubits and perform various operations like superposition and entanglement. Faulty quantum gate logic refers to the intentional introduction of errors into these quantum gates to simulate real-world imperfections.

https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/d2fd18b2-425b-4da8-a25a-f64bd923cae8

## _Code_

```
import serial
import folium
from folium.plugins import MarkerCluster
from IPython.display import display, clear_output
import time
import numpy as np
import random

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

# Define quantum gates
X = np.array([[0, 1], [1, 0]])
I = np.array([[1, 0], [0, 1]])
Z = np.array([[1, 0], [0, -1]])
ZX = np.dot(Z, X)
H =  np.array([[1, 1], [1, -1]]) / np.sqrt(2)
CNOT = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1],
    [0, 0, 1, 0]
])

XI = np.kron(X, I)
II = np.kron(I, I)
ZI = np.kron(Z, I)
ZXI = np.kron(ZX, I)
HI = np.kron(H, I)

# Define 4x4 fault gates
X4 = np.kron(X, I)
Z4 = np.kron(Z, I)
H4 = np.kron(H, I)
I4 = np.kron(I, I)

# Function to randomly inject a fault
def inject_fault(init_state):
    # List of potential fault gates
    fault_gates = [X4, Z4, H4, I4]
    # Randomly select a fault gate
    fault_gate = random.choice(fault_gates)
    # Apply the selected fault gate to the initial state
    return np.dot(fault_gate, init_state)

# Function to encode GPS data
def encode_gps_data(lat, lon):
    binary_lat = ''.join(format(ord(x), '08b') for x in f"{lat:.6f}")
    binary_lon = ''.join(format(ord(x), '08b') for x in f"{lon:.6f}")
    msg = binary_lat + binary_lon
    encode = ''
    for i in range(0, len(msg) - 1, 2):
        bits = msg[i:i+2]
        if bits == '00':
            encode += 'I '
        elif bits == '11':
            encode += 'ZX '
        elif bits == '10':
            encode += 'Z '
        elif bits == '01':
            encode += 'X '
    return encode.strip()

# Function to process encoded message
def process_encoded_message(message):
    messageDigest = message.split(" ")

    digest = ""
    for m in messageDigest:
        init1 = np.array([1, 0, 0, 0])
        init2 = np.array([0, 0, 0, 1])
        if m == "I":
            init1 = np.dot(II, init1)
            init2 = np.dot(II, init2)
        elif m == "X":
            init1 = np.dot(XI, init1)
            init2 = np.dot(XI, init2)
        elif m == "Z":
            init1 = np.dot(ZI, init1)
            init2 = np.dot(ZI, init2)
        elif m == "ZX":
            init1 = np.dot(ZXI, init1)
            init2 = np.dot(ZXI, init2)
        else:
            print(f"Unknown Op: {m}")

        # Inject fault randomly with a 30% probability
        if random.random() < 0.3:  # 30% chance to inject a fault
            # Fault injection for the first state vector
            init1 = inject_fault(init1)
            # Fault injection for the second state vector
            init2 = inject_fault(init2)

        dig1 = np.dot(CNOT, init1)
        dig2 = np.dot(CNOT, init2)
        dig1 = np.dot(HI, dig1)
        dig2 = np.dot(HI, dig2)
        dig = dig1 + dig2
        if dig[0] != 0:
            digest += "00"
        elif dig[1] != 0:
            digest += "01"
        elif dig[2] != 0:
            digest += "10"
        elif dig[3] != 0:
            digest += "11"

    # Convert the binary digest to a string
    n = int(digest, 2)
    decoded_message = n.to_bytes((n.bit_length() + 7) // 8, 'big').decode('utf-8', errors='ignore')
    return decoded_message

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

                            # Encode and process GPS data
                            encoded_message = encode_gps_data(lat, lon)
                            print(f"Encoded Message: {encoded_message}")

                            decoded_message = process_encoded_message(encoded_message)
                            print(f"Decoded Message: {decoded_message}")

                            # Update the map with decoded coordinates
                            # Assuming the decoded message is in the format "lat,lon"
                            decoded_lat, decoded_lon = map(float, decoded_message.split(','))
                            update_map(decoded_lat, decoded_lon)
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

## **_Securing faults induced in GPS module_**

Here, we use the quantum communication protocol - [Superdense coding](https://medium.com/geekculture/understanding-superdense-coding-c10b42adecca) in order to encrypt the NMEA strings so that bad actors can't intercept them and decode easily, below given is the decryption script along with working demo

https://github.com/BipinRajC/t1tan-strik3rs/assets/112572356/9520f66d-b617-4e5e-a7ac-515a52534ed7

## _Code_

```
import numpy
import binascii

message = "I Z X I X I X ZX X X I I X X X Z X X X I X I X ZX I Z ZX I I Z ZX I I Z ZX I I Z ZX I X I ZX X I Z ZX I I ZX I X I Z ZX Z I ZX I X I ZX I I I ZX X Z I Z ZX I I Z ZX I I ZX I Z I Z ZX Z I ZX I I I ZX X I I ZX Z X I Z ZX I X I Z ZX I Z ZX I X I I X I ZX I Z X I I X I I Z Z I Z X I X I X ZX X X I I X I X ZX X I X ZX X I I X I ZX I I I ZX Z X I ZX I ZX I ZX X I I ZX X I I ZX I ZX I Z ZX Z I ZX I I I ZX I I I ZX I X I ZX I Z I ZX X X I ZX X X I Z ZX Z I ZX I ZX I ZX X ZX I ZX I I I ZX I X I ZX X ZX X I ZX Z I Z ZX I I ZX I I I ZX X ZX I ZX X ZX I ZX I ZX I ZX I I I Z ZX Z I ZX I I I ZX X X I ZX I Z I ZX Z X I Z ZX I X I X X I Z ZX I I ZX I X I Z ZX I I ZX I I I ZX X X I Z ZX I I Z ZX Z I ZX X I I ZX X X I Z ZX I I ZX X ZX I ZX Z X I ZX I Z I Z ZX Z I ZX I I I Z ZX I I Z ZX I I Z ZX X I ZX Z I I ZX X Z I Z ZX Z I ZX X I I Z ZX I X I ZX X I Z ZX I I Z Z Z I ZX X ZX I ZX I ZX I I Z Z I Z X I X I X ZX X X I I X I X ZX X X I ZX I Z ZX I X I I X I Z ZX I I ZX I ZX I Z ZX I I ZX I ZX I ZX I Z I Z ZX I I ZX I Z I ZX X I I ZX I Z I ZX Z X I Z ZX I I ZX I Z I ZX I ZX I Z ZX I I ZX I Z I ZX X X I Z ZX I I Z ZX I I Z ZX I I Z ZX I I Z ZX I I Z ZX I I Z ZX I I ZX I Z I Z ZX Z I ZX X ZX I ZX X I I Z ZX I I Z ZX Z I ZX X I I ZX X X I Z ZX I I ZX I Z I Z ZX Z I ZX I ZX I ZX I Z I Z Z Z I ZX I I I I Z Z I Z X I X I X ZX X X I I X I X ZX X X I ZX X X X Z I Z ZX I I Z ZX I I ZX I X I Z ZX I I ZX I I I ZX Z X I Z ZX I I ZX I I I ZX I X I Z ZX I I Z ZX I I ZX I ZX I ZX X I I Z ZX I I ZX I X I ZX I I I Z ZX I I ZX I Z I ZX X Z I Z ZX I I ZX I Z I ZX X ZX I Z ZX I I Z ZX I I ZX I X I ZX Z I I Z ZX I I ZX X I I ZX I Z I Z ZX I I ZX I I I ZX I ZX I Z ZX I I ZX I Z I ZX I Z I Z ZX I I ZX I Z I ZX I ZX I Z ZX I I ZX X I I ZX I X I ZX I ZX I ZX I X I ZX X X I Z ZX I I ZX I X I ZX Z I I Z Z Z I ZX X I I ZX X X I I Z Z I Z X I X I X ZX X X I I X I X ZX X X I ZX X X X Z I Z ZX I I ZX I ZX I Z ZX I I Z ZX I I ZX I I I ZX Z X I Z ZX I I ZX I Z I ZX X I I Z ZX I I ZX I Z I ZX Z I I Z ZX I I ZX I X I ZX X I I Z ZX I I ZX I ZX I ZX I I I Z ZX I I ZX I Z I ZX X X I Z ZX I I ZX I X I Z ZX I I ZX I X I ZX Z I I ZX I I I Z ZX I I ZX I ZX I ZX X I I Z ZX I I ZX I Z I ZX Z X I ZX X Z I ZX X Z I Z ZX I I ZX I X I ZX X Z I ZX I Z I Z ZX I I ZX I ZX I ZX X X I Z ZX I I ZX I Z I Z ZX I I ZX I Z I ZX X I I Z ZX I I ZX I Z"
messageDigest = message.split(" ")

X = numpy.array([[0, 1], [1, 0]])
I = numpy.array([[1, 0], [0, 1]])
Z = numpy.array([[1, 0], [0, -1]])
ZX = numpy.dot(Z, X)
H =  numpy.array([[1, 1], [1, -1]])
CNOT = numpy.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1],
    [0, 0, 1, 0]
])

XI = numpy.kron(X, I)
II = numpy.kron(I, I)
ZI = numpy.kron(Z, I)
ZXI = numpy.kron(ZX, I)
HI = numpy.kron(H, I)


digest = ""
for m in messageDigest:
    init1 = numpy.array([1, 0, 0, 0])
    init2 = numpy.array([0, 0, 0, 1])
    if m == "I":
        init1 = numpy.transpose(numpy.dot(init1,II))
        init2 = numpy.transpose(numpy.dot(init2,II))
    elif m == "X":
        init1 = numpy.transpose(numpy.dot(init1,XI))
        init2 = numpy.transpose(numpy.dot(init2,XI))
    elif m == "Z":
        init1 = numpy.transpose(numpy.dot(init1,ZI))
        init2 = numpy.transpose(numpy.dot(init2,ZI))
    elif m == "ZX":
        init1 = numpy.transpose(numpy.dot(init1,ZXI))
        init2 = numpy.transpose(numpy.dot(init2,ZXI))
    else:
        print("Unknown Op: {0}".format(m))

    dig1 = numpy.transpose(numpy.dot( init1,CNOT))
    dig2 = numpy.transpose(numpy.dot(init2, CNOT))
    dig1 = numpy.transpose(numpy.dot(dig1,HI))
    dig2 = numpy.transpose(numpy.dot(dig2,HI))
    dig = dig1 + dig2
    if dig[0] != 0:
        digest += "00"
    elif dig[1] != 0:
        digest += "01"
    elif dig[2] != 0:
        digest += "10"
    elif dig[3] != 0:
        digest += "11"
n = int('0b'+digest,2)
print(n.to_bytes((n.bit_length() + 7) // 8, 'big').decode('utf-8', errors='ignore'))
```

## **_Conclusion_** <br>
This project involved a multifaceted approach to understanding and mitigating system vulnerabilities through fault injection techniques. Working with the VSD Squadron Mini and a pulse sensor, we explored how faults can be introduced and detected in embedded systems. By injecting faults and analyzing their impacts, we gained valuable insights into the resilience and reliability of these systems.

The experience underscored the importance of robust design and error detection mechanisms in ensuring system integrity. This project also highlighted the critical role of fault tolerance in maintaining system performance and security, whether in simple embedded systems or more complex computing environments.

Through this project, we enhanced our practical skills in hardware interfacing, fault injection, and system security. The knowledge gained will be instrumental in developing more secure and reliable systems in future endeavors.


**_Team Members_** - _Bipin Raj C_ , _B Jnyanadeep_ <br>
**_College_** - _RV College of Engineering_ 




