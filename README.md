🚗 Obstacle Avoiding Robot (ESP8266)
This project is a smart Obstacle Avoiding Robot built using an ESP8266, Ultrasonic Sensor, BO Motors, and a Servo Motor.
The robot detects obstacles, scans the surroundings, and automatically chooses the safe direction to move.
It also supports a web-based controller, allowing manual driving and a draw-mode path system.
🛠️ Features
✔ Automatic Obstacle Avoidance
✔ Ultrasonic distance detection
✔ Servo-based front scanning
✔ Manual control using a web interface
✔ Left / Right turning, rotation, and brake controls
✔ Draw Mode — draw a path and the robot follows it
✔ ESP8266 Wi-Fi AP Mode
✔ LED indicators
✔ Non-blocking servo scanning
📦 Components Used
Component
Purpose
ESP8266 NodeMCU
Main controller & Wi-Fi server
Ultrasonic Sensor (HC-SR04)
Detects obstacles in front
4× BO Motors
Robot movement
4× Wheels
Support motion
Motor Driver (MX1508 / L298N)
Drives motors with PWM
Servo Motor
Rotates ultrasonic sensor for scanning
Battery Pack
Powers the entire robot
Jumper Wires
Circuit connections
⚙️ How the Robot Works
The ultrasonic sensor reads distance in centimeters.
If the distance is safe, the robot moves forward automatically.
If an obstacle is detected:
Robot reverses
Performs a scan left → front → right
Chooses the direction with maximum clearance
Robot continues moving safely without human control.
🌐 Web Interface
The robot creates its own Wi-Fi hotspot:
Copy code
 
IP: .........
From the webpage, you can:
Drive manually (Forward / Back / Left / Right / Stop)
Rotate 360°
Control LEDs
Perform scanning
Use Draw Mode to send a custom path
📂 Project Structure
Copy code

NovaX_Final.ino → Main firmware file
index.html      → Web UI (embedded in PROGMEM)
Everything is written inside a single .ino file for easy flashing.
🚀 Applications
Smart home robots
Warehouse navigation
Industrial automation
Obstacle avoidance research
Autonomous delivery robots
Robotics education & IoT training
🎥 Demo Video Script (Short)
“This is our obstacle avoiding robot. It automatically scans for obstacles and chooses the safest path. It uses an ultrasonic sensor, servo motor, BO motors, and ESP8266. This robot can be used for smart homes, warehouses, automation, delivery robots, and robotics training.”
