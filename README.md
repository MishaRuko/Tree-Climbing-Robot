# Soviet Cruiser Tree Climbing Robot
The legenday tree climbing robot.

<img width="450" alt="Soviet_cruiser_cad" src="https://github.com/MishaRuko/Tree-Climbing-Robot/assets/51261953/12b1a9f0-a3aa-4c49-a5b0-459988fa11a3">

## Usage
Operating the robot is simple, the steps are as follows:
1. Flash the Arduino with the appropriate code depending on the desired mode of operation (i.e. with/without branch avoidance capability)
2. Ensure there are no disconnected wires
3. Ensure that the jumpers on the motor shield are set to use the power pin as the power source
4. Attach a 12V power source that can supply up to 2A of current to the motor shield power pins, ensuring correct polarity
5. When ready to climb, connect power to the Arduino and stand clear; after a short delay, the robot will begin to climb
6. The robot will climb to the top of the tree, deploy the sensor, and return to the bottom of the tree fully autonomously
7. Disconnect the power from the robot to stop it

## Implemented Algorithms
### MotorControl.ino
This code makes the robot go straight up until it sees the platform 25cm above it at which point it will gradually slow down and reverse the direction of the wheels, initiating the descent process.
The flowchart for this code is:

<img width="323" alt="Low_level_control" src="https://github.com/MishaRuko/Tree-Climbing-Robot/assets/51261953/bc87e72c-a201-4630-86b0-cb9686e8f468">

### SovietCruiserControl.ino
This code makes the robot detect branches and uses a PID controller to rotate the robot so that the branches pass through the gap between the gripper arms.
The flowchart for this code is:

<img width="389" alt="PID_flow_diagram" src="https://github.com/MishaRuko/Tree-Climbing-Robot/assets/51261953/0df59edb-411a-4a66-b060-b627a18f3371">

