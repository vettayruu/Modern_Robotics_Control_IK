The IK is based on Modern Robotics. 

Source code: https://github.com/NxRLab/ModernRobotics

2025/6/30:

Create branch "Piper_MQTT_Control". 

Robot AgileX-Piper now can be controlled by VR through MQTT.

Firstly, to start MQTT communication, run the code:

npm run dev-https

To open the viewer, add "/viewer" after the address such as

https://192.168.197.37:3000/viewer/

Then set the robot connection.

Then， to control AgileX-Piper with MQTT: 
1. Activate can bus: bash can_activate.sh can0 1000000
2. Start Piper_sdk_ui
   In Piper SDK Tools
   (1) Reset
   (2) Enable
   (3) Go Zero
   If the robot can not go zero, try several times
3. Set the robot to work position. Run code "piper_work_position_initialize.py"
4. Memo the UUID from the viewer. Open the viewer on the browser, press F12, and check "USER_UUID"
5. Copy the UUID and change USER_UUID in "MQTT_Recv.py"
6. Run robot controller "MQTT_Robot_Feedback_PD.py" with PD control, ""MQTT_Robot_Feedback_PD_Traj.py" with PD + Trajectory Plan,
   "MQTT_Robot_Control.py" send the control signal directly
