# AgileX-PiPER-MetworkMQTT
MetaworkMQTT protocol based PiPER controller

To control AgileX-Piper with MQTT: 
1. Activate can bus: bash can_activate.sh can0 1000000
2. Start Piper_sdk_ui
   In Piper SDK Tools
   (1) Reset
   (2) Enable
   (3) Go Zero
   If the robot not go zero, try several times
3. Set robot to work position. Run code "piper_work_position_initialize.py"
4. Memo the UUID from viewer. Open viewer on browser, press F12, check "USER_UUID"
5. Copy the UUID and change USER_UUID in "MQTT_Recv.py"
6. Run robot controller "MQTT_Robot_Feedback_PD.py" with PD control, ""MQTT_Robot_Feedback_PD_Traj.py" with PD + Trajectory Plan,
   "MQTT_Robot_Control.py" send signal directly
