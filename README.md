# MetaworkMQTT Protocol-Based PiPER Controller

## 🚀 Quick Start

### 🧩 Step 1: Run the MQTT Controller
```bash
cd ./src/app
npm run dev-https
```

💡 If this is your first time running the project, you need to install the required Node.js modules first:
```bash
cd ./src/app
npm install
```

🌐 After the server is running, you can access the VR Viewer in your browser by appending /viewer to the server address. For example:
```arduion
https://192.168.197.37:3000/viewer/
```
This interface is used to visualize and send VR controller data via MQTT.

📊 You can also access the status monitor via the following URL:
```arduion
https://sora2.uclab.jp/menroll
```
This page provides a real-time interface for observing user information.

###  🧩 Step 2: Run PiPER Controller
Follow the steps below to control the **AgileX-PiPER** robot via MQTT:

1. **Activate the CAN bus**
   Execute the following command in your terminal:
   ```bash
   cd ./AgileX-PiPER-MetworkMQTT
   bash can_activate.sh can0 1000000
   ```

2. **Start the PiPER SDK UI**
   
   (check "agilex_pipier_connect.mkv" in the video folder )
   
   Download the PiPER SDK UI
   
   ```arduion
   https://github.com/agilexrobotics/Piper_sdk_ui.git
   ```

   Open the PiPER SDK Tools and perform the following operations:

      (1) Click Reset
      
      (2) Click Enable
      
      (3) Click Go Zero
      
   🔁 If the robot fails to go to the zero position, repeat this step a few times until successful.

5. **Set the robot to the working position**
   Run the following script:
   ```bash
   python piper_work_position_initialize.py
   ```

6. **Retrieve your USER_UUID from the Viewer**
   Open the Viewer in your browser: https://<your-server-address>/viewer
   
   Press F12 to open Developer Tools
   
   Look for the USER_UUID in the console or network tab and copy it
   
7. 
   
8. Activate can bus: bash can_activate.sh can0 1000000
9. Start Piper_sdk_ui
   In Piper SDK Tools
   (1) Reset
   (2) Enable
   (3) Go Zero
   If the robot can not go zero, try several times
10. Set the robot to work position. Run code "piper_work_position_initialize.py"
11. Memo the UUID from the viewer. Open the viewer on the browser, press F12, and check "USER_UUID"
12. Copy the UUID and change USER_UUID in "MQTT_Recv.py"
13. Run robot controller "MQTT_Robot_Feedback_PD.py" with PD control, ""MQTT_Robot_Feedback_PD_Traj.py" with PD + Trajectory Plan,
   "MQTT_Robot_Control.py" send the control signal directly

   
The IK is based on Modern Robotics. 

Source code: https://github.com/NxRLab/ModernRobotics
