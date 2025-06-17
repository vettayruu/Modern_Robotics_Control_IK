"use client";
import 'aframe'
import * as React from 'react'
import RobotScene from './RobotScene';
import registerAframeComponents from './registerAframeComponents'; 

const THREE = window.AFRAME.THREE;
// import 'aframe-troika-text';

import useMqtt from './useMqtt';
import { connectMQTT, mqttclient,idtopic,subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'
import { Coming_Soon } from 'next/font/google';

const order = 'ZYX'; 

const mr = require('../modern_robotics/modern_robotics_core.js');
const RobotKinematics = require('../modern_robotics/modern_robotics_Kinematics.js');
const RobotDynamcis = require('../modern_robotics/modern_robotics_Dynamics.js');

const rk = new RobotDynamcis("piper_agilex");
const M = rk.get_M();
const Mlist = rk.get_Mlist();
const Glist = rk.get_Glist();
const Slist = rk.get_Slist();
const Kplist = rk.get_Kplist(); 
const Kilist = rk.get_Kilist(); 
const Kdlist = rk.get_Kdlist(); 
const jointLimits = rk.jointLimits;
const Blist = mr.SlistToBlist(M, Slist); // Convert Slist to Blist

// 全局常量与变量定义
const MQTT_REQUEST_TOPIC = "mgr/request";
const MQTT_DEVICE_TOPIC = "dev/"+idtopic;
const MQTT_CTRL_TOPIC = "control/"+idtopic; // 自分のIDに制御を送信
const MQTT_ROBOT_STATE_TOPIC = "robot/"; // Viwer のばあい

// React组件主函数
export default function DynamicHome(props) {
  // 当前时间戳（用于动画驱动）
  const [now, setNow] = React.useState(new Date())
  // 是否已渲染主场景
  const [rendered,set_rendered] = React.useState(false)
  // 机器人型号列表与当前型号
  const robotNameList = ["Model"]
  const [robotName,set_robotName] = React.useState(robotNameList[0])
  // 辅助线、测试框可见性
  const [cursor_vis,set_cursor_vis] = React.useState(false)
  const [box_vis,set_box_vis] = React.useState(false)
  // 目标点是否超出可达范围
  const [target_error,set_target_error] = React.useState(false)

  // 当前输出的关节角度数组（用于MQTT/渲染等）
  const rotateRef = React.useRef([0,0,0,0,0,0,0]); // 用于跨渲染帧保存关节角度

  /* VR Controller State Initialize*/
  const [controller_object, set_controller_object] = React.useState(() => {
    const controller_object = new THREE.Object3D();
    return controller_object;
    });

  const [trigger_on,set_trigger_on] = React.useState(false)
  const vrModeRef = React.useRef(false); // VR模式标志

  // 夹爪/按钮状态
  const gripRef = React.useRef(false);
  const gripValueRef = React.useRef(0);

  const [button_a_on, set_button_a_on] = React.useState(false);
  const button_A_Ref = React.useRef(null);
  const [button_b_on, set_button_b_on] = React.useState(false)
  const button_B_Ref = React.useRef(null);


  const [selectedMode, setSelectedMode] = React.useState('control'); // 控制/监控模式
  const robotIDRef = React.useRef("none"); // 当前机器人ID

  // 摄像机位置与朝向
  const [c_pos_x,set_c_pos_x] = React.useState(0.23)
  const [c_pos_y,set_c_pos_y] = React.useState(0.3)
  const [c_pos_z,set_c_pos_z] = React.useState(-0.6)
  const [c_deg_x,set_c_deg_x] = React.useState(0)
  const [c_deg_y,set_c_deg_y] = React.useState(150)
  const [c_deg_z,set_c_deg_z] = React.useState(0)

  const [dsp_message,set_dsp_message] = React.useState("")

  // 工具列表与当前工具
  const toolNameList = ["No tool"]
  const [toolName,set_toolName] = React.useState(toolNameList[0])

  // 动画帧ID
  const reqIdRef = React.useRef()

  // MQTT/状态同步相关
  const receiveStateRef = React.useRef(false);
  const currentRotationRef = React.useRef(new THREE.Euler(0.6654549523360951,0,0,order));
  
  const lastLoopTimeRef = React.useRef(performance.now());
  // 动画主循环，驱动渲染和运动学动画
  const loop = ()=>{
    const now = performance.now();
    const dt = now - lastLoopTimeRef.current;
    lastLoopTimeRef.current = now;
    // console.log('主循环周期(ms):', dt); //mean dt 16.5ms
    setNow(performance.now()); // 更新时间戳，驱动依赖 now 的动画 useEffect
    reqIdRef.current = window.requestAnimationFrame(loop) // 持续递归调用，形成动画循环
  }

  // 初始化动画循环
  React.useEffect(() => {
    loop()
    return () => window.cancelAnimationFrame(reqIdRef.current) // 组件卸载时清理动画帧
  },[])

  // 切换机器人模型
  const robotChange = ()=>{
    const get = (robotName)=>{
      let changeIdx = robotNameList.findIndex((e)=>e===robotName) + 1
      if(changeIdx >= robotNameList.length){
        changeIdx = 0
      }
      return robotNameList[changeIdx]
    }
    set_robotName(get)
  }


  // 机器人请求函数，向MQTT服务器请求机器人信息
  const requestRobot = (mqclient) =>{
        // 制御対象のロボットを探索（表示された時点で実施）
        const requestInfo = {
          devId: idtopic, // 自分のID
          type: codeType,  //  コードタイプ（Request でマッチングに利用)
        }
        console.log("Publish request",requestInfo)
        publishMQTT(MQTT_REQUEST_TOPIC, JSON.stringify(requestInfo));
  }

  // MQTT 相关的 useEffect（连接、订阅、消息处理）
  const mqttclientRef = React.useRef(null);
  useMqtt({
  props,
  set_rendered,
  robotIDRef,
  publishMQTT,
  MQTT_REQUEST_TOPIC, MQTT_DEVICE_TOPIC, MQTT_CTRL_TOPIC, MQTT_ROBOT_STATE_TOPIC,
  receiveStateRef,
  mqttclientRef, //not used
  gripRef,
  button_A_Ref,
  button_B_Ref,
  gripValueRef,
  set_target_error,
  set_dsp_message,
  setNow,
  vrModeRef,
  THREE
  });


  // 持续传输机器人状态
  /**
   * 定时通过 MQTT 发送当前机器人关节状态（用于 Viewer/Monitor 实时同步）
   * @param {number} time - 当前时间戳
   */
  const onAnimationMQTT = (time) =>{
    const robot_state_json = JSON.stringify({
      time: time,
      joints: theta_body.current, // 当前各关节角度
      grip: gripRef.current      // 当前夹爪状态
    });
    publishMQTT(MQTT_ROBOT_STATE_TOPIC+idtopic , robot_state_json); // 发布到机器人状态主题
    window.requestAnimationFrame(onAnimationMQTT); // 持续递归调用，形成定时推送
  }

  /**
   * 在 XR 渲染帧回调中发送控制指令到 MQTT
   * @param {number} time - 当前时间戳
   * @param {XRFrame} frame - XR 帧对象
   */
  const onXRFrameMQTT = (time, frame) => {
    // Viewer 模式下每帧递归调用自身
    if(props.viewer){
      frame.session.requestAnimationFrame(onXRFrameMQTT);
    }else{
      // 仅在 VR 模式下递归调用自身
      if (vrModeRef.current){
        frame.session.requestAnimationFrame(onXRFrameMQTT);
        setNow(performance.now()); // VR模式下需手动更新时间戳，驱动动画
      }
    }

    // 仅在已连接 MQTT 且允许发送状态时推送
    if ((mqttclient != null) && receiveStateRef.current) {
      // 组装并发送控制指令
      const ctl_json = JSON.stringify({
        time: time,
        joints: theta_body.current,
        trigger: [gripRef.current, button_A_Ref.current, button_B_Ref.current, gripValueRef.current]
      });
      publishMQTT(MQTT_CTRL_TOPIC, ctl_json);
    }
  }

  /*** Robot Controller ***/
  // Initial joint and tool angles
  // const theta_body_initial = mr.deg2rad([0, -30, 70, 0, 65, 0]);
  const theta_body_initial = [0, -0.27473, 1.44144, 0, 1.22586, 0];
  const [theta_body, setThetaBody] = React.useState(theta_body_initial);

  const dtheta_body_initial = [0, 0, 0, 0, 0, 0];
  const [dtheta_body, setdThetaBody] = React.useState(dtheta_body_initial);
  

  const theta_tool_inital = 0;
  const [theta_tool, setThetaTool] = React.useState(theta_tool_inital);
  // Theta guess for Newton's method in inverse kinematics
  const [theta_body_guess, setThetaBodyGuess] = React.useState(theta_body);

  /*** Dynamics VR Animation ***/
  const [qHistQueue, setQHistQueue] = React.useState([]);
  const animatingRef = React.useRef(false);

  React.useEffect(() => {
    if (animatingRef.current) return;
    if (qHistQueue.length === 0) return;
    animatingRef.current = true;
    const q_hist = qHistQueue[0];
    let idx = 0;
    function animate() {
      if (!q_hist) return;
      if (idx < q_hist.length) {
        setThetaBody(q_hist[idx]);
        idx += Math.max(1, Math.floor(q_hist.length / 16.5));
        // console.log('setThetaBody:', idx, q_hist.length, q_hist[idx]);
        setTimeout(animate, 16.5); // 可调节速度
      } else {
        setThetaBodyGuess(q_hist[q_hist.length - 1]);
        animatingRef.current = false;
        // 播放完后移除队首，自动播放下一个
        setQHistQueue(queue => queue.slice(1));
      }
    }
    animate();
  }, [qHistQueue]);

  // Foward Kinematics solution
  const T0 = mr.FKinBody(M, Blist, theta_body);
  const [R0, p0] = mr.TransToRp(T0);
  const Euler_order = 'ZYX'; // Euler angle order

  // Position and orientation (euler angle) of end effector
  const position_ee_initial = p0
  const [position_ee, setPositionEE] = React.useState(position_ee_initial);
  const position_ee_Three = mr.worlr2three(position_ee);

  const euler_ee_initial = mr.RotMatToEuler(R0, Euler_order); 
  const [euler_ee, setEuler] = React.useState(euler_ee_initial);
  const euler_ee_Three = mr.worlr2three(euler_ee);

  // const quaternion_ee_initial = mr.RotMatToQuaternion(R0);
  // const [quaternion_ee, setQuaternionEE] = React.useState(quaternion_ee_initial);

  // Update end effector position and orientation (for webcontroller)
  React.useEffect(() => {
    const T = mr.FKinBody(M, Blist, theta_body);
    const [R, p] = mr.TransToRp(T);
    setPositionEE(p);
    setEuler(mr.RotMatToEuler(R, Euler_order)); // Update to ZYX Euler angles
    }, [theta_body]);
  

  /**
   *  Control Methods
   * /
  /** Kinamatics Control **/
  function KinamaticsControl(newPos, newEuler) {
    const T_sd = mr.RpToTrans(mr.EulerToRotMat(newEuler, Euler_order), newPos);
    const [thetalist_sol, ik_success] = mr.IKinBody(Blist, M, T_sd, theta_body_guess, 1e-5, 1e-5);

    if (ik_success) {
      const thetalist_sol_limited = thetalist_sol.map((theta, i) =>
      Math.max(jointLimits[i].min, Math.min(jointLimits[i].max, theta))
      );
      setThetaBody(thetalist_sol_limited);
      setThetaBodyGuess(thetalist_sol_limited);
    } else {
      console.warn("IK failed to converge");
    }
  }

  /** Dynamics Control **/
  function DynamicsControl(newPos, newEuler) {
    const T_sd = mr.RpToTrans(mr.EulerToRotMat(newEuler, Euler_order), newPos);
    const [thetalist_sol, ik_success] = mr.IKinBody(Blist, M, T_sd, theta_body_guess, 1e-5, 1e-5);

    const thetalist_sol_limited = thetalist_sol.map((theta, i) =>
      Math.max(jointLimits[i].min, Math.min(jointLimits[i].max, theta))
      );

    const dt = 0.01;
    const steps = 200;

    const q0 = theta_body.slice();
    const dq0 = Array(q0.length).fill(0);

    const q_ref = thetalist_sol_limited.slice();
    const dq_ref = Array(q0.length).fill(0);
    
    const [q_hist, dq_hist] = mr.simulate_PIDcontrol(q0, dq0, q_ref, dq_ref, dt, steps, Mlist, Glist, Slist, Kplist, Kilist, Kdlist)

    if (ik_success) {
      setQHistQueue(queue => [...queue, q_hist]); 
    } 
    else {
      console.warn("IK failed to converge");
    }
  }

  /** Virtual Force control
   * Using VR controller's virtal force as input to control robot end effector
  */
  function VirtualForceControl(Ftip, dt) {
    const g = [0, 0, -9.81]; 

    const q = theta_body.slice();
    const dq = dtheta_body.slice();

    // Gravity compensation
    const tau_gravity = mr.GravityForces(q, g, Mlist, Glist, Slist)

    // Forward dynamics simulation, return joint accelerations
    const ddq = mr.ForwardDynamics(q, dq, tau_gravity, g, Ftip, Mlist, Glist, Slist)

    //Euler integration to update joint angles and velocities
    const dq_next = dq.map((v, i) => v + ddq[i] * dt);
    const q_next = q.map((v, i) => v + dq_next[i] * dt);

    // Update joint and joint velocities
    setThetaBody(q_next);
    setdThetaBody(dq_next);
  }

  /* VR Controller Simulation */
  const lastPosRef = React.useRef(controller_object.position.clone());
  const lastEulerRef = React.useRef(controller_object.rotation.clone());
  // const lastQuatRef = React.useRef(controller_object.quaternion.clone());
  const Pos_scale = 0.5; 
  const Euler_scale = 0.5;

  const lastVRInputTimeRef = React.useRef(performance.now());
  const v_imp_ref = React.useRef([0, 0, 0, 0, 0, 0]);
  React.useEffect(() => {
    const now = performance.now();
    // const dt = now - lastVRInputTimeRef.current;
    const dt = 16.5/1000;
    lastVRInputTimeRef.current = now;
    console.log('VR input period(ms):', dt);

    if (!trigger_on) {
      setdThetaBody([0, 0, 0, 0, 0, 0]);
    }

    if (rendered && vrModeRef.current && trigger_on) {
      const deltaPos = {
        x: controller_object.position.x - lastPosRef.current.x,
        y: controller_object.position.y - lastPosRef.current.y,
        z: controller_object.position.z - lastPosRef.current.z
      };

      /**
       * Revelant Rotation 
       * (Using abusolute Euler angles will cause drift, especeially when VR controller is far from human body)
       */

      // current and last Euler angles of VR controller
      const currentEuler_VR = [
        controller_object.rotation.x,
        controller_object.rotation.y,
        controller_object.rotation.z
      ];
      const lastEuler_VR = [
        lastEulerRef.current.x,
        lastEulerRef.current.y,
        lastEulerRef.current.z
      ];

      // revelant rotation matrix
      const R_current = mr.EulerToRotMat(currentEuler_VR, Euler_order);
      const R_last = mr.EulerToRotMat(lastEuler_VR, Euler_order);

      function matTranspose(R) {
        return R[0].map((_, i) => R.map(row => row[i]));
      }
      function matDot(A, B) {
        return A.map((row, i) =>
          B[0].map((_, j) =>
            row.reduce((sum, a, k) => sum + a * B[k][j], 0)
          )
        );
      }
      const R_relative = matDot(R_current, matTranspose(R_last));

      // revelant Euler angles
      const deltaEuler_rev = mr.RotMatToEuler(R_relative, Euler_order); 
      const deltaEuler = {
        x: deltaEuler_rev[0], 
        y: deltaEuler_rev[1],
        z: deltaEuler_rev[2]
      };
      
      // calculate the difference in position and orientation
      const deltaPos_Three = [deltaPos.x, deltaPos.y, deltaPos.z];
      const deltaPos_World = mr.three2world(deltaPos_Three);

      const deltaEuler_Three = [deltaEuler.x, deltaEuler.y, deltaEuler.z];
      const deltaEuler_World = mr.three2world(deltaEuler_Three);

      console.log('ΔPosition:', deltaPos, 'ΔEuler:', deltaEuler);

      /* 
      * VR Input Method
      */
      // Speed
      // const speedPos = {
      //   x: deltaPos_World[0] / dt,
      //   y: deltaPos_World[1] / dt,
      //   z: deltaPos_World[2] / dt
      // };
      // const speedEuler = {
      //   x: deltaEuler_World[0] / dt,
      //   y: deltaEuler_World[1] / dt,
      //   z: deltaEuler_World[2] / dt
      // };

      // Limited Speed
      // const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

      // const speedPos = {
      //   x: clamp(deltaPos_World[0] / dt, -0.001, 0.001),
      //   y: clamp(deltaPos_World[1] / dt, -0.001, 0.001),
      //   z: clamp(deltaPos_World[2] / dt, -0.001, 0.001)
      // };
      // const speedEuler = {
      //   x: clamp(deltaEuler_World[0] / dt, -0.002, 0.002),
      //   y: clamp(deltaEuler_World[1] / dt, -0.002, 0.002),
      //   z: clamp(deltaEuler_World[2] / dt, -0.002, 0.002)
      // };

      // Inpedance
      const Kp_VR = [80, 120, 80, 15, 15, 12]; // 位置/姿态弹簧系数
      const Kd_VR = [18, 20, 18, 1.8, 1.5, 1.2];    // 位置/姿态阻尼系数
      const M_VR = [1, 1, 1, 0.10, 0.10, 0.10];     // 虚拟质量

      let v = v_imp_ref.current.slice();

      const ForcePos = {
        x: Kp_VR[0] * deltaPos_World[0] + Kd_VR[0] * (deltaPos_World[0] / dt),
        y: Kp_VR[1] * deltaPos_World[1] + Kd_VR[1] * (deltaPos_World[1] / dt),
        z: Kp_VR[2] * deltaPos_World[2] + Kd_VR[2] * (deltaPos_World[2] / dt)
      }
      const ForceEuler = {
        x: Kp_VR[3] * deltaEuler_World[0] + Kd_VR[3] * (deltaEuler_World[0] / dt),
        y: Kp_VR[4] * deltaEuler_World[1] + Kd_VR[4] * (deltaEuler_World[1] / dt),
        z: Kp_VR[5] * deltaEuler_World[2] + Kd_VR[5] * (deltaEuler_World[2] / dt)
      }

      const accPos = {
        x: ForcePos.x / M_VR[0],
        y: ForcePos.y / M_VR[1],
        z: ForcePos.z / M_VR[2]
      };
      const accEuler = {
        x: ForceEuler.x / M_VR[3],
        y: ForceEuler.y / M_VR[4],
        z: ForceEuler.z / M_VR[5]
      };

      const speedPos = {
        x: v[0] + accPos.x * dt,
        y: v[1] + accPos.y * dt,
        z: v[2] + accPos.z * dt
      };
      const speedEuler = {
        x: v[3] + accEuler.x * dt,
        y: v[4] + accEuler.y * dt,
        z: v[5] + accEuler.z * dt
      };

      console.log('Translation Speed:', speedPos, 'Rotation Speed:', speedEuler, 'dt(s):', dt);

      /**
       * Inverse Kinematics Control
       */
      const currentPos = [...position_ee];
      const currentEuler = [...euler_ee];

      const newPos = [
        currentPos[0] + speedPos.x * dt,
        currentPos[1] + speedPos.y * dt,
        currentPos[2] + speedPos.z * dt
      ];
      const newEuler = [
        currentEuler[0] + speedEuler.z * dt,
        currentEuler[1] - speedEuler.y * dt,
        currentEuler[2] - speedEuler.x * dt
      ];

      KinamaticsControl(newPos, newEuler);

      v_imp_ref.current = [
        Pos_scale*speedPos.x, Pos_scale*speedPos.y, Pos_scale*speedPos.z,
        Euler_scale*speedEuler.x, Euler_scale*speedEuler.y, Euler_scale*speedEuler.z
      ];

      /**
       * Virtual Force Control
       */
      // const Ftip = [
      //   ForcePos.x * Pos_scale,
      //   ForcePos.y * Pos_scale,
      //   ForcePos.z * Pos_scale,
      //   ForceEuler.x * Euler_scale,
      //   ForceEuler.y * Euler_scale,
      //   ForceEuler.z * Euler_scale
      // ];
      // VirtualForceControl(Ftip, dt);


      /**
       * Dynamics Control
       */
      // DynamicsControl(newPos, newEuler);

    }
    // Update last position and orientation
    lastPosRef.current.copy(controller_object.position);
    lastEulerRef.current.copy(controller_object.rotation);
  }, [
    controller_object.position.x,
    controller_object.position.y,
    controller_object.position.z,
    controller_object.rotation.x,
    controller_object.rotation.y,
    controller_object.rotation.z,
    rendered,
    trigger_on,
  ]);

  const vr_controller_pos = [
  controller_object.position.x,
  controller_object.position.y,
  controller_object.position.z
  ];
  const vr_controller_euler = [
    controller_object.rotation.x,
    controller_object.rotation.y,
    controller_object.rotation.z
  ];

  // AB 按钮独立控制夹爪/工具
  React.useEffect(() => {
    if (button_A_Ref.current) {
      setThetaTool(prev => prev + 5); 
    }
    if (button_B_Ref.current) {
      setThetaTool(prev => prev + 5); 
    }
  }, [button_A_Ref.current, button_B_Ref.current]);


  // webController Inputs
  const controllerProps = React.useMemo(() => ({
    robotName, robotNameList, set_robotName,
    toolName, toolNameList, set_toolName,
    c_pos_x,set_c_pos_x,c_pos_y,set_c_pos_y,c_pos_z,set_c_pos_z,
    c_deg_x,set_c_deg_x,c_deg_y,set_c_deg_y,c_deg_z,set_c_deg_z,
    vr_mode:vrModeRef.current,
    selectedMode, setSelectedMode,
    theta_body, setThetaBody,
    theta_tool, setThetaTool,
    position_ee, setPositionEE,
    euler_ee, setEuler,
    onTargetChange: KinamaticsControl,
    // onTargetChange: DynamicsControl
  }), [
    robotName, robotNameList, set_robotName,
    toolName, toolNameList, set_toolName,
    c_pos_x,set_c_pos_x,c_pos_y,set_c_pos_y,c_pos_z,set_c_pos_z,
    c_deg_x,set_c_deg_x,c_deg_y,set_c_deg_y,c_deg_z,set_c_deg_z,
    vrModeRef.current,
    selectedMode, setSelectedMode,
    theta_body, setThetaBody,
    theta_tool, setThetaTool,
    position_ee, setPositionEE,
    euler_ee, setEuler,
    KinamaticsControl,
    // DynamicsControl
  ]);

  // VRController Inputs (Aframe Components)
  React.useEffect(() => {
    registerAframeComponents({
      set_rendered,
      robotChange,
      set_controller_object,
      set_trigger_on,
      set_c_pos_x, set_c_pos_y, set_c_pos_z,
      set_c_deg_x, set_c_deg_y, set_c_deg_z,
      vrModeRef,
      currentRotationRef,
      controller_object,
      order,
      props,
      onAnimationMQTT,
      onXRFrameMQTT,
    });
  }, []);

  // Robot Model Update Props
  const robotProps = React.useMemo(() => ({
    robotNameList, robotName, theta_body, theta_tool
  }), [robotNameList, robotName, theta_body, theta_tool]);

  // Robot Secene Render
  return (
    <RobotScene
      rendered={rendered}
      target_error={target_error}
      box_vis={box_vis}
      cursor_vis={cursor_vis}
      robotProps={robotProps}
      controllerProps={controllerProps}
      dsp_message={dsp_message}
      c_pos_x={c_pos_x}
      c_pos_y={c_pos_y}
      c_pos_z={c_pos_z}
      c_deg_x={c_deg_x}
      c_deg_y={c_deg_y}
      c_deg_z={c_deg_z}
      viewer={props.viewer}
      monitor={props.monitor}
      position_ee={position_ee_Three}
      euler_ee={euler_ee_Three}
      vr_controller_pos={vr_controller_pos}
      vr_controller_euler={vr_controller_euler}
    />
  );
}
