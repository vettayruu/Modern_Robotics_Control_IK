"use client";
import 'aframe'
import * as React from 'react'
import RobotScene from './RobotScene';

import registerAframeComponents from './registerAframeComponents'; // A-Frame 组件注册

const THREE = window.AFRAME.THREE; // これで　AFRAME と　THREEを同時に使える
// import 'aframe-troika-text';

import useMqtt from './useMqtt';
import { connectMQTT, mqttclient,idtopic,subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'

// 关节初始位置、全局变量、修正值、辅助向量等定义
import { joint_pos, order} from '../lib/robot_globals.js'

import * as IKUtils from '../lib/ik_utils';
const {
  round, normalize180, toAngle, toRadian,
  pos_add, pos_sub, distance,
  degree3, calc_side_1, calc_side_2, calc_side_4,
  quaternionToRotation, quaternionToAngle, quaternionDifference, direction_angle,
  get_j5_quaternion,
} = IKUtils;

const mr = require('../modern_robotics/modern_robotics_core.js');
const RobotKinematics = require('../modern_robotics/modern_robotics_Kinematics.js');

// 创建 piper_agilex 机器人运动学对象
const rk = new RobotKinematics("piper_agilex");

// 获取 M 和 Slist
const M = rk.get_M();
const Slist = rk.get_Slist();
const jointLimits = rk.jointLimits; // 获取关节限制

// 全局常量与变量定义
const MQTT_REQUEST_TOPIC = "mgr/request";
const MQTT_DEVICE_TOPIC = "dev/"+idtopic;
const MQTT_CTRL_TOPIC =        "control/"+idtopic; // 自分のIDに制御を送信
const MQTT_ROBOT_STATE_TOPIC = "robot/"; // Viwer のばあい

// React组件主函数
export default function DynamicHome(props) {
  // 0. React 状态变量定义（useState、useRef）
  // 关节角度、目标点、控制对象等所有状态变量
  
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

  // 各关节角度（J1~J7），单位：度
  const [j1_rotate,set_j1_rotate] = React.useState(0)
  const [j2_rotate,set_j2_rotate] = React.useState(0)
  const [j3_rotate,set_j3_rotate] = React.useState(0)
  const [j4_rotate,set_j4_rotate] = React.useState(0)
  const [j5_rotate,set_j5_rotate] = React.useState(-90)
  const [j6_rotate,set_j6_rotate] = React.useState(0)
  const [j7_rotate,set_j7_rotate] = React.useState(24) // 夹爪用

  // 机器人初始位置
  const realTargetRef = React.useRef({x:0,y:0.19647,z:-0.195}); 
  // const realTargetRef = React.useRef(pos_inital_three); 

  // 当前输出的关节角度数组（用于MQTT/渲染等）
  const [rotate, set_rotate] = React.useState([0,0,0,0,0,0,0])
  const rotateRef = React.useRef([0,0,0,0,0,0,0]); // 用于跨渲染帧保存关节角度

  // 输入角度（外部输入或UI输入）
  const [input_rotate, set_input_rotate] = React.useState([0,0,0,0,0,0,0])

  // 各关键点的三维对象（用于A-Frame/Three.js渲染）
  const [p11_object,set_p11_object] = React.useState()
  const [p12_object,set_p12_object] = React.useState()
  const [p13_object,set_p13_object] = React.useState()
  const [p14_object,set_p14_object] = React.useState()
  const [p15_object,set_p15_object] = React.useState()
  const [p16_object,set_p16_object] = React.useState()
  const target_p16_ref = React.useRef(null)
  const [p20_object,set_p20_object] = React.useState()
  const [p21_object,set_p21_object] = React.useState()
  const [p22_object,set_p22_object] = React.useState()
  const [p51_object,set_p51_object] = React.useState()

  // 控制器相关状态
  // const [controller_object,set_controller_object] = React.useState(new THREE.Object3D())
  const [controller_object, set_controller_object] = React.useState(() => {
    const controller_object = new THREE.Object3D();
    // 设置与机器人初始位置一致
    controller_object.position.set(realTargetRef.current.x, realTargetRef.current.y, realTargetRef.current.z);
    controller_object.rotation.set(0, 0, 0); // 如需与机器人朝向一致可设置对应角度
    return controller_object;
    });


  const [trigger_on,set_trigger_on] = React.useState(false)
  const [start_pos,set_start_pos] = React.useState(new THREE.Vector3())
  const [save_target,set_save_target] = React.useState()
  const vrModeRef = React.useRef(false); // VR模式标志

  // 夹爪/按钮状态
  const [grip_on, set_grip_on] = React.useState(false);
  const [grip_value, set_grip_value] = React.useState(0);
  const gripRef = React.useRef(false);
  const gripValueRef = React.useRef(0);

  const [button_a_on, set_button_a_on] = React.useState(false);
  const buttonaRef = React.useRef(null);
  const [button_b_on, set_button_b_on] = React.useState(false)
  const buttonbRef = React.useRef(null);
  const [selectedMode, setSelectedMode] = React.useState('control'); // 控制/监控模式
  const robotIDRef = React.useRef("none"); // 当前机器人ID

  // 测试点坐标(显示为绿色方块)
  const [test_pos,set_test_pos] = React.useState({x:0,y:0,z:0})

  // 摄像机位置与朝向
  const [c_pos_x,set_c_pos_x] = React.useState(0.23)
  const [c_pos_y,set_c_pos_y] = React.useState(0.3)
  const [c_pos_z,set_c_pos_z] = React.useState(-0.6)
  const [c_deg_x,set_c_deg_x] = React.useState(0)
  const [c_deg_y,set_c_deg_y] = React.useState(150)
  const [c_deg_z,set_c_deg_z] = React.useState(0)

  // 手腕欧拉角与工具旋转
  const [wrist_rot_x,set_wrist_rot_x_org] = React.useState(180)
  const [wrist_rot_y,set_wrist_rot_y_org] = React.useState(0)
  const [wrist_rot_z,set_wrist_rot_z_org] = React.useState(0)
  const [tool_rotate,set_tool_rotate] = React.useState(0)
  const [wrist_degree,set_wrist_degree] = React.useState({direction:0,angle:0})
  const [dsp_message,set_dsp_message] = React.useState("")

  // 工具列表与当前工具
  const toolNameList = ["No tool"]
  const [toolName,set_toolName] = React.useState(toolNameList[0])

  // IK相关参数
  const [p14_maxlen,set_p14_maxlen] = React.useState(0)
  const [do_target_update, set_do_target_update] = React.useState(0) // 目标点更新计数

  // 动画帧ID
  const reqIdRef = React.useRef()

  // 在组件顶部定义
  const rotatetableREF = React.useRef([[], [], [], [], [], []]);
  const object3DtableREF = React.useRef([]);

  // MQTT/状态同步相关
  const receiveStateRef = React.useRef(false);
  const currentRotationRef = React.useRef(new THREE.Euler(0.6654549523360951,0,0,order));
  const targetMoveDistanceRef = React.useRef(0.2);
  const [target,set_target_org] = React.useState(realTargetRef.current)
  const jointErrorRef = React.useRef([false, false, false, false, false]); // j1~j5错误标志

  // 动画主循环，驱动渲染和运动学动画
  const loop = ()=>{
    setNow(performance.now()); // 更新时间戳，驱动依赖 now 的动画 useEffect
    reqIdRef.current = window.requestAnimationFrame(loop) // 持续递归调用，形成动画循环
  }

  // 初始化动画循环
  React.useEffect(() => {
    loop()
    return () => window.cancelAnimationFrame(reqIdRef.current) // 组件卸载时清理动画帧
  },[])

  // 目标点设置，记录目标点移动距离
  const set_target = (new_pos)=>{
    set_target_org((prev_pos)=>{
      targetMoveDistanceRef.current = distance(realTargetRef.current,new_pos) // 记录目标点移动距离，用于动画插值
      return new_pos
    })
  }

  // 手腕旋转设置，重置目标点移动距离
  const set_wrist_rot_x = (new_rot)=>{
    set_wrist_rot_x_org((prev_rot)=>{
      targetMoveDistanceRef.current = 0 // 手腕旋转时，重置移动距离
      return new_rot
    })
  }
  const set_wrist_rot_y = (new_rot)=>{
    set_wrist_rot_y_org((prev_rot)=>{
      targetMoveDistanceRef.current = 0
      return new_rot
    })
  }
  const set_wrist_rot_z = (new_rot)=>{
    set_wrist_rot_z_org((prev_rot)=>{
      targetMoveDistanceRef.current = 0
      return new_rot
    })
  }

  // VR/AR 控制相关的 useEffect
  // VR控制器位置变化时，实时更新目标点
  React.useEffect(() => {
    // 只有在渲染完成、VR模式开启、扳机按下时才响应
    if(rendered && vrModeRef.current && trigger_on){
      const move_pos = pos_sub(start_pos,controller_object.position)
      let target_pos
      if(save_target === undefined){
        set_save_target({...target})
        target_pos = pos_sub(target,move_pos)
      }else{
        target_pos = pos_sub(save_target,move_pos)
      }
      if(target_pos.y < 0.012){ // 防止目标点低于地面
        target_pos.y = 0.012
      }
      set_target({x:round(target_pos.x),y:round(target_pos.y),z:round(target_pos.z)})
    }
  },[controller_object.position.x,controller_object.position.y,controller_object.position.z])

  // VR控制器旋转变化时，实时更新手腕旋转角度
  React.useEffect(() => {
    if(rendered && vrModeRef.current && trigger_on){
      // 计算当前控制器旋转与起始旋转的差值
      const quat_start = new THREE.Quaternion().setFromEuler(start_rotation);
      const quat_controller = new THREE.Quaternion().setFromEuler(controller_object.rotation);
      const quatDifference1 = quat_start.clone().invert().multiply(quat_controller);

      const quat_save = new THREE.Quaternion().setFromEuler(save_rotation);
      const quatDifference2 = quat_start.clone().invert().multiply(quat_save);

      const wk_mtx = quat_start.clone().multiply(quatDifference1).multiply(quatDifference2)
      currentRotationRef.current = new THREE.Euler().setFromQuaternion(wk_mtx,controller_object.rotation.order)

      wk_mtx.multiply(
        new THREE.Quaternion().setFromEuler(
          new THREE.Euler(
            (0.6654549523360951*-1),  //x
            Math.PI,  //y
            Math.PI,  //z
            controller_object.rotation.order
          )
        )
      )

      const wk_euler = new THREE.Euler().setFromQuaternion(wk_mtx,controller_object.rotation.order)
      set_wrist_rot_x(round(toAngle(wk_euler.x)))
      set_wrist_rot_y(round(toAngle(wk_euler.y)))
      set_wrist_rot_z(round(toAngle(wk_euler.z)))
    }
  },[controller_object.rotation.x,controller_object.rotation.y,controller_object.rotation.z])

  // rendered 状态变化时，触发目标点更新
  React.useEffect(() => {
    if(rendered){
      set_do_target_update((prev) => prev + 1) // increment the counter to trigger main
    }
  },[rendered])

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
  set_input_rotate,
  set_target_org,
  set_wrist_rot_x, set_wrist_rot_y, set_wrist_rot_z,
  set_rendered,
  set_p14_maxlen,
  set_p11_object, set_p12_object, set_p13_object, set_p14_object, set_p15_object, set_p16_object,
  set_p20_object, set_p21_object, set_p22_object, set_p51_object,
  target_p16_ref,
  robotIDRef,
  joint_pos,
  round,
  toAngle,
  order,
  publishMQTT,
  MQTT_REQUEST_TOPIC, MQTT_DEVICE_TOPIC, MQTT_CTRL_TOPIC, MQTT_ROBOT_STATE_TOPIC,
  receiveStateRef,
  mqttclientRef, //not used
  gripRef,
  buttonaRef,
  buttonbRef,
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
      joints: rotateRef.current, // 当前各关节角度
      grip: gripRef.current      // 当前夹爪状态
      // trigger: [gripRef.current, buttonaRef.current, buttonbRef.current, gripValueRef.current]
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
        joints: rotateRef.current,
        trigger: [gripRef.current, buttonaRef.current, buttonbRef.current, gripValueRef.current]
      });
      publishMQTT(MQTT_CTRL_TOPIC, ctl_json);
    }
  }

  // A-Frame 组件注册（如 robot-click、j_id、vr-controller-right、jtext、scene 等自定义组件）
  // 注册自定义 A-Frame 组件，绑定回调和状态
  const start_rotation = React.useRef();
  const save_rotation = React.useRef();
  React.useEffect(() => {
    registerAframeComponents({
      set_rendered,
      set_p14_maxlen,
      set_p11_object, set_p12_object, set_p13_object, set_p14_object, set_p15_object, set_p16_object,
      set_p20_object, set_p21_object, set_p22_object, set_p51_object,
      target_p16_ref,
      robotChange,
      set_controller_object,
      set_start_pos,
      set_trigger_on,
      set_save_target,
      set_c_pos_x, set_c_pos_y, set_c_pos_z,
      set_c_deg_x, set_c_deg_y, set_c_deg_z,
      vrModeRef,
      currentRotationRef,
      start_rotation,
      save_rotation,
      controller_object,
      order,
      props,
      onAnimationMQTT,
      onXRFrameMQTT,
      object3DtableREF
    });
  }, []);

  /*** Forward Kinematics Part ***/
  // Initial joint and tool angles
  // const theta_body_initial = mr.deg2rad([0, -30, 70, 0, 65, 0]);
  const theta_body_initial = [0, -0.27473, 1.44144, 0, 1.22586, 0];
  const theta_tool_inital = 0;
  const [theta_body, setThetaBody] = React.useState(theta_body_initial);
  const [theta_tool, setThetaTool] = React.useState(theta_tool_inital);
  const Blist = mr.SlistToBlist(M, Slist); // Convert Slist to Blist

  // Foward Kinematics solution
  // const T0 = mr.FKinSpace(M, Slist, theta_body);
  const T0 = mr.FKinBody(M, Blist, theta_body);
  const [R0, p0] = mr.TransToRp(T0);
  const Euler_order = 'ZYX'; // Euler angle order

  // Position and orientation (euler angle) of end effector
  const position_ee_initial = p0
  const [position_ee, setPositionEE] = React.useState(position_ee_initial);
  const position_ee_Three = mr.worlr2three(position_ee);

  /* Effector orientation (Body-Fixed: ZYX order, Space-Fixed: XYZ order) */
  const euler_ee_initial = mr.RotMatToEuler(R0, Euler_order); 
  const [euler_ee, setEuler] = React.useState(euler_ee_initial);
  const euler_ee_Three = mr.worlr2three(euler_ee);

  // // Twist of end effector (angular velocity and angle)
  // const [omghat0, theta0] = mr.RotMatToAxisAngle(R0); // end effector twist
  // const twist_ee_initial = [omghat0, theta0]
  // const [twist_ee, setTwistEE] = React.useState(twist_ee_initial);

  // Update end effector position and orientation 
  React.useEffect(() => {
    // const T = mr.FKinSpace(M, Slist, theta_body);
    const T = mr.FKinBody(M, Blist, theta_body);
    const [R, p] = mr.TransToRp(T);

    setPositionEE(p);
    setEuler(mr.RotMatToEuler(R, Euler_order)); // Update to ZYX Euler angles
    // setTwistEE(mr.RotMatToAxisAngle(R));
    }, [theta_body]);
  

  /*** Inverse Kinematics Part ***/
  // Theta guess for Newton's method in inverse kinematics
  const [theta_body_guess, setThetaBodyGuess] = React.useState(theta_body);

  // User control 
  function handleTargetChange(newPos, newEuler) {
    /* Space-fixed IK, use Slist */
    // const T_sd = mr.RpToTrans(mr.EulerXYZToRotMat(newEuler), newPos);
    // const [thetalist_sol, ik_success] = mr.IKinSpace(Slist, M, T_sd, theta_body_guess, 1e-4, 1e-5);

    /* Body-fixed IK, use Blist */
    const T_sd = mr.RpToTrans(mr.EulerToRotMat(newEuler, order), newPos);
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

  // Controller Inputs
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
    // twist_ee, setTwistEE
    onTargetChange: handleTargetChange,
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
    // twist_ee, setTwistEE
    handleTargetChange
  ]);

  // Robot Model Update Props
  const robotProps = React.useMemo(() => ({
    robotNameList, robotName, theta_body, theta_tool
  }), [robotNameList, robotName, theta_body, theta_tool]);

  // Robot Secene Render
  return (
    <RobotScene
      rendered={rendered} // render trigger
      target_error={target_error}
      box_vis={box_vis}
      test_pos={test_pos}
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
    />
  );
}
