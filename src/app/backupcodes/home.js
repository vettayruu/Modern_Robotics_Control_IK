"use client";
import 'aframe'
import * as React from 'react'

import Assets from './Assets' //机器人模型加载零件渲染
import { Model, Model_Tool, Select_Robot } from './Model'; // 机器人模型加载
import Line from './Line'; // 辅助线组件

import registerAframeComponents from './registerAframeComponents'; // A-Frame 组件注册

//import * as THREE from 'three' // これだと、THREE のインスタンスが複数になり問題
const THREE = window.AFRAME.THREE; // これで　AFRAME と　THREEを同時に使える
import 'aframe-troika-text';

import Controller from './controller.js'
// import useVrController from './useVrController';

import useMqtt from './useMqtt';
import { connectMQTT, mqttclient,idtopic,subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'

// 关节初始位置、全局变量、修正值、辅助向量等定义
import { joint_pos, order, x_vec_base, y_vec_base, z_vec_base,
         max_move_unit, rotvec_table, target_move_speed,
         j1_Correct_value, j2_Correct_value, j3_Correct_value, j4_Correct_value, j5_Correct_value,
         j6_Correct_value, j7_Correct_value,} from '../lib/robot_globals.js'

import * as IKUtils from '../lib/ik_utils';
const {
  round, normalize180, toAngle, toRadian,
  pos_add, pos_sub, distance,
  degree3, calc_side_1, calc_side_2, calc_side_4,
  quaternionToRotation, quaternionToAngle, quaternionDifference, direction_angle,
  get_j5_quaternion, get_p21_pos 
} = IKUtils;

import {
  // target_update_core,
  // target15_update_core,
  // get_all_rotate_core,
  // get_J2_J3_rotate_core
} from '../lib/ik_core'

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

  // VR控制器相关状态
  const [controller_object,set_controller_object] = React.useState(new THREE.Object3D())
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

  // 测试点坐标
  const [test_pos,set_test_pos] = React.useState({x:0,y:0,z:0})

  // 摄像机位置与朝向
  const [c_pos_x,set_c_pos_x] = React.useState(0)
  const [c_pos_y,set_c_pos_y] = React.useState(0.35)
  const [c_pos_z,set_c_pos_z] = React.useState(0.6)
  const [c_deg_x,set_c_deg_x] = React.useState(0)
  const [c_deg_y,set_c_deg_y] = React.useState(0)
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
  const [p15_16_len,set_p15_16_len] = React.useState(joint_pos.j7.z)
  const [p14_maxlen,set_p14_maxlen] = React.useState(0)
  const [do_target_update, set_do_target_update] = React.useState(0) // 目标点更新计数

  // 动画帧ID
  const reqIdRef = React.useRef()

  // MQTT/状态同步相关
  const receiveStateRef = React.useRef(false);
  const currentRotationRef = React.useRef(new THREE.Euler(0.6654549523360951,0,0,order));
  const targetMoveDistanceRef = React.useRef(0.2);
  const realTargetRef = React.useRef({x:0,y:0.19647,z:-0.195});
  const [target,set_target_org] = React.useState(realTargetRef.current)
  const jointErrorRef = React.useRef([false, false, false, false, false]); // j1~j5错误标志

  // 动画主循环，驱动渲染和运动学动画
  const loop = ()=>{
    setNow(performance.now()); // 更新时间戳，驱动依赖 now 的动画 useEffect
    reqIdRef.current = window.requestAnimationFrame(loop) // 持续递归调用，形成动画循环
  }

  // 在组件顶部定义
  const rotatetableREF = React.useRef([[], [], [], [], [], []]);
  const object3DtableREF = React.useRef([]);

// 使用时：rotate_table.current、object3D_table.current

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
      set_do_target_update((prev) => prev + 1) // increment the counter to trigger target_update
    }
  },[rendered])

  // 目标点更新计数变化时，执行逆运动学主流程
  React.useEffect(()=>{
    target_update();
  },[do_target_update])

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

  // 关节角度动画插值
  React.useEffect(()=>{
    const flg = props.viewer || props.monitor;
    // 调试输出
    console.log("Now!", now, controller_object.rotation.x);
    for (let i = 0; i < rotatetableREF.current.length; i++) {
      const current_table = rotatetableREF.current[i];
      const current_object3D = object3DtableREF.current[i];
      if (!current_object3D) continue; // 防止未初始化时报错
      if (current_table.length > 0) {
        const current_data = current_table[0];
        if (current_data.first) {
          // 初始化插值参数
          current_data.first = false;
          current_data.starttime = performance.now();
          current_data.start_quaternion = current_object3D.quaternion.clone();
          current_data.end_quaternion = new THREE.Quaternion().setFromAxisAngle(rotvec_table[i], toRadian(current_data.rot));
          const move_time_1 = targetMoveDistanceRef.current * target_move_speed;
          const wk_euler = new THREE.Quaternion().angleTo(
            current_data.start_quaternion.clone().invert().multiply(current_data.end_quaternion)
          );
          const move_time_2 = (toAngle(wk_euler) * max_move_unit) * 1000;
          current_data.move_time = !flg ? Math.max(move_time_1, move_time_2) : move_time_1;
          current_data.endtime = current_data.starttime + current_data.move_time;
        }
        const current_time = performance.now();
        if (current_time < current_data.endtime) {
          // 插值动画
          const elapsed_time = current_time - current_data.starttime;
          current_object3D.quaternion.slerpQuaternions(
            current_data.start_quaternion,
            current_data.end_quaternion,
            (elapsed_time / current_data.move_time)
          );
        } else {
          // 动画结束，应用最终姿态
          current_object3D.quaternion.copy(current_data.end_quaternion);
          current_table.shift();
        }
      }
    }
  }, [now])

  // 关节角度输入变化时，更新动画队列
  React.useEffect(() => {
  if (object3DtableREF.current[0] !== undefined) {
    if(rotatetableREF.current[0].length > 1){
      rotatetableREF.current[0].pop()
    }
    rotatetableREF.current[0].push({rot:j1_rotate,first:true})
    }
  }, [j1_rotate])

  // 其它关节角度输入变化时，更新动画队列
  const joint_rotates = [
  j1_rotate, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate
  ];

  for (let i = 0; i < 6; i++) {
    React.useEffect(() => {
      if (object3DtableREF.current[i] !== undefined) {
        if (rotatetableREF.current[i].length > 1) {
          rotatetableREF.current[i].pop();
        }
        rotatetableREF.current[i].push({ rot: joint_rotates[i], first: true });
      }
    }, [joint_rotates[i]]);
  }

  // 关节角度变化时，更新输出数组
  React.useEffect(() => {
  //    if(!props.viewer){
      const new_rotate = [
        round(normalize180(j1_rotate+j1_Correct_value),3),
        round(j2_rotate+j2_Correct_value,3),
        round(j3_rotate+j3_Correct_value,3),
        round(j4_rotate+j4_Correct_value,3),
        round(j5_rotate+j5_Correct_value,3),
        round(j6_rotate+j6_Correct_value,3),
        round(j7_rotate+j7_Correct_value,3)
      ]
      set_rotate(new_rotate)
      rotateRef.current = new_rotate
  //      console.log("New Rotate",new_rotate)
  //    }
  }, [j1_rotate,j2_rotate,j3_rotate,j4_rotate,j5_rotate,j6_rotate,j7_rotate])

  // 输入角度变化时，直接设置关节角度
  React.useEffect(() => {
    if (object3DtableREF.current[0] !== undefined) {
      targetMoveDistanceRef.current = 0
      const rotate_value = normalize180(input_rotate[0]-j1_Correct_value)
      set_j1_rotate(rotate_value)
    }
  }, [input_rotate[0]])
  
  // 其它输入角度变化时，直接设置关节角度
  const correct_values = [
  j1_Correct_value, j2_Correct_value, j3_Correct_value,
  j4_Correct_value, j5_Correct_value, j6_Correct_value
  ];
  const set_rotates = [
    set_j1_rotate, set_j2_rotate, set_j3_rotate,
    set_j4_rotate, set_j5_rotate, set_j6_rotate
  ];

  for (let i = 0; i < 6; i++) {
    React.useEffect(() => {
      if (object3DtableREF.current[i] !== undefined) {
        targetMoveDistanceRef.current = 0;
        const rotate_value = normalize180(input_rotate[i] - correct_values[i]);
        set_rotates[i](rotate_value);
      }
    }, [input_rotate[i]]);
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

  // 目标点与手腕姿态的计算与更新
  const get_j5_quaternion = (rot_x=wrist_rot_x,rot_y=wrist_rot_y,rot_z=wrist_rot_z)=>{
    return new THREE.Quaternion().setFromEuler(
      new THREE.Euler(toRadian(rot_x), toRadian(rot_y), toRadian(rot_z), order)
    )
  }

  const get_p21_pos = ()=>{
    const j5q = get_j5_quaternion()
    const p21_pos = quaternionToRotation(j5q,{x:0,y:0,z:p15_16_len})
    return p21_pos
  }

  // const j5q = get_j5_quaternion(wrist_rot_x, wrist_rot_y, wrist_rot_z, order);

  // const p21_pos = get_p21_pos(wrist_rot_x, wrist_rot_y, wrist_rot_z, order, p15_16_len);

  React.useEffect(() => {
    if(rendered){
      set_do_target_update((prev) => prev + 1) // increment the counter to trigger target_update

      if(p51_object)p51_object.quaternion.copy(get_j5_quaternion())
  
    }
  },[target,tool_rotate,wrist_rot_x,wrist_rot_y,wrist_rot_z])

  // 逆运动学主流程 target_update、target15_update、get_all_rotate、get_J2_J3_rotate

/**
 * 目标点/手腕姿态变化时的主入口，自动计算逆运动学并更新关节角度
 */
const target_update = ()=>{
  // 计算末端执行器的目标点（p21），并根据基向量计算手腕方向与俯仰角
  console.log('调用get_p21_pos参数:', wrist_rot_x, wrist_rot_y, wrist_rot_z, order, p15_16_len);
  const p21_pos = get_p21_pos(wrist_rot_x, wrist_rot_y, wrist_rot_z, order, p15_16_len);
  const {direction,angle} = direction_angle(p21_pos, z_vec_base, y_vec_base)
  if(isNaN(direction)){
    console.log("p21_pos 指定可能範囲外！")
    set_dsp_message("p21_pos 指定可能範囲外！")
    return
  }
  if(isNaN(angle)){
    console.log("p21_pos 指定可能範囲外！")
    set_dsp_message("p21_pos 指定可能範囲外！")
    return
  }
  // 更新手腕姿态
  set_wrist_degree({direction,angle})
  // 进入下一步逆解流程
  target15_update(direction,angle)
}

/**
 * 逆运动学主循环，尝试多次修正目标点以获得最优解
 * @param {number} wrist_direction 手腕方向角
 * @param {number} wrist_angle     手腕俯仰角
 */
const target15_update = (wrist_direction,wrist_angle)=>{
  let dsp_message = ""
  const shift_target = {...target}
  let save_target = {...target}
  let result_rotate = {j1_rotate,j2_rotate,j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
  let save_distance = undefined
  let save_distance_cnt = 0
  let save_rotate = {...result_rotate}
  let j3_pos_wk = new THREE.Vector3()
  // 关节错误标志清零
  jointErrorRef.current[0] = false
  jointErrorRef.current[1] = false
  jointErrorRef.current[2] = false
  jointErrorRef.current[3] = false
  jointErrorRef.current[4] = false

  // 迭代修正目标点，尝试收敛到最优解
  for(let i=0; i<10; i=i+1){
    // 打印每次循环的关键参数
    console.log(
      `第${i}次循环:`,
      'wrist_rot_x:', wrist_rot_x,
      'wrist_rot_y:', wrist_rot_y,
      'wrist_rot_z:', wrist_rot_z,
      'order:', order,
      'p15_16_len:', p15_16_len
    );
      // 计算所有关节角度前，打印 get_p21_pos 的参数
    console.log(
      `get_all_rotate_core前: final_target=`, shift_target,
      'wrist_direction:', wrist_direction,
      'wrist_angle:', wrist_angle,
      'p15_16_len:', p15_16_len
    );
    set_test_pos({...shift_target})
    // 计算所有关节角度
    result_rotate = get_all_rotate(shift_target,wrist_direction,wrist_angle)
    // const result_rotate = get_all_rotate_core({
    //     final_target: shift_target,
    //     wrist_direction,
    //     wrist_angle,
    //     p15_16_len,
    //     get_j5_quaternion
    //   });
    if(result_rotate.dsp_message){
      dsp_message = result_rotate.dsp_message
      console.log(dsp_message)
      set_target_error(true)
    }
    // 检查末端误差，若已收敛则提前退出
    const base_m4 = new THREE.Matrix4().multiply(
      new THREE.Matrix4().makeRotationY(toRadian(result_rotate.j1_rotate)).setPosition(joint_pos.j1.x,joint_pos.j1.y,joint_pos.j1.z)
    ).multiply(
      new THREE.Matrix4().makeRotationX(toRadian(result_rotate.j2_rotate)).setPosition(joint_pos.j2.x,joint_pos.j2.y,joint_pos.j2.z)
    ).multiply(
      new THREE.Matrix4().makeRotationX(toRadian(result_rotate.j3_rotate)).setPosition(joint_pos.j3.x,joint_pos.j3.y,joint_pos.j3.z)
    )
    j3_pos_wk = new THREE.Vector3().applyMatrix4(base_m4)
    base_m4.multiply(
      new THREE.Matrix4().makeRotationY(toRadian(result_rotate.j4_rotate)).setPosition(joint_pos.j4.x,joint_pos.j4.y,joint_pos.j4.z)
    ).multiply(
      new THREE.Matrix4().makeRotationX(toRadian(result_rotate.j5_rotate)).setPosition(joint_pos.j5.x,joint_pos.j5.y,joint_pos.j5.z)
    ).multiply(
      new THREE.Matrix4().makeRotationZ(toRadian(result_rotate.j6_rotate)).setPosition(joint_pos.j6.x,joint_pos.j6.y,joint_pos.j6.z)
    ).multiply(
      new THREE.Matrix4().setPosition(joint_pos.j7.x,joint_pos.j7.y,joint_pos.j7.z)
    )
    const result_target = new THREE.Vector3().applyMatrix4(base_m4)
    const sabun_pos = pos_sub(target,result_target)
    const sabun_distance = sabun_pos.x**2+sabun_pos.y**2+sabun_pos.z**2
    const wk_euler = new THREE.Euler().setFromRotationMatrix(base_m4,order)
    const sabun_angle = get_j5_quaternion().angleTo(new THREE.Quaternion().setFromEuler(wk_euler))
    // 如果位置和姿态误差都很小，则认为收敛
    if(round(sabun_distance) <= 0 && round(sabun_angle,2) <= 0){
      save_target = {...result_target}
      break
    }
    // 误差未收敛，判断是否需要提前终止
    if(save_distance === undefined){
      save_distance = sabun_distance
    }else{
      if(save_distance < sabun_distance){
        save_distance_cnt = save_distance_cnt + 1
        if(save_distance_cnt > 1){
          if(round(sabun_distance,4) <= 0){
            result_rotate = {...save_rotate}
            console.log("姿勢制御困難！")
            save_target = {...result_target}
            break  
          }
          console.log("姿勢制御不可！")
          set_dsp_message("姿勢制御不可！")
          console.log(`result_target:{x:${result_target.x}, y:${result_target.y}, z:${result_target.z}}`)
          set_target_error(true)
          return
        }
      }
      save_distance = sabun_distance
      save_rotate = {...result_rotate}
    }
    // 根据误差修正目标点，继续迭代
    shift_target.x = shift_target.x + sabun_pos.x
    shift_target.y = shift_target.y + sabun_pos.y
    shift_target.z = shift_target.z + sabun_pos.z
  }

  // 检查各关节角度是否超限，设置错误标志
  if(dsp_message === ""){
    const wk_j1_rotate = normalize180(result_rotate.j1_rotate + j1_Correct_value)
    if(wk_j1_rotate<-165 || wk_j1_rotate>165){
      dsp_message = `j1_rotate 指定可能範囲外！:(${wk_j1_rotate})`
      jointErrorRef.current[0] = true
    }
    const wk_j2_rotate = result_rotate.j2_rotate + j2_Correct_value
    if(wk_j2_rotate<-5 || wk_j2_rotate>200){
      dsp_message = `j2_rotate 指定可能範囲外！:(${wk_j2_rotate})`
      jointErrorRef.current[1] = true
    }
    const wk_j3_rotate = result_rotate.j3_rotate + j3_Correct_value
    if(wk_j3_rotate<-180 || wk_j3_rotate>5){
      dsp_message = `j3_rotate 指定可能範囲外！:(${wk_j3_rotate})`
      jointErrorRef.current[2] = true
    }
    const wk_j4_rotate = result_rotate.j4_rotate + j4_Correct_value
    if(wk_j4_rotate<-110 || wk_j4_rotate>110){
      dsp_message = `j4_rotate 指定可能範囲外！:(${wk_j4_rotate})`
      jointErrorRef.current[3] = true
    }
    const wk_j5_rotate = result_rotate.j5_rotate + j5_Correct_value
    if(wk_j5_rotate<-80 || wk_j5_rotate>80){
      dsp_message = `j5_rotate 指定可能範囲外！:(${wk_j5_rotate})`
      jointErrorRef.current[4] = true
    }
  }

  // 若无错误，更新关节角度和目标点
  if(dsp_message === ""){
    set_target_error(false)
    set_j1_rotate(round(result_rotate.j1_rotate))
    set_j2_rotate(round(result_rotate.j2_rotate))
    set_j3_rotate(round(result_rotate.j3_rotate))
    set_j4_rotate(round(result_rotate.j4_rotate))
    set_j5_rotate(round(result_rotate.j5_rotate))
    set_j6_rotate(normalize180(round(result_rotate.j6_rotate + tool_rotate)))
    realTargetRef.current = {...save_target}
  }else{
    set_target_error(true)
  }
  set_dsp_message(dsp_message)
}

/**
 * 计算所有关节角度（逆运动学核心步骤）
 * 根据末端目标点和手腕姿态，推导出各关节的旋转角度。
 * 
 * @param {Object} final_target - 末端执行器的目标点坐标 {x, y, z}
 * @param {number} wrist_direction - 手腕方向角（平面旋转）
 * @param {number} wrist_angle - 手腕俯仰角
 * @returns {Object} 
 *   {
 *     j1_rotate, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate, 
 *     dsp_message // 错误信息（如有）
 *   }
 * 
 * 主要流程说明：
 * 1. 计算各关键点的空间位置（如p15、p16等），判断目标点是否在可达范围内。
 * 2. 通过三角学和空间几何关系，推导出 J2、J3 的旋转角度（调用 get_J2_J3_rotate）。
 * 3. 计算 J1 的旋转角度（基座方向），并判断是否需要“反向”解。
 * 4. 计算 J4、J5 的旋转角度，确保手腕姿态与目标一致。
 * 5. 通过四元数差值，推导 J6（末端旋转）角度。
 * 6. 检查所有角度的合法性，若有异常则返回错误信息。
 * 
 * 注意：本函数为逆运动学的核心，涉及大量空间几何和三角学推导，具体实现请结合机械臂结构理解。
 */
const get_all_rotate = (final_target, wrist_direction, wrist_angle) => {
  let dsp_message = "";

  // 1. 计算末端目标点（p16）和手腕点（p15）的位置
  const p16_pos = new THREE.Vector3(final_target.x, final_target.y, final_target.z);
  const p15_16_offset_pos = get_p21_pos(); // 末端到手腕的偏移
  const p15_pos_wk = pos_sub(p16_pos, p15_16_offset_pos);
  const p15_pos = new THREE.Vector3(p15_pos_wk.x, p15_pos_wk.y, p15_pos_wk.z);

    let back = false
    if(round(p15_pos.x) !== round(p16_pos.x) || round(p15_pos.z) !== round(p16_pos.z)){
      let wk_angleC = toAngle(
        new THREE.Vector3(p15_pos.x,0,p15_pos.z).sub(new THREE.Vector3()).angleTo(
          new THREE.Vector3(p16_pos.x,0,p16_pos.z).sub(new THREE.Vector3())))
      if(isNaN(wk_angleC)){
        wk_angleC = 0
      }
      wk_angleC = round(wk_angleC)
      if(wk_angleC > 90){
        back = true
      }else{
        const distance_p15 = distance({x:0,y:0,z:0},{x:p15_pos.x,y:0,z:p15_pos.z})
        const distance_p16 = distance({x:0,y:0,z:0},{x:p16_pos.x,y:0,z:p16_pos.z})
        if(distance_p15 > distance_p16){
          back = true
        }
      }
    }

    // 3. 通过三角学推导J2、J3角度
    const syahen_t15 = distance(joint_pos.j2, p15_pos); // 斜边长度
    const takasa_t15 = (p15_pos.y - joint_pos.j2.y);    // 高度差
    const { k: angle_t15 } = calc_side_4(syahen_t15, takasa_t15);
    const distance_j3 = distance({ x: 0, y: 0, z: 0 }, joint_pos.j3);
    const distance_j4 = distance({ x: 0, y: 0, z: 0 }, joint_pos.j4);
    const result_t15 = get_J2_J3_rotate(angle_t15 * (back ? -1 : 1), distance_j3, distance_j4, syahen_t15);
    if (result_t15.dsp_message) {
      dsp_message = result_t15.dsp_message;
      return { j1_rotate, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate, dsp_message };
    }
    const wk_j2_rotate = result_t15.j2_rotate;
    const wk_j3_rotate = result_t15.j3_rotate;
    
    // 4. 计算J1角度（基座方向）
    const dir_sign_t15 = p15_pos.x < 0 ? -1 : 1
    const xz_vector_t15 = new THREE.Vector3(p15_pos.x,0,p15_pos.z).normalize()
    const direction_t15 = (toAngle(z_vec_base.angleTo(xz_vector_t15)))*dir_sign_t15
    if(isNaN(direction_t15)){
      dsp_message = "direction_t15 指定可能範圍外！"
      return {j1_rotate,j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
    }

    let wk_j1_rotate = normalize180(direction_t15 + (back?180:0))
    if(isNaN(wk_j1_rotate)){
      dsp_message = "wk_j1_rotate 指定可能範圍外！"
      return {j1_rotate,j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
    }

    // 5. 计算J4、J5角度，确保手腕姿态与目标一致
    // 通过空间几何关系和三角学公式推导，先构造前级关节的变换矩阵
    const mtx_j2 = new THREE.Matrix4().multiply(
      new THREE.Matrix4().makeRotationY(toRadian(wk_j1_rotate)).setPosition(joint_pos.j1.x,joint_pos.j1.y,joint_pos.j1.z)
    ).multiply(
      new THREE.Matrix4().makeRotationX(toRadian(wk_j2_rotate)).setPosition(joint_pos.j2.x,joint_pos.j2.y,joint_pos.j2.z)
    )
    const j2_pos = new THREE.Vector3().applyMatrix4(mtx_j2)

    const mtx_j3 = mtx_j2.clone().multiply(
      new THREE.Matrix4().makeRotationX(toRadian(wk_j3_rotate)).setPosition(joint_pos.j3.x,joint_pos.j3.y,joint_pos.j3.z)
    )
    const j3_pos = new THREE.Vector3().applyMatrix4(mtx_j3)

    const mtx_j4 = mtx_j3.clone().multiply(
      new THREE.Matrix4().setPosition(joint_pos.j4.x,joint_pos.j4.y,joint_pos.j4.z)
    )
    const j4_pos = new THREE.Vector3().applyMatrix4(mtx_j4)

    // 计算手腕末端的理论位置，用于后续J5/J4推导
    const mtx_j3_wk = mtx_j3.clone().multiply(
      new THREE.Matrix4().setPosition(0,0,joint_pos.j4.z)
    )
    const j3_pos_wk = new THREE.Vector3().applyMatrix4(mtx_j3_wk)

    // 计算J5三角形的角度
    const distance_13_16 = (distance(j3_pos_wk,p16_pos))
    let j5_angle_C = 180
    if((joint_pos.j4.y + p15_16_len) > distance_13_16){
      j5_angle_C = degree3(joint_pos.j4.y,p15_16_len,distance_13_16).angle_C
    }
    if(isNaN(j5_angle_C)){
      dsp_message = "j5_angle_C 指定可能範囲外！"
      return {j1_rotate:wk_j1_rotate,j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,
        j4_rotate,j5_rotate,j6_rotate,dsp_message}
    }
    let j5_base = (180 - j5_angle_C)
    // 判断手腕朝向，决定J5正负
    const j3_arm_angle = round((wk_j2_rotate+wk_j3_rotate),10)
    const judge_wrist_angle = round(wrist_angle,10)
    if(j3_arm_angle < 90){
      if(j3_arm_angle > judge_wrist_angle){
        j5_base = j5_base*-1
      }
    }else
    if(j3_arm_angle > 90){
      if(j3_arm_angle < judge_wrist_angle){
        j5_base = j5_base*-1
      }
    }
    let wk_j5_rotate = normalize180((j5_base - 90))

    // 计算J5/J4的空间变换，推导J4角度
    const mtx_j5_zero = mtx_j4.clone().multiply(
      new THREE.Matrix4().makeRotationX(toRadian(wk_j5_rotate)).setPosition(joint_pos.j5.x,joint_pos.j5.y,joint_pos.j5.z)
    ).multiply(
      new THREE.Matrix4().setPosition(joint_pos.j6.x,joint_pos.j6.y,joint_pos.j6.z)
    ).multiply(
      new THREE.Matrix4().setPosition(joint_pos.j7.x,joint_pos.j7.y,p15_16_len)
    )
    const p16_zero_pos = new THREE.Vector3().applyMatrix4(mtx_j5_zero)

    // 计算J5三角形的中心点
    const j5_tri_wk = calc_side_1(p15_16_len,Math.abs(j5_base))
    const mtx_j5_center = mtx_j4.clone().multiply(
      new THREE.Matrix4().setPosition(0,j5_tri_wk.a,0)
    )
    const j5_center_pos = new THREE.Vector3().applyMatrix4(mtx_j5_center)

    // 通过空间角度关系推导J4角度
    const wk_j4_angle_C = toAngle(p16_zero_pos.clone().sub(j5_center_pos).angleTo(p16_pos.clone().sub(j5_center_pos)))
    const direction_offset = normalize180(wrist_direction - wk_j1_rotate)
    const j4_base = wk_j4_angle_C * (direction_offset<0?-1:1)
    let wk_j4_rotate = normalize180((j4_base))  //*(j5_minus?-1:1)

    // 6. 通过四元数差值，推导J6（末端旋转）角度
    // 先构造从基座到J5的四元数
    const baseq = new THREE.Quaternion().multiply(
      new THREE.Quaternion().setFromAxisAngle(y_vec_base,toRadian(wk_j1_rotate))
    ).multiply(
      new THREE.Quaternion().setFromAxisAngle(x_vec_base,toRadian(wk_j2_rotate))
    ).multiply(
      new THREE.Quaternion().setFromAxisAngle(x_vec_base,toRadian(wk_j3_rotate))
    ).multiply(
      new THREE.Quaternion().setFromAxisAngle(y_vec_base,toRadian(wk_j4_rotate))
    ).multiply(
      new THREE.Quaternion().setFromAxisAngle(x_vec_base,toRadian(wk_j5_rotate))
    ).multiply(
      new THREE.Quaternion().setFromAxisAngle(z_vec_base,Math.PI)
    )
    // 目标手腕四元数
    const j5q = get_j5_quaternion()
    // 计算当前姿态与目标姿态的四元数差
    const p14_j5_diff = quaternionDifference(baseq,j5q)
    const p14_j5_diff_angle = quaternionToAngle(p14_j5_diff)
    // J6角度由四元数差的旋转轴方向决定正负
    const wk_j6_rotate = p14_j5_diff_angle.angle * ((p14_j5_diff_angle.axis.z < 0)?-1:1)
    
    // 7. 返回所有关节角度和错误信息
    return {
      j1_rotate: wk_j1_rotate,
      j2_rotate: wk_j2_rotate,
      j3_rotate: wk_j3_rotate,
      j4_rotate: wk_j4_rotate,
      j5_rotate: wk_j5_rotate,
      j6_rotate: wk_j6_rotate,
      dsp_message
    };
  }

  
  /**
 * 根据三角形三边和基角，推导机械臂 J2、J3 两个关节的旋转角度
 * 
 * @param {number} angle_base - 基角（通常为前级关节或空间夹角）
 * @param {number} side_a - 三角形边a长度（如上一关节到下一关节的连杆长度）
 * @param {number} side_b - 三角形边b长度（如下一关节到末端的连杆长度）
 * @param {number} side_c - 三角形边c长度（如基点到目标点的距离）
 * @returns {Object} 
 *   {
 *     j2_rotate, // J2关节角度
 *     j3_rotate, // J3关节角度
 *     dsp_message // 错误信息（如有）
 *   }
 * 
 * 主要流程说明：
 * 1. 判断三边能否构成三角形，若不能则直接返回极限角度。
 * 2. 若能构成三角形，利用余弦定理计算各内角。
 * 3. 由基角和三角形内角推导出 J2、J3 的实际旋转角度。
 * 4. 若计算结果超出物理极限或出现NaN，返回错误信息。
 * 
 * 注意：此函数为逆运动学的关键步骤，直接影响机械臂的可达性和解算精度。
 */
const get_J2_J3_rotate = (angle_base, side_a, side_b, side_c) => {
  let dsp_message = undefined
  const max_dis = side_a + side_b
  const min_dis = Math.abs(side_a - side_b)

  let wk_j2_rotate = 0
  let wk_j3_rotate = 0
  // 1. 边长无法构成三角形（目标点过近或过远），直接返回极限角度
  if(min_dis > side_c){
    wk_j2_rotate = angle_base
    wk_j3_rotate = 180
  }else
  if(side_c >= max_dis){
    wk_j2_rotate = angle_base
    wk_j3_rotate = 0
  }else{
    // 2. 正常三角形，利用余弦定理计算各内角
    const result = degree3(side_a, side_b, side_c)
    let {angle_B, angle_C} = result

    if(isNaN(angle_B)) angle_B = 0
    if(isNaN(angle_C)) angle_C = 0

    // 3. 推导J2、J3角度
    const angle_j2 = normalize180((angle_base - angle_B))
    const angle_j3 = normalize180((angle_C === 0 ? 0 : 180 - angle_C))
    if(isNaN(angle_j2)){
      console.log("angle_j2 指定可能範圍外！")
      dsp_message = "angle_j2 指定可能範圍外！"
      wk_j2_rotate = j2_rotate
    }else{
      wk_j2_rotate = angle_j2
    }
    if(isNaN(angle_j3)){
      console.log("angle_j3 指定可能範圍外！")
      dsp_message = "angle_j3 指定可能範圍外！"
      wk_j3_rotate = j3_rotate
    }else{
      wk_j3_rotate = angle_j3
    }
  }
  // 4. J3角度需加上末端连杆的偏移修正
  const j4_sabun = calc_side_2(-joint_pos.j4.z, joint_pos.j4.y)
  wk_j3_rotate = wk_j3_rotate + j4_sabun.k
  return {j2_rotate:wk_j2_rotate, j3_rotate:wk_j3_rotate, dsp_message}
}


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

  // A-Frame 组件属性设置
  // 用 useCallback 包装函数，避免不必要的重新渲染
  const edit_pos = React.useCallback((posxyz) => `${posxyz.x} ${posxyz.y} ${posxyz.z}`, []);

  // 用 useMemo 进行渲染性能优化，缓存机器人属性
  const robotProps = React.useMemo(() => ({
    robotNameList, robotName, joint_pos, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate, j7_rotate,
    toolNameList, toolName, cursor_vis, box_vis, edit_pos, pos_add, 
    j1_error: jointErrorRef.current[0], 
    j2_error: jointErrorRef.current[1], 
    j3_error: jointErrorRef.current[2], 
    j4_error: jointErrorRef.current[3], 
    j5_error: jointErrorRef.current[4]
  }), [
    robotNameList, robotName, joint_pos, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate, j7_rotate,
    toolNameList, toolName, cursor_vis, box_vis, edit_pos, pos_add, 
    jointErrorRef.current[0], jointErrorRef.current[1], jointErrorRef.current[2], jointErrorRef.current[3], jointErrorRef.current[4]
  ]);

  // 控制器属性 useMemo 优化
  const controllerProps = React.useMemo(() => ({
    robotName, robotNameList, set_robotName,
    target, set_target,
    toolName, toolNameList, set_toolName,
    j1_rotate,set_j1_rotate,j2_rotate,set_j2_rotate,j3_rotate,set_j3_rotate,
    j4_rotate,set_j4_rotate,j5_rotate,set_j5_rotate,j6_rotate,set_j6_rotate,j7_rotate,set_j7_rotate,
    c_pos_x,set_c_pos_x,c_pos_y,set_c_pos_y,c_pos_z,set_c_pos_z,
    c_deg_x,set_c_deg_x,c_deg_y,set_c_deg_y,c_deg_z,set_c_deg_z,
    wrist_rot_x,set_wrist_rot_x,wrist_rot_y,set_wrist_rot_y,wrist_rot_z,set_wrist_rot_z,
    tool_rotate,set_tool_rotate,normalize180,vr_mode:vrModeRef.current,
    selectedMode, setSelectedMode
  }), [
    robotName, robotNameList, set_robotName,
    target, set_target,
    toolName, toolNameList, set_toolName,
    j1_rotate,set_j1_rotate,j2_rotate,set_j2_rotate,j3_rotate,set_j3_rotate,
    j4_rotate,set_j4_rotate,j5_rotate,set_j5_rotate,j6_rotate,set_j6_rotate,j7_rotate,set_j7_rotate,
    c_pos_x,set_c_pos_x,c_pos_y,set_c_pos_y,c_pos_z,set_c_pos_z,
    c_deg_x,set_c_deg_x,c_deg_y,set_c_deg_y,c_deg_z,set_c_deg_z,
    wrist_rot_x,set_wrist_rot_x,wrist_rot_y,set_wrist_rot_y,wrist_rot_z,set_wrist_rot_z,
    tool_rotate,set_tool_rotate,normalize180,vrModeRef.current,
    selectedMode, setSelectedMode
  ]);

  // 渲染函数
  // rendered 由 useMqtt 中的 set_rendered 控制，决定是否渲染主场景
  if(rendered){
    return (
      <>
        {/* 主 A-Frame 场景，包含机器人、辅助线、光源等 */}
        <a-scene scene xr-mode-ui="XRMode: ar">
          {/* VR 控制器实体 */}
          <a-entity oculus-touch-controls="hand: right" vr-controller-right visible={true}></a-entity>
          {/* 地面平面，颜色根据目标点错误状态变化 */}
          <a-plane position="0 0 0" rotation="-90 0 0" width="0.4" height="0.4" color={target_error ? "#ff7f50" : "#7BC8A4"} opacity="0.5"></a-plane>
          {/* 机器人模型及工具 */}
          <Assets viewer={props.viewer} monitor={props.monitor}/>
          <Select_Robot {...robotProps}/>
          {/* 辅助线和光源 */}
          <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 1"></a-entity>
          <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="-1 1 1"></a-entity>
          <a-entity light="type: directional; color: #EEE; intensity: 0.25" position="-1 1 -1"></a-entity>
          <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 -1"></a-entity>
          <a-entity light="type: directional; color: #EFE; intensity: 0.1" position="0 -1 0"></a-entity>
          {/* 摄像机与 UI 提示 */}
          <a-entity id="rig" position={`${c_pos_x} ${c_pos_y} ${c_pos_z}`} rotation={`${c_deg_x} ${c_deg_y} ${c_deg_z}`}>
            <a-camera id="camera" cursor="rayOrigin: mouse;" position="0 0 0">
              <a-entity jtext={`text: ${dsp_message}; color: black; background:rgb(31, 219, 131); border: #000000`} position="0 0.7 -1.4"></a-entity>
            </a-camera>
          </a-entity>
          {/* 目标点与测试点可视化 */}
          <a-sphere position={edit_pos(target)} scale="0.012 0.012 0.012" color={target_error ? "red" : "yellow"} visible={true}></a-sphere>
          <a-box position={edit_pos(test_pos)} scale="0.03 0.03 0.03" color="green" visible={box_vis}></a-box>
          {/* 辅助十字线 */}
          <Line pos1={{x:1,y:0.0001,z:1}} pos2={{x:-1,y:0.0001,z:-1}} visible={cursor_vis} color="white"></Line>
          <Line pos1={{x:1,y:0.0001,z:-1}} pos2={{x:-1,y:0.0001,z:1}} visible={cursor_vis} color="white"></Line>
          <Line pos1={{x:1,y:0.0001,z:0}} pos2={{x:-1,y:0.0001,z:0}} visible={cursor_vis} color="white"></Line>
          <Line pos1={{x:0,y:0.0001,z:1}} pos2={{x:0,y:0.0001,z:-1}} visible={cursor_vis} color="white"></Line>
        </a-scene>
        {/* 控制器 UI */}
        <Controller {...controllerProps}/>
        {/* 底部状态栏，显示当前手腕姿态、关节角度和提示信息 */}
        <div className="footer">
          <div>{`wrist_degree:{direction:${wrist_degree.direction},angle:${wrist_degree.angle}}  ${dsp_message}  outdeg[j1:${rotate[0]}, j2:${rotate[1]}, j3:${rotate[2]}, j4:${rotate[3]}, j5:${rotate[4]}, j6:${rotate[5]}, grip:${rotate[6]}]`}</div>
        </div>
      </>
    );
  }else{
    // 未渲染时只显示资源加载
    return(
      <a-scene xr-mode-ui="XRMode: ar">
        <Assets viewer={props.viewer}/>
      </a-scene>
    )
  }
}
