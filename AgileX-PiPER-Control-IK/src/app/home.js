"use client";
import 'aframe'
import * as React from 'react'
//import * as THREE from 'three' // これだと、THREE のインスタンスが複数になり問題
const THREE = window.AFRAME.THREE; // これで　AFRAME と　THREEを同時に使える
import 'aframe-troika-text';

import Controller from './controller.js'

import { connectMQTT, mqttclient,idtopic,subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'
const MQTT_REQUEST_TOPIC = "mgr/request";
const MQTT_DEVICE_TOPIC = "dev/"+idtopic;
const MQTT_CTRL_TOPIC =        "control/"+idtopic; // 自分のIDに制御を送信
const MQTT_ROBOT_STATE_TOPIC = "robot/"; // Viwer のばあい
let receive_state = false // ロボットの状態を受信してるかのフラグ

const joint_pos = {
  base:{x:0,y:0,z:0},
  j1:{x:0,y:0,z:0},
  j2:{x:0,y:0.1212,z:0},
  j3:{x:0,y:0.28503,z:0},
  j4:{x:0,y:0.25,z:-0.02194},
  j5:{x:0,y:0,z:0},
  j6:{x:0,y:0,z:0},
  j7:{x:0,y:0,z:0.225},
}

let registered = false
const order = 'ZYX'

const x_vec_base = new THREE.Vector3(1,0,0).normalize()
const y_vec_base = new THREE.Vector3(0,1,0).normalize()
const z_vec_base = new THREE.Vector3(0,0,1).normalize()

let start_rotation = new THREE.Euler(0.6654549523360951,0,0,order)
let save_rotation = new THREE.Euler(0.6654549523360951,0,0,order)
let current_rotation = new THREE.Euler(0.6654549523360951,0,0,order)
const max_move_unit = (1/720)
const rotate_table = [[],[],[],[],[],[]]
const object3D_table = []
const rotvec_table = [y_vec_base,x_vec_base,x_vec_base,y_vec_base,x_vec_base,z_vec_base]
let target_move_distance = 0.2
const target_move_speed = (1000/2)
let real_target = {x:0,y:0.19647,z:-0.195}

const wrist_degree = {direction:0,angle:0}

const j1_Correct_value = 180
const j2_Correct_value = (90-10.784)
const j3_Correct_value = (-180+10.784)
const j4_Correct_value = 0.0
const j5_Correct_value = 90
const j6_Correct_value = 0.0
const j7_Correct_value = 0.0

let j1_error = false
let j2_error = false
let j3_error = false
let j4_error = false
let j5_error = false

const controller_object_position = new THREE.Vector3()
const controller_object_rotation = new THREE.Euler(0,0,0,order)

export default function DynamicHome(props) {
  //const [tick, setTick] = React.useState(0)
  //const [now, setNow] = React.useState(new Date())
  const [rendered,set_rendered] = React.useState(false)
  const robotNameList = ["Model"]
  const [robotName,set_robotName] = React.useState(robotNameList[0])
  const [cursor_vis,set_cursor_vis] = React.useState(false)
  const [box_vis,set_box_vis] = React.useState(false)
  const [target_error,set_target_error] = React.useState(false)

  const [j1_rotate,set_j1_rotate] = React.useState(0)
  const [j2_rotate,set_j2_rotate] = React.useState(0)
  const [j3_rotate,set_j3_rotate] = React.useState(0)
  const [j4_rotate,set_j4_rotate] = React.useState(0)
  const [j5_rotate,set_j5_rotate] = React.useState(-90)
  const [j6_rotate,set_j6_rotate] = React.useState(0)
  const [j7_rotate,set_j7_rotate] = React.useState(24) //指用

  //const [rotate, set_rotate] = React.useState([0,0,0,0,0,0,0])  //出力用
  const rotateRef = React.useRef([0,0,0,0,0,0,0]); // ref を使って rotate を保持する

  const [input_rotate, set_input_rotate] = React.useState([0,0,0,0,0,0,0])  //入力用

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

  const [trigger_on,set_trigger_on] = React.useState(false)
  const [start_pos,set_start_pos] = React.useState(new THREE.Vector3())
  const [save_target,set_save_target] = React.useState()
  const vrModeRef = React.useRef(false); // vr_mode はref のほうが使いやすい
  //const [save_j3_pos,set_save_j3_pos] = React.useState(undefined)
  //const [save_j4_rot,set_save_j4_rot] = React.useState(undefined)

  const [grip_on, set_grip_on] = React.useState(false);
  const [grip_value, set_grip_value] = React.useState(0);
  const gripRef = React.useRef(false);
  const gripValueRef = React.useRef(0);

  const [button_a_on, set_button_a_on] = React.useState(false);
  const buttonaRef = React.useRef(null);
  const [button_b_on, set_button_b_on] = React.useState(false)
  const buttonbRef = React.useRef(null);
  const [selectedMode, setSelectedMode] = React.useState('control'); //　モード
  const robotIDRef = React.useRef("none"); // ロボットのIDを保持するためのref


  const [test_pos,set_test_pos] = React.useState({x:0,y:0,z:0})

  const [c_pos_x,set_c_pos_x] = React.useState(0)
  const [c_pos_y,set_c_pos_y] = React.useState(0.35)
  const [c_pos_z,set_c_pos_z] = React.useState(0.6)
  const [c_deg_x,set_c_deg_x] = React.useState(0)
  const [c_deg_y,set_c_deg_y] = React.useState(0)
  const [c_deg_z,set_c_deg_z] = React.useState(0)

  const [wrist_rot,set_wrist_rot_org] = React.useState({x:180,y:0,z:0})
  const [tool_rotate,set_tool_rotate] = React.useState(0)
  //const [wrist_degree,set_wrist_degree] = React.useState({direction:0,angle:0})
  const [dsp_message,set_dsp_message] = React.useState("")

  const toolNameList = ["No tool"]
  const [toolName,set_toolName] = React.useState(toolNameList[0])

  const [target,set_target_org] = React.useState(real_target)
  const [p15_16_len,set_p15_16_len] = React.useState(joint_pos.j7.z)
  const [p14_maxlen,set_p14_maxlen] = React.useState(0)

  const [do_target_update, set_do_target_update] = React.useState(0) // count up for each target_update call
  const [vrcontroller_move, set_vrcontroller_move] = React.useState(false)

  // レンダリング毎に表示させる
  /*
  const renderCount = React.useRef(0);
  renderCount.current++;
  console.log('Render count:', renderCount.current);
 */
  /*const reqIdRef = React.useRef()

  const loop = (timestamp)=>{
    setNow(timestamp);
    reqIdRef.current = window.requestAnimationFrame(loop)
  }

  React.useEffect(() => {
    loop(performance.now())
    return () => window.cancelAnimationFrame(reqIdRef.current)
  },[])*/

  const set_target = (new_pos)=>{
    target_move_distance = distance(real_target,new_pos)
    set_target_org(new_pos)
  }

  const set_wrist_rot = (new_rot)=>{
    target_move_distance = 0
    set_wrist_rot_org({...new_rot})
  }

  React.useEffect(() => {
//    console.log("effect controller!",rendered , vrModeRef.current, trigger_on)
    if(rendered && vrModeRef.current && trigger_on){
      const move_pos = pos_sub(start_pos,controller_object_position)
/*
      move_pos.x = move_pos.x/2
      move_pos.y = move_pos.y/2
      move_pos.z = move_pos.z/2
      */
      let target_pos
      if(save_target === undefined){
        set_save_target({...target})
        target_pos = pos_sub(target,move_pos)
      }else{
        target_pos = pos_sub(save_target,move_pos)
      }
      if(target_pos.y < 0.012){ // do not touch ground
        target_pos.y = 0.012
      }
      set_target({x:round(target_pos.x),y:round(target_pos.y),z:round(target_pos.z)})
    }
  },[controller_object_position.x,controller_object_position.y,controller_object_position.z])

  React.useEffect(() => {
    if(rendered && vrModeRef.current && trigger_on){
      const quat_start = new THREE.Quaternion().setFromEuler(start_rotation);
      const quat_controller = new THREE.Quaternion().setFromEuler(controller_object_rotation);
      const quatDifference1 = quat_start.clone().invert().multiply(quat_controller);

      const quat_save = new THREE.Quaternion().setFromEuler(save_rotation);
      const quatDifference2 = quat_start.clone().invert().multiply(quat_save);

      const wk_mtx = quat_start.clone().multiply(quatDifference1).multiply(quatDifference2)
      current_rotation = new THREE.Euler().setFromQuaternion(wk_mtx,controller_object_rotation.order)

      wk_mtx.multiply(
        new THREE.Quaternion().setFromEuler(
          new THREE.Euler(
            (0.6654549523360951*-1),  //x
            Math.PI,  //y
            Math.PI,  //z
            controller_object_rotation.order
          )
        )
      )

      const wk_euler = new THREE.Euler().setFromQuaternion(wk_mtx,controller_object_rotation.order)
      set_wrist_rot({x:round(toAngle(wk_euler.x)),y:round(toAngle(wk_euler.y)),z:round(toAngle(wk_euler.z))})
    }
  },[controller_object_rotation.x,controller_object_rotation.y,controller_object_rotation.z])

  // これで、同じレンダリングタイミングでの複数の target_update を回避
  React.useEffect(()=>{
    target_update();
  },[do_target_update])

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

  //React.useEffect(()=>{
  const joint_slerp = () => {
    const flg = props.viewer || props.monitor
    let raw_data = 0
    for(let i=0; i<rotate_table.length; i=i+1){
      const current_table = rotate_table[i]
      const current_object3D = object3D_table[i]
      raw_data = raw_data + current_table.length
      if(current_object3D !== undefined && current_table.length > 0){
        const current_data = current_table[0]
        if(current_data.first){
          current_data.first = false
          current_data.starttime = performance.now()
          current_data.start_quaternion = current_object3D.quaternion.clone()
          current_data.end_quaternion = new THREE.Quaternion().setFromAxisAngle(rotvec_table[i],toRadian(current_data.rot))
          const move_time_1 = target_move_distance*target_move_speed
          const wk_euler = new THREE.Quaternion().angleTo(
            current_data.start_quaternion.clone().invert().multiply(current_data.end_quaternion))
          const move_time_2 = (toAngle(wk_euler)*max_move_unit)*1000
          current_data.move_time = !flg?Math.max(move_time_1,move_time_2):move_time_1
          current_data.endtime = current_data.starttime + current_data.move_time
        }
        const current_time = performance.now()
        if(current_time < current_data.endtime){
          const elapsed_time = current_time - current_data.starttime
          current_object3D.quaternion.slerpQuaternions(
            current_data.start_quaternion,current_data.end_quaternion,(elapsed_time/current_data.move_time))
        }else{
          current_object3D.quaternion.copy(current_data.end_quaternion)
          current_table.shift()
        }
      }
    }
    if(raw_data > 0){
      requestAnimationFrame(joint_slerp)
      //setTimeout(()=>{joint_slerp()},0)
    }
  }
  //}, [now])

  React.useEffect(() => {
    if(rotate_table[0].length > 1){
      rotate_table[0].pop()
    }
    rotate_table[0].push({rot:j1_rotate,first:true})
  }, [j1_rotate])

  React.useEffect(() => {
    if(rotate_table[1].length > 1){
      rotate_table[1].pop()
    }
    rotate_table[1].push({rot:j2_rotate,first:true})
  }, [j2_rotate])

  React.useEffect(() => {
    if(rotate_table[2].length > 1){
      rotate_table[2].pop()
    }
    rotate_table[2].push({rot:j3_rotate,first:true})
  }, [j3_rotate])

  React.useEffect(() => {
    if(rotate_table[3].length > 1){
      rotate_table[3].pop()
    }
    rotate_table[3].push({rot:j4_rotate,first:true})
  }, [j4_rotate])

  React.useEffect(() => {
    if(rotate_table[4].length > 1){
      rotate_table[4].pop()
    }
    rotate_table[4].push({rot:j5_rotate,first:true})
  }, [j5_rotate])

  React.useEffect(() => {
    if(rotate_table[5].length > 1){
      rotate_table[5].pop()
    }
    rotate_table[5].push({rot:j6_rotate,first:true})
  }, [j6_rotate])

  React.useEffect(() => {
    requestAnimationFrame(joint_slerp)
    //setTimeout(()=>{joint_slerp()},0)
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
      //set_rotate(new_rotate)
      rotateRef.current = new_rotate
//      console.log("New Rotate",new_rotate)
//    }
  }, [j1_rotate,j2_rotate,j3_rotate,j4_rotate,j5_rotate,j6_rotate,j7_rotate])

  React.useEffect(() => {
    if (object3D_table[0] !== undefined) {
      target_move_distance = 0
      const rotate_value = normalize180(input_rotate[0]-j1_Correct_value)
      set_j1_rotate(rotate_value)
    }
  }, [input_rotate[0]])

  React.useEffect(() => {
    if (object3D_table[1] !== undefined) {
      target_move_distance = 0
      const rotate_value = normalize180(input_rotate[1]-j2_Correct_value)
      set_j2_rotate(rotate_value)
    }
  }, [input_rotate[1]])

  React.useEffect(() => {
    if (object3D_table[2] !== undefined) {
      target_move_distance = 0
      const rotate_value = normalize180(input_rotate[2]-j3_Correct_value)
      set_j3_rotate(rotate_value)
    }
  }, [input_rotate[2]])

  React.useEffect(() => {
    if (object3D_table[3] !== undefined) {
      target_move_distance = 0
      const rotate_value = normalize180(input_rotate[3]-j4_Correct_value)
      set_j4_rotate(rotate_value)
    }
  }, [input_rotate[3]])

  React.useEffect(() => {
    if (object3D_table[4] !== undefined) {
      target_move_distance = 0
      const rotate_value = normalize180(input_rotate[4]-j5_Correct_value)
      set_j5_rotate(rotate_value)
    }
  }, [input_rotate[4]])

  React.useEffect(() => {
    if (object3D_table[5] !== undefined) {
      target_move_distance = 0
      const rotate_value = normalize180(input_rotate[5]-j6_Correct_value)
      set_j6_rotate(rotate_value)
    }
  }, [input_rotate[5]])

  React.useEffect(() => {
    if(rendered){
      const rotate_value = input_rotate[6]
      set_j7_rotate(rotate_value)
    }
  }, [input_rotate[6]])


  const requestRobot = (mqclient) =>{
        // 制御対象のロボットを探索（表示された時点で実施）
        const requestInfo = {
          devId: idtopic, // 自分のID
          type: codeType,  //  コードタイプ（Request でマッチングに利用)
        }
        console.log("Publish request",requestInfo)
        publishMQTT(MQTT_REQUEST_TOPIC, JSON.stringify(requestInfo));
  }
// MQTT の初期設定
// MetaworkMQTT protocol
  // register to MQTT
  React.useEffect(() => {
    if (typeof window.mqttClient === 'undefined') {
      //サブスクライブするトピックの登録
      window.mqttClient = connectMQTT(requestRobot);
      subscribeMQTT([
        MQTT_DEVICE_TOPIC
      ]);

      if(props.viewer){
        //サブスクライブ時の処理
        window.mqttClient.on('message', (topic, message) => {
          if (topic == MQTT_DEVICE_TOPIC){ // デバイスへの連絡用トピック
            console.log(" MQTT Device Topic: ", message.toString());
              // ここでは Viewer の設定を実施！
            let data = JSON.parse(message.toString())
            if (data.controller != undefined) {// コントローラ情報ならば！
              robotIDRef.current = data.devId
              subscribeMQTT([
                "control/"+data.devId
              ]);
            }
          }else if (topic == "control/"+robotIDRef.current){
            let data = JSON.parse(message.toString())
            if (data.joints != undefined) {
              set_input_rotate(data.joints) // Viwer の場合
            }
          }
        })
      }else{// not viewer 
        //自分向けメッセージサブスクライブ処理
        window.mqttClient.on('message', (topic, message) => {
          if (topic === MQTT_DEVICE_TOPIC){ // デバイスへの連絡用トピック
            let data = JSON.parse(message.toString())
            console.log(" MQTT Device Topic: ", message.toString());
            if (data.devId === "none") {
              console.log("Can't find robot!")
            }else{
              robotIDRef.current = data.devId 
              if (receive_state == false ){ // ロボットの姿勢を受け取るまで、スタートしない。
                subscribeMQTT([
                  MQTT_ROBOT_STATE_TOPIC+robotIDRef.current // ロボットの姿勢を待つ
                ])
              }
            }
          }
          if (topic === MQTT_ROBOT_STATE_TOPIC+robotIDRef.current){ // ロボットの姿勢を受け取ったら
            let data = JSON.parse(message.toString()) ///
            const joints = data.joints
            // ここで、本来は joints の安全チェックをすべき
            if (!receive_state){
              if(props.monitor ===undefined){  // モニターじゃなかったら
                mqttclient.unsubscribe(MQTT_ROBOT_STATE_TOPIC+robotIDRef.current) // これでロボット姿勢の受信は終わり
                receive_state = true;
              }
              console.log(joints)
              set_input_rotate(joints)
            
              window.setTimeout(()=>{
                // まず IK の結果を自分の位置に設定
                // p16_objectに位置があるはず
                if (target_p16_ref.current !== null){
                  const obj = target_p16_ref.current
                  const p16_pos = obj.getWorldPosition(new THREE.Vector3())
                  const p16_quat = obj.getWorldQuaternion(new THREE.Quaternion())
                  const p16_euler = new THREE.Euler().setFromQuaternion(p16_quat,order)
//                  console.log("P16 ",p16_pos,p16_quat)

                  // target を設定
                  set_target_org({x:p16_pos.x,y:p16_pos.y,z:p16_pos.z})
                  set_wrist_rot({x:round(toAngle(p16_euler.x)),y:round(toAngle(p16_euler.y)),z:round(toAngle(p16_euler.z))})

                }else{
                  console.log("P16 object is null!")
                  receive_state = false
                }
                if(props.monitor === undefined){ // モニターじゃなかったら
  
                  publishMQTT("dev/"+robotIDRef.current, JSON.stringify({controller: "browser", devId: idtopic})) // 自分の topic を教える
                }
              }, 500);// 500msec 後に自分の位置を取得する
            
            }


          }

 
  
        })
      }
    }
    // 消える前にイベントを呼びたい
    window.addEventListener('beforeunload', handleBeforeUnload);
    return () => {
      window.removeEventListener('beforeunload', handleBeforeUnload);
    }
  }, [])

  const handleBeforeUnload = () => {
    if (mqttclient != undefined) {
      publishMQTT("mgr/unregister", JSON.stringify({ devId: idtopic }));
    }
  }



  const get_j5_quaternion = (rot_x=wrist_rot.x,rot_y=wrist_rot.y,rot_z=wrist_rot.z)=>{
    return new THREE.Quaternion().setFromEuler(
      new THREE.Euler(toRadian(rot_x), toRadian(rot_y), toRadian(rot_z), order)
    )
  }

  const get_p21_pos = ()=>{
    const j5q = get_j5_quaternion()
    const p21_pos = quaternionToRotation(j5q,{x:0,y:0,z:p15_16_len})
    return p21_pos
  }

  React.useEffect(() => {
    if(rendered){
      set_do_target_update((prev) => prev + 1) // increment the counter to trigger target_update

      if(p51_object)p51_object.quaternion.copy(get_j5_quaternion())
  
    }
  },[target.x,target.y,target.z,tool_rotate,wrist_rot.x,wrist_rot.y,wrist_rot.z])

  const quaternionToRotation = (q,v)=>{
    const q_original = new THREE.Quaternion(q.x, q.y, q.z, q.w)
    const q_conjugate = new THREE.Quaternion(q.x, q.y, q.z, q.w).conjugate()
    const q_vector = new THREE.Quaternion(v.x, v.y, v.z, 0)
    const result = q_original.multiply(q_vector).multiply(q_conjugate)
    return new THREE.Vector3((result.x),(result.y),(result.z))
  }

  const quaternionToAngle = (q)=>{
    const wk_angle = 2 * Math.acos(round(q.w))
    if(wk_angle === 0){
      return {angle:(toAngle(wk_angle)),axis:new THREE.Vector3(0,0,0)}
    }
    const angle = (toAngle(wk_angle))
    const sinHalfAngle = Math.sqrt(1 - q.w * q.w)
    if (sinHalfAngle > 0) {
      const axisX = (q.x / sinHalfAngle)
      const axisY = (q.y / sinHalfAngle)
      const axisZ = (q.z / sinHalfAngle)
      return {angle,axis:new THREE.Vector3(axisX,axisY,axisZ)}
    }else{
      return {angle,axis:new THREE.Vector3(0,0,0)}
    }
  }

  const quaternionDifference = (q1,q2)=>{
    return new THREE.Quaternion(q1.x, q1.y, q1.z, q1.w).invert().multiply(
      new THREE.Quaternion(q2.x, q2.y, q2.z, q2.w)
    )
  }

  const direction_angle = (vec)=>{
    const dir_sign1 = vec.x < 0 ? -1 : 1
    const xz_vector = new THREE.Vector3(vec.x,0,vec.z).normalize()
    const direction = (toAngle(Math.acos(xz_vector.dot(z_vec_base))))*dir_sign1
    const y_vector = new THREE.Vector3(vec.x,vec.y,vec.z).normalize()
    const angle = (toAngle(Math.acos(y_vector.dot(y_vec_base))))
    return {direction,angle}
  }

  const pos_add = (pos1, pos2)=>{
    return {x:(pos1.x + pos2.x), y:(pos1.y + pos2.y), z:(pos1.z + pos2.z)}
  }

  const pos_sub = (pos1, pos2)=>{
    return {x:(pos1.x - pos2.x), y:(pos1.y - pos2.y), z:(pos1.z - pos2.z)}
  }

  const degree3 = (side_a, side_b, side_c)=>{
    const angle_A = (toAngle(Math.acos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))))
    const angle_B = (toAngle(Math.acos((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c))))
    const angle_C = (toAngle(Math.acos((side_a ** 2 + side_b ** 2 - side_c ** 2) / (2 * side_a * side_b))))
    return {angle_A,angle_B,angle_C}
  }

  const target_update = ()=>{
    const p21_pos = get_p21_pos()
    const {direction,angle} = direction_angle(p21_pos)
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
//    console.log("set wrist!", direction, angle, do_target_update)

    //set_wrist_degree({direction,angle})
    wrist_degree.direction = direction
    wrist_degree.angle = angle

    target15_update(direction,angle)
  }

  const target15_update = (wrist_direction,wrist_angle)=>{
    let dsp_message = ""
    const shift_target = {...target}
    let save_target = {...target}
    let result_rotate = {j1_rotate,j2_rotate,j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
    let save_distance = undefined
    let save_distance_cnt = 0
    let save_rotate = {...result_rotate}
    let j3_pos_wk = new THREE.Vector3()
    j1_error = false
    j2_error = false
    j3_error = false
    j4_error = false
    j5_error = false

    for(let i=0; i<10; i=i+1){
      set_test_pos({...shift_target})
      result_rotate = get_all_rotate(shift_target,wrist_direction,wrist_angle)
      if(result_rotate.dsp_message){
        dsp_message = result_rotate.dsp_message
        console.log(dsp_message)
        set_target_error(true)
      }

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
      if(round(sabun_distance) <= 0.0001){
        save_target = {...result_target}
        break
      }
      if(save_distance === undefined){
        save_distance = sabun_distance
      }else{
        if(save_distance < sabun_distance){
          save_distance_cnt = save_distance_cnt + 1
          if(save_distance_cnt > 1){
            if(round(sabun_distance) <= 0.001){
              result_rotate = {...save_rotate}
              console.log("姿勢制御困難！")
              save_target = {...result_target}
              break  
            }
            console.log("姿勢制御不可！")
            set_dsp_message("姿勢制御不可！")
            console.log(`result_target:{x:${result_target.x}, y:${result_target.y}, z:${result_target.z}}`)
            set_target_error(true)
            //set_save_j3_pos(undefined)
            //set_save_j4_rot(undefined)
            return
          }
        }
        save_distance = sabun_distance
        save_rotate = {...result_rotate}
      }
      shift_target.x = shift_target.x + sabun_pos.x
      shift_target.y = shift_target.y + sabun_pos.y
      shift_target.z = shift_target.z + sabun_pos.z
    }

    //ロボットの各関節の可動可能範囲を確認する
    if(dsp_message === ""){
      const wk_j1_rotate = normalize180(result_rotate.j1_rotate + j1_Correct_value)
      if(wk_j1_rotate<-165 || wk_j1_rotate>165){
        dsp_message = `j1_rotate 指定可能範囲外！:(${wk_j1_rotate})`
        j1_error = true
      }
      const wk_j2_rotate = result_rotate.j2_rotate + j2_Correct_value
      if(wk_j2_rotate<-5 || wk_j2_rotate>200){
        dsp_message = `j2_rotate 指定可能範囲外！:(${wk_j2_rotate})`
        j2_error = true
      }
      const wk_j3_rotate = result_rotate.j3_rotate + j3_Correct_value
      if(wk_j3_rotate<-180 || wk_j3_rotate>5){
        dsp_message = `j3_rotate 指定可能範囲外！:(${wk_j3_rotate})`
        j3_error = true
      }
      const wk_j4_rotate = result_rotate.j4_rotate + j4_Correct_value
      if(wk_j4_rotate<-110 || wk_j4_rotate>110){
        dsp_message = `j4_rotate 指定可能範囲外！:(${wk_j4_rotate})`
        j4_error = true
      }
      const wk_j5_rotate = result_rotate.j5_rotate + j5_Correct_value
      if(wk_j5_rotate<-80 || wk_j5_rotate>80){
        dsp_message = `j5_rotate 指定可能範囲外！:(${wk_j5_rotate})`
        j5_error = true
      }
    }

    /*if(dsp_message === ""){
      if(save_j3_pos === undefined){
        set_save_j3_pos(j3_pos_wk)
      }else{
        const move_distance = distance(save_j3_pos,j3_pos_wk)
        if(move_distance > 0.25){
          dsp_message = `j3_pos 急旋回指示！:(${move_distance})`
          console.log(dsp_message)
        }else{
          set_save_j3_pos(j3_pos_wk)
        }
      }
      if(save_j4_rot === undefined){
        set_save_j4_rot(result_rotate.j4_rotate)
      }else{
        const j4_rot_diff = Math.abs(save_j4_rot - round(result_rotate.j4_rotate))
        if(j4_rot_diff > 120){
          dsp_message = `j4_rotate 急旋回指示！:(${j4_rot_diff})`
          console.log(dsp_message)
        }else{
          set_save_j4_rot(result_rotate.j4_rotate)
        }
      }
    }*/

    if(dsp_message === ""){
      set_target_error(false)
      set_j1_rotate(round(result_rotate.j1_rotate))
      set_j2_rotate(round(result_rotate.j2_rotate))
      set_j3_rotate(round(result_rotate.j3_rotate))
      set_j4_rotate(round(result_rotate.j4_rotate))
      set_j5_rotate(round(result_rotate.j5_rotate))
      set_j6_rotate(normalize180(round(result_rotate.j6_rotate + tool_rotate)))
      real_target = {...save_target}
    }else{
      set_target_error(true)
    }
    set_dsp_message(dsp_message)
  }

  const get_all_rotate = (final_target,wrist_direction,wrist_angle)=>{
    let dsp_message = ""
    const p16_pos = new THREE.Vector3(final_target.x,final_target.y,final_target.z)
    const p15_16_offset_pos = get_p21_pos()
    const p15_pos_wk = pos_sub(p16_pos,p15_16_offset_pos)
    const p15_pos = new THREE.Vector3(p15_pos_wk.x,p15_pos_wk.y,p15_pos_wk.z)

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

    const syahen_t15 = distance(joint_pos.j2,p15_pos)
    const takasa_t15 = (p15_pos.y - joint_pos.j2.y)
    const {k:angle_t15} = calc_side_4(syahen_t15,takasa_t15)
    const distance_j3 = distance({x:0,y:0,z:0},joint_pos.j3)
    const distance_j4 = distance({x:0,y:0,z:0},joint_pos.j4)
    const result_t15 = get_J2_J3_rotate(angle_t15*(back?-1:1),distance_j3,distance_j4,syahen_t15)
    if(result_t15.dsp_message){
      dsp_message = result_t15.dsp_message
      return {j1_rotate,j2_rotate,j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
    }
    const wk_j2_rotate = result_t15.j2_rotate
    const wk_j3_rotate = result_t15.j3_rotate

    const dir_sign_t15 = p15_pos.x < 0 ? -1 : 1
    const xz_vector_t15 = new THREE.Vector3(p15_pos.x,0,p15_pos.z).normalize()
    const direction_t15 = (toAngle(z_vec_base.angleTo(xz_vector_t15)))*dir_sign_t15
    if(isNaN(direction_t15)){
      dsp_message = "direction_t15 指定可能範囲外！"
      return {j1_rotate,j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
    }

    let wk_j1_rotate = normalize180(direction_t15 + (back?180:0))
    if(isNaN(wk_j1_rotate)){
      dsp_message = "wk_j1_rotate 指定可能範囲外！"
      return {j1_rotate,j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,j4_rotate,j5_rotate,j6_rotate,dsp_message}
    }

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

    const mtx_j3_wk = mtx_j3.clone().multiply(
      new THREE.Matrix4().setPosition(0,0,joint_pos.j4.z)
    )
    const j3_pos_wk = new THREE.Vector3().applyMatrix4(mtx_j3_wk)

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

    const wk_j5_rotate_p = normalize180((j5_base - 90))
    const wk_j5_rotate_m = normalize180(((j5_base*-1) - 90))

    const j5_tri_wk = calc_side_1(p15_16_len,Math.abs(j5_base))
    const mtx_j5_center = mtx_j4.clone().multiply(
      new THREE.Matrix4().setPosition(0,j5_tri_wk.a,0)
    )
    const j5_center_pos = new THREE.Vector3().applyMatrix4(mtx_j5_center)

    const direction_offset = normalize180(wrist_direction - wk_j1_rotate)

    const mtx_j5_zero_p = mtx_j4.clone().multiply(
      new THREE.Matrix4().makeRotationX(toRadian(wk_j5_rotate_p)).setPosition(joint_pos.j5.x,joint_pos.j5.y,joint_pos.j5.z)
    ).multiply(
      new THREE.Matrix4().setPosition(joint_pos.j6.x,joint_pos.j6.y,joint_pos.j6.z)
    ).multiply(
      new THREE.Matrix4().setPosition(joint_pos.j7.x,joint_pos.j7.y,p15_16_len)
    )
    const p16_zero_pos_p = new THREE.Vector3().applyMatrix4(mtx_j5_zero_p)

    const wk_j4_angle_C_p = toAngle(p16_zero_pos_p.clone().sub(j5_center_pos).angleTo(p16_pos.clone().sub(j5_center_pos)))
    const j4_base_p = wk_j4_angle_C_p * (direction_offset<0?-1:1)
    const wk_j4_rotate_p = normalize180(j4_base_p)

    let wk_j4_rotate = wk_j4_rotate_p
    let wk_j5_rotate = wk_j5_rotate_p
    if(Math.abs(wk_j4_rotate_p) > 110){
      const mtx_j5_zero_m = mtx_j4.clone().multiply(
        new THREE.Matrix4().makeRotationX(toRadian(wk_j5_rotate_m)).setPosition(joint_pos.j5.x,joint_pos.j5.y,joint_pos.j5.z)
      ).multiply(
        new THREE.Matrix4().setPosition(joint_pos.j6.x,joint_pos.j6.y,joint_pos.j6.z)
      ).multiply(
        new THREE.Matrix4().setPosition(joint_pos.j7.x,joint_pos.j7.y,p15_16_len)
      )
      const p16_zero_pos_m = new THREE.Vector3().applyMatrix4(mtx_j5_zero_m)
  
      const wk_j4_angle_C_m = toAngle(p16_zero_pos_m.clone().sub(j5_center_pos).angleTo(p16_pos.clone().sub(j5_center_pos)))
      const j4_base_m = wk_j4_angle_C_m * (direction_offset<0?1:-1)
      wk_j4_rotate = normalize180(j4_base_m)
      wk_j5_rotate = wk_j5_rotate_m
    }

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
    const j5q = get_j5_quaternion()
    const p14_j5_diff = quaternionDifference(baseq,j5q)
    const p14_j5_diff_angle = quaternionToAngle(p14_j5_diff)
    const wk_j6_rotate = p14_j5_diff_angle.angle * ((p14_j5_diff_angle.axis.z < 0)?-1:1)

    return {j1_rotate:wk_j1_rotate,j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,
      j4_rotate:wk_j4_rotate,j5_rotate:wk_j5_rotate,j6_rotate:wk_j6_rotate,dsp_message}
  }

  const get_J2_J3_rotate = (angle_base,side_a,side_b,side_c)=>{
    let dsp_message = undefined
    const max_dis = side_a + side_b
    const min_dis = Math.abs(side_a - side_b)

    let wk_j2_rotate = 0
    let wk_j3_rotate = 0
    if(min_dis > side_c){
      wk_j2_rotate = angle_base
      wk_j3_rotate = 180
    }else
    if(side_c >= max_dis){
      wk_j2_rotate = angle_base
      wk_j3_rotate = 0
    }else{
      const result = degree3(side_a,side_b,side_c)
      let {angle_B, angle_C} = result

      if(isNaN(angle_B)) angle_B = 0
      if(isNaN(angle_C)) angle_C = 0

      const angle_j2 = normalize180((angle_base - angle_B))
      const angle_j3 = normalize180((angle_C === 0 ? 0 : 180 - angle_C))
      if(isNaN(angle_j2)){
        console.log("angle_j2 指定可能範囲外！")
        dsp_message = "angle_j2 指定可能範囲外！"
        wk_j2_rotate = j2_rotate
      }else{
        wk_j2_rotate = angle_j2
      }
      if(isNaN(angle_j3)){
        console.log("angle_j3 指定可能範囲外！")
        dsp_message = "angle_j3 指定可能範囲外！"
        wk_j3_rotate = j3_rotate
      }else{
        wk_j3_rotate = angle_j3
      }
    }
    const j4_sabun = calc_side_2(-joint_pos.j4.z,joint_pos.j4.y)
    wk_j3_rotate = wk_j3_rotate + j4_sabun.k
    return {j2_rotate:wk_j2_rotate,j3_rotate:wk_j3_rotate,dsp_message}
  }
  
  const round = (x,d=5)=>{
    const v = 10 ** (d|0)
    return Math.round(x*v)/v
  }

  const normalize180 = (angle)=>{
    if(Math.abs(angle) <= 180){
      return angle
    }
    const amari = angle % 180
    if(amari === 0){
      return amari
    }else
    if(amari < 0){
      return (180 + amari)
    }else{
      return (-180 + amari)
    }
  }

  const toAngle = (radian)=>{
    return normalize180(radian*(180/Math.PI))
  }

  const toRadian = (angle)=>{
    return normalize180(angle)*(Math.PI/180)
  }

  const getposq = (parts_obj)=>{
    const mat = parts_obj.matrixWorld
    let position = new THREE.Vector3();
    let quaternion = new THREE.Quaternion();
    let scale = new THREE.Vector3()
    mat.decompose(position, quaternion, scale)
    return {position, quaternion, scale}
  }

  const getpos = (position)=>{
    const wkpos = {x:(position.x),y:(position.y),z:(position.z)}
    return wkpos
  }

  const distance = (s_pos, t_pos)=>{
    return (Math.sqrt((t_pos.x - s_pos.x) ** 2 + (t_pos.y - s_pos.y) ** 2 + (t_pos.z - s_pos.z) ** 2))
  }

  const calc_side_1 = (syahen, kakudo)=>{
    const teihen = (Math.abs(kakudo)===90  ? 0:(syahen * Math.cos(toRadian(kakudo))))
    const takasa = (Math.abs(kakudo)===180 ? 0:(syahen * Math.sin(toRadian(kakudo))))
    return {a:teihen, b:takasa}
  }

  const calc_side_2 = (teihen, takasa)=>{
    const syahen = (Math.sqrt(teihen ** 2 + takasa ** 2))
    const kakudo = (toAngle(Math.atan2(teihen, takasa)))
    return {s:syahen, k:kakudo}
  }

  const calc_side_4 = (syahen, teihen)=>{
    const wk_rad = Math.acos(teihen / syahen)
    const takasa = (teihen * Math.tan(wk_rad))
    const kakudo = (toAngle(wk_rad))
    return {k:kakudo, t:takasa}
  }

  // ロボット姿勢を定常的に送信 
  const onAnimationMQTT = (time) =>{
    const robot_state_json = JSON.stringify({
      time: time,
      joints: rotateRef.current,
      grip: gripRef.current
//        trigger: [gripRef.current, buttonaRef.current, buttonbRef.current, gripValueRef.current]
    });
    publishMQTT(MQTT_ROBOT_STATE_TOPIC+idtopic , robot_state_json);
    window.requestAnimationFrame(onAnimationMQTT);
  }


  // XR のレンダリングフレーム毎に MQTTを呼び出したい
  const onXRFrameMQTT = (time, frame) => {

    if(props.viewer){
      frame.session.requestAnimationFrame(onXRFrameMQTT);
    }else{
      if (vrModeRef.current){// VR_mode じゃなかったら呼び出さない
        frame.session.requestAnimationFrame(onXRFrameMQTT);
        //setNow(time); // VR mode の場合は、通常の AnimationFrame が出ないので、これが必要(loop の代わり)
      }
    }
    

    if ((mqttclient != null) && receive_state) {// 状態を受信していないと、送信しない

      // MQTT 送信
      const ctl_json = JSON.stringify({
        time: time,
        joints: rotateRef.current,
        trigger: [gripRef.current, buttonaRef.current, buttonbRef.current, gripValueRef.current]
      });

      publishMQTT(MQTT_CTRL_TOPIC, ctl_json);
    }

  }


  React.useEffect(() => {
    if(!registered){
      registered = true

      setTimeout(()=>set_rendered(true),10)

      const teihen = joint_pos.j5.x
      const takasa = joint_pos.j3.y + joint_pos.j4.y
      const result = calc_side_2(teihen, takasa)
      set_p14_maxlen(result.s)

      AFRAME.registerComponent('robot-click', {
        init: function () {
          this.el.addEventListener('click', (evt)=>{
            robotChange()
            console.log('robot-click')
          });
        }
      });
      AFRAME.registerComponent('j_id', {
        schema: {type: 'number', default: 0},
        init: function () {
          if(this.data === 1){
            object3D_table[0] = this.el.object3D
          }else
          if(this.data === 2){
            object3D_table[1] = this.el.object3D
          }else
          if(this.data === 3){
            object3D_table[2] = this.el.object3D
          }else
          if(this.data === 4){
            object3D_table[3] = this.el.object3D
          }else
          if(this.data === 5){
            object3D_table[4] = this.el.object3D
          }else
          if(this.data === 6){
            object3D_table[5] = this.el.object3D
          }else
          if(this.data === 11){
            set_p11_object(this.el.object3D)
          }else
          if(this.data === 12){
            set_p12_object(this.el.object3D)
          }else
          if(this.data === 13){
            set_p13_object(this.el.object3D)
          }else
          if(this.data === 14){
            set_p14_object(this.el.object3D)
          }else
          if(this.data === 15){
            set_p15_object(this.el.object3D)
          }else
          if(this.data === 16){
            set_p16_object(this.el.object3D)
            target_p16_ref.current = this.el.object3D //　これでtarget 位置が参照できる
          }else
          if(this.data === 20){
            set_p20_object(this.el.object3D)
          }else
          if(this.data === 21){
            set_p21_object(this.el.object3D)
          }else
          if(this.data === 22){
            set_p22_object(this.el.object3D)
          }else
          if(this.data === 51){
            set_p51_object(this.el.object3D)
          }
        },
        remove: function () {
          if(this.data === 16){
            set_p16_object(this.el.object3D)
          }
        }
      });
//      console.log("Trigger component!")
      AFRAME.registerComponent('vr-controller-right', {
        schema: {type: 'string', default: ''},
        init: function () {
          this.el.object3D.rotation.order = order
          console.log("Set TriggerDown event!")
          this.el.addEventListener('triggerdown', (evt)=>{
            start_rotation = this.el.object3D.rotation.clone()
            const wk_start_pos = new THREE.Vector3().applyMatrix4(this.el.object3D.matrix)
            set_start_pos(wk_start_pos)
            set_trigger_on(true)
          });
          this.el.addEventListener('triggerup', (evt)=>{
            save_rotation = current_rotation.clone()
            set_save_target(undefined)
            set_trigger_on(false)
          });
        },
        tick: function () {
          let move = false
          const obj = this.el.object3D
          if(!controller_object_position.equals(obj.position)){
            controller_object_position.set(obj.position.x,obj.position.y,obj.position.z)
            move = true
          }
          if(!controller_object_rotation.equals(obj.rotation)){
            controller_object_rotation.set(obj.rotation.x,obj.rotation.y,obj.rotation.z,obj.rotation.order)
            move = true
          }
          if(move){
            set_vrcontroller_move((flg)=>!flg)
          }
        }
      });

//      console.log("JText component!")
      // mytext ComponentComponent
      AFRAME.registerComponent('jtext', {
        schema: {
          text: { type: 'string', default: '' },
          width: { type: 'number', default: 1 },
          height: { type: 'number', default: 0.12 },
          color: { type: 'string', default: 'black' },
          background: { type: 'string', default: 'white' },
          border: { type: 'string', default: 'black' }
        },

        init: function () {
          const el = this.el;
          const data = this.data;

          // 背景枠（外側）＝ ボーダー
          /*
          const border = document.createElement('a-plane');
          border.setAttribute('width', data.width + 0.02);
          border.setAttribute('height', data.height + 0.02);
          border.setAttribute('color', data.border);
          border.setAttribute('position', '0 0 0');
            */
          // 背景（内側）
          const bg = document.createElement('a-plane');
          bg.setAttribute('width', data.width);
          bg.setAttribute('height', data.height);
          bg.setAttribute('color', data.background);
          bg.setAttribute('position', '0 0 0.01');
          bg.setAttribute('opacity', '0.8');

          // テキスト// 初期値しか使わないから！
          const text = document.createElement('a-entity');
          console.log("Text:",data.text)
          text.setAttribute('troika-text', {
            value: data.text,
            align: 'center',
            color: data.color,
            fontSize: 0.05,
            maxWidth: data.width * 0.9,
            font: "BIZUDPGothic-Bold.ttf",
          });
          text.setAttribute('position', '0 0 0.01');
          this.text = text;
//          el.appendChild(border);
          el.appendChild(bg);
          el.appendChild(text);
        },
        update: function(oldData){
          const data = this.data;
          this.text.setAttribute('troika-text', {
            value: data.text,
            align: 'center',
            color: data.color,
            fontSize: 0.05,
            maxWidth: data.width * 0.95,
            font: "BIZUDPGothic-Bold.ttf",
          });
          this.text.setAttribute('position', '0 0 0.01');

            //          console.log("update:",oldData,this.data.text)

        },
        setText: function(text){

        }

      });
    
      console.log("Scene component!")
      AFRAME.registerComponent('scene', {
        init: function () {
          if (props.viewer){// viewer は VR モードじゃなくても requestする
            window.requestAnimationFrame(onAnimationMQTT);
          }
          this.el.addEventListener('enter-vr', ()=>{
            vrModeRef.current = true
            console.log('enter-vr')
            
            if(!props.viewer){
              let xrSession = this.el.renderer.xr.getSession();  
              xrSession.requestAnimationFrame(onXRFrameMQTT);
            }

            // ここでカメラ位置を変更します
            set_c_pos_x(0)
            set_c_pos_y(-0.6)
            set_c_pos_z(0.90)
            set_c_deg_x(0)
            set_c_deg_y(0)
            set_c_deg_z(0)
            
          });
          this.el.addEventListener('exit-vr', ()=>{
            vrModeRef.current = false
            console.log('exit-vr')
          });
          
        }
      });
    }
  },[])

  const edit_pos = (posxyz)=>`${posxyz.x} ${posxyz.y} ${posxyz.z}`

  const controllerProps = {
    robotName, robotNameList, set_robotName,
    target, set_target,
    toolName, toolNameList, set_toolName,
    j1_rotate,set_j1_rotate,j2_rotate,set_j2_rotate,j3_rotate,set_j3_rotate,
    j4_rotate,set_j4_rotate,j5_rotate,set_j5_rotate,j6_rotate,set_j6_rotate,j7_rotate,set_j7_rotate,
    c_pos_x,set_c_pos_x,c_pos_y,set_c_pos_y,c_pos_z,set_c_pos_z,
    c_deg_x,set_c_deg_x,c_deg_y,set_c_deg_y,c_deg_z,set_c_deg_z,
    wrist_rot,set_wrist_rot,
    tool_rotate,set_tool_rotate,normalize180,vr_mode:vrModeRef.current,
    selectedMode, setSelectedMode

  }

  const robotProps = {
    robotNameList, robotName, joint_pos, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate, j7_rotate,
    toolNameList, toolName, cursor_vis, box_vis, edit_pos, pos_add, j1_error, j2_error, j3_error, j4_error, j5_error
  }

  if(rendered){
    const rotate = rotateRef.current
    return (
    <>
      <a-scene scene xr-mode-ui="XRMode: ar"  > 
      {/* for AFRAME 1.7.0 upper 
        <a-light type="ambient" color="#ffffff" intensity="0.4"></a-light>
        <a-light type="directional" color="#ffffff" intensity="1" position="1 3 0.5" castShadow="true"> </a-light>
        */}
        <a-entity oculus-touch-controls="hand: right" vr-controller-right visible={`${false}`}></a-entity>
        <a-plane  position="0 0 0" rotation="-90 0 0" width="0.4" height="0.4" color={target_error?"#ff7f50":"#7BC8A4"} opacity="0.5"></a-plane>
        <Assets viewer={props.viewer} monitor={props.monitor}/>
        <Select_Robot {...robotProps}/>
        {/*
        <Cursor3dp j_id="20" pos={{x:0,y:0,z:0}} visible={cursor_vis}>
          <Cursor3dp j_id="21" pos={{x:0,y:0,z:p15_16_len}} visible={cursor_vis}></Cursor3dp>
          <Cursor3dp j_id="22" pos={{x:0,y:-joint_pos.j5.y,z:0}} rot={{x:0,y:j1_rotate,z:0}} visible={cursor_vis}></Cursor3dp>
        </Cursor3dp>
        */}

        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="-1 1 1"></a-entity>
        <a-entity light="type: directional; color: #EEE; intensity: 0.25" position="-1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #EFE; intensity: 0.1" position="0 -1 0"></a-entity>
        <a-entity id="rig" position={`${c_pos_x} ${c_pos_y} ${c_pos_z}`} rotation={`${c_deg_x} ${c_deg_y} ${c_deg_z}`}>
          <a-camera id="camera" cursor="rayOrigin: mouse;" position="0 0 0">
          {/* camera と一緒に情報提示も動かす             */}
            <a-entity jtext={`text: ${dsp_message}; color: black; background:rgb(31, 219, 131); border: #000000`} position="0 0.7 -1.4"></a-entity>
          </a-camera>
        </a-entity>
        <a-sphere position={edit_pos(target)} scale="0.012 0.012 0.012" color={target_error?"red":"yellow"} visible={`${true}`}></a-sphere>
        <a-box position={edit_pos(test_pos)} scale="0.03 0.03 0.03" color="green" visible={`${box_vis}`}></a-box>
        <Line pos1={{x:1,y:0.0001,z:1}} pos2={{x:-1,y:0.0001,z:-1}} visible={cursor_vis} color="white"></Line>
        <Line pos1={{x:1,y:0.0001,z:-1}} pos2={{x:-1,y:0.0001,z:1}} visible={cursor_vis} color="white"></Line>
        <Line pos1={{x:1,y:0.0001,z:0}} pos2={{x:-1,y:0.0001,z:0}} visible={cursor_vis} color="white"></Line>
        <Line pos1={{x:0,y:0.0001,z:1}} pos2={{x:0,y:0.0001,z:-1}} visible={cursor_vis} color="white"></Line>
        {/*<a-cylinder j_id="51" color="green" height="0.1" radius="0.005" position={edit_pos({x:0.3,y:0.3,z:0.3})}></a-cylinder>*/}
      </a-scene>
      <Controller {...controllerProps}/>
      <div className="footer" >
        <div>{`wrist_degree:{direction:${wrist_degree.direction},angle:${wrist_degree.angle}}  ${dsp_message}  outdeg[j1:${rotate[0]}, j2:${rotate[1]}, j3:${rotate[2]}, j4:${rotate[3]}, j5:${rotate[4]}, j6:${rotate[5]}, grip:${rotate[6]}]`}</div>
      </div>
    </>
    );
  }else{
    return(
      <a-scene xr-mode-ui="XRMode: xr"  >
       {/* こちらに scene コンポーネントを置くと、なぜか動かない */} 
       <Assets viewer={props.viewer} monitor={props.monitor}/>
      </a-scene>
    )
  }
}

const Assets = (props)=>{
  const path = (props.viewer|| props.monitor) ?"../":""
  return (
    <a-assets>
      {/*Model*/}
      <a-asset-items id="base" src={`${path}base_link.gltf`} ></a-asset-items>
      <a-asset-items id="j1" src={`${path}link1.gltf`} ></a-asset-items>
      <a-asset-items id="j2" src={`${path}link2.gltf`} ></a-asset-items>
      <a-asset-items id="j3" src={`${path}link3.gltf`} ></a-asset-items>
      <a-asset-items id="j4" src={`${path}link4.gltf`} ></a-asset-items>
      <a-asset-items id="j5" src={`${path}link5.gltf`} ></a-asset-items>
      <a-asset-items id="j6" src={`${path}link6.gltf`} ></a-asset-items>
      <a-asset-items id="j6_1" src={`${path}link7.gltf`} ></a-asset-items>
      <a-asset-items id="j6_2" src={`${path}link8.gltf`} ></a-asset-items>
    </a-assets>
  )
}

const Model = (props)=>{
  const {visible, cursor_vis, edit_pos, joint_pos, pos_add, j1_error, j2_error, j3_error, j4_error, j5_error} = props
  return (<>{visible?
    <a-entity robot-click="" gltf-model="#base" position={edit_pos(joint_pos.base)} visible={`${visible}`}>
      <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: 105; thetaLength: 330" material="color: #00FFFF" position="0 0.086 0" rotation="90 0 0" visible={`${j1_error}`}></a-entity>
      <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: -75; thetaLength: 330" material="color: #00FFFF" position="0 0.086 0" rotation="-90 0 0" visible={`${j1_error}`}></a-entity>
      <a-entity j_id="1" gltf-model="#j1" position={edit_pos(joint_pos.j1)}>
        <a-cylinder position="0 0.086 0.05" rotation="90 0 0" radius="0.005" height="0.1" color="#FF0000" visible={`${j1_error}`}></a-cylinder>
        <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: -5; thetaLength: 205" material="color: #00FFFF" position={edit_pos(pos_add(joint_pos.j2,{x:0.025,y:0,z:0}))} rotation="0 90 0" visible={`${j2_error}`}></a-entity>
        <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: -20; thetaLength: 205" material="color: #00FFFF" position={edit_pos(pos_add(joint_pos.j2,{x:-0.025,y:0,z:0}))} rotation="0 -90 0" visible={`${j2_error}`}></a-entity>
        <a-entity j_id="2" gltf-model="#j2" position={edit_pos(joint_pos.j2)}>
          <a-entity position="-0.025 0 0" rotation="-10.784 0 0" visible={`${j2_error}`}>
            <a-cylinder position="0 0.05 0" rotation="0 0 0" radius="0.005" height="0.1" color="#FF0000"></a-cylinder>
          </a-entity>
          <a-entity position="0.025 0 0" rotation="-10.784 0 0" visible={`${j2_error}`}>
            <a-cylinder position="0 0.05 0" rotation="0 0 0" radius="0.005" height="0.1" color="#FF0000"></a-cylinder>
          </a-entity>
          <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: 79.216; thetaLength: 185" material="color: #00FFFF" position={edit_pos(joint_pos.j3)} rotation="0 90 0" visible={`${j3_error}`}></a-entity>
          <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: -84.216; thetaLength: 185" material="color: #00FFFF" position={edit_pos(joint_pos.j3)} rotation="0 -90 0" visible={`${j3_error}`}></a-entity>
          <a-entity j_id="3" gltf-model="#j3" position={edit_pos(joint_pos.j3)}>
            <a-cylinder position="0 0.05 0" radius="0.005" height="0.1" color="#FF0000" visible={`${j3_error}`}></a-cylinder>
            <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: 160; thetaLength: 220" material="color: #00FFFF" position="0 0.214 -0.02194" rotation="90 0 0" visible={`${j4_error}`}></a-entity>
            <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: -20; thetaLength: 220" material="color: #00FFFF" position="0 0.214 -0.02194" rotation="-90 0 0" visible={`${j4_error}`}></a-entity>
            <a-entity j_id="4" gltf-model="#j4" position={edit_pos(joint_pos.j4)}>
              <a-cylinder position="0 -0.036 -0.05" rotation="90 0 0" radius="0.005" height="0.1" color="#FF0000" visible={`${j4_error}`}></a-cylinder>
              <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: 10; thetaLength: 160" material="color: #00FFFF" position="0.03 0 0" rotation="0 90 0" visible={`${j5_error}`}></a-entity>
              <a-entity geometry="primitive: circle; radius: 0.1; thetaStart: 10; thetaLength: 160" material="color: #00FFFF" position="-0.03 0 0" rotation="0 -90 0" visible={`${j5_error}`}></a-entity>
              <a-entity j_id="5" gltf-model="#j5" position={edit_pos(joint_pos.j5)}>
                <a-entity position="0.03 0 0" rotation="90 0 0" visible={`${j5_error}`}>
                  <a-cylinder position="0 0.05 0" radius="0.005" height="0.1" color="#FF0000"></a-cylinder>
                </a-entity>
                <a-entity position="-0.03 0 0" rotation="90 0 0" visible={`${j5_error}`}>
                  <a-cylinder position="0 0.05 0" radius="0.005" height="0.1" color="#FF0000"></a-cylinder>
                </a-entity>
                <a-entity j_id="6" gltf-model="#j6" position={edit_pos(joint_pos.j6)}>
                  <Model_Tool {...props}/>
                  {/*<a-cylinder color="crimson" height="0.1" radius="0.005" position={edit_pos(joint_pos.j7)}></a-cylinder>*/}
                </a-entity>
                {/*  <Cursor3dp j_id="15" visible={cursor_vis}/> */}
              </a-entity>
              {/*
              <Cursor3dp j_id="14" pos={{x:joint_pos.j5.x,y:0,z:0}} visible={cursor_vis}/>
              <Cursor3dp j_id="13" visible={cursor_vis}/>
              */}
            </a-entity>
            {/*<Cursor3dp j_id="12" visible={cursor_vis}/>*/}
          </a-entity>
          {/*<Cursor3dp j_id="11" visible={cursor_vis}/>*/}
        </a-entity>
      </a-entity>
    </a-entity>:null}</>
  )
}

const Model_Tool = (props)=>{
  const Toolpos = {x:0,y:0,z:0}
  const {j7_rotate, joint_pos:{j7:j7pos}, cursor_vis, box_vis, edit_pos} = props
  const x = 36/90
  const finger_pos = ((j7_rotate*x) / 1000)+0.0004
  const j6_1_pos = { x: finger_pos, y:0, z:0.226 }
  const j6_2_pos = { x: -finger_pos, y:0, z:0.226 }
  const return_table = [
    <>
      <a-entity gltf-model="#j6_1" position={edit_pos(j6_1_pos)}></a-entity>
      <a-entity gltf-model="#j6_2" position={edit_pos(j6_2_pos)}></a-entity>
      <Cursor3dp j_id="16" pos={j7pos} visible={cursor_vis}/>
      <a-box color="yellow" scale="0.02 0.02 0.02" position={edit_pos(j7pos)} visible={`${box_vis}`}></a-box>
    </>,
  ]
  const {toolNameList, toolName} = props
  const findindex = toolNameList.findIndex((e)=>e===toolName)
  if(findindex >= 0){
    return (return_table[findindex])
  }
  return null
}

const Select_Robot = (props)=>{
  const {robotNameList, robotName, ...rotateProps} = props
  const visibletable = robotNameList.map(()=>false)
  // const robotNameList = ["Model"]
  const findindex = robotNameList.findIndex((e)=>e===robotName)
  if(findindex >= 0){
    visibletable[findindex] = true
  }
  return (<>
    <Model visible={visibletable[0]} {...rotateProps}/>
  </>)
}

const Cursor3dp = (props) => {
  const { pos={x:0,y:0,z:0}, rot={x:0,y:0,z:0}, len=0.3, opa=1, children, visible=false, ...otherprops } = props;

  const line_x = `start: 0 0 0; end: ${len} 0 0; color: red; opacity: ${opa};`
  const line_y = `start: 0 0 0; end: 0 ${len} 0; color: green; opacity: ${opa};`
  const line_z = `start: 0 0 0; end: 0 0 ${len}; color: blue; opacity: ${opa};`

  return <a-entity
      {...otherprops}
      line={line_x}
      line__1={line_y}
      line__2={line_z}
      position={`${pos.x} ${pos.y} ${pos.z}`}
      rotation={`${rot.x} ${rot.y} ${rot.z}`}
      visible={`${visible}`}
  >{children}</a-entity>
}

const Line = (props) => {
  const { pos1={x:0,y:0,z:0}, pos2={x:0,y:0,z:0}, color="magenta", opa=1, visible=false, ...otherprops } = props;

  const line_para = `start: ${pos1.x} ${pos1.y} ${pos1.z}; end: ${pos2.x} ${pos2.y} ${pos2.z}; color: ${color}; opacity: ${opa};`

  return <a-entity
      {...otherprops}
      line={line_para}
      position={`0 0 0`}
      visible={`${visible}`}
  ></a-entity>
}
