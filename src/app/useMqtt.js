import { useEffect } from 'react';
import { connectMQTT, mqttclient, idtopic, subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'

export default function useMqtt({
  props,
  set_input_rotate,
  set_target_org,
  set_wrist_rot_x,
  set_wrist_rot_y,
  set_wrist_rot_z,
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
  MQTT_REQUEST_TOPIC,
  MQTT_DEVICE_TOPIC,
  MQTT_CTRL_TOPIC,
  MQTT_ROBOT_STATE_TOPIC,
  receiveStateRef,
  mqttclientRef,
  gripRef,
  buttonaRef,
  buttonbRef,
  gripValueRef,
  set_target_error,
  set_dsp_message,
  setNow,
  vrModeRef,
  THREE
}) {
  useEffect(() => {
    const requestRobot = (mqclient) => {
      const requestInfo = {
        devId: idtopic,
        type: codeType,
      }
      publishMQTT(MQTT_REQUEST_TOPIC, JSON.stringify(requestInfo));
    }

    if (typeof window.mqttClient === 'undefined') {
      window.mqttClient = connectMQTT(requestRobot);
      subscribeMQTT([
        MQTT_DEVICE_TOPIC
      ]);

      if (props.viewer) {
        window.mqttClient.on('message', (topic, message) => {
          if (topic == MQTT_DEVICE_TOPIC) {
            let data = JSON.parse(message.toString())
            if (data.controller != undefined) {
              robotIDRef.current = data.devId
              subscribeMQTT([
                "control/" + data.devId
              ]);
            }
          } else if (topic == "control/" + robotIDRef.current) {
            let data = JSON.parse(message.toString())
            if (data.joints != undefined) {
              set_input_rotate(data.joints)
            }
          }
        })
      } else {
        window.mqttClient.on('message', (topic, message) => {
          if (topic === MQTT_DEVICE_TOPIC) {
            let data = JSON.parse(message.toString())
            if (data.devId === "none") {
              // 机器人未找到
            } else {
              robotIDRef.current = data.devId
              if (!receiveStateRef.current) {
                subscribeMQTT([
                  MQTT_ROBOT_STATE_TOPIC + robotIDRef.current
                ])
              }
            }
          }
          if (topic === MQTT_ROBOT_STATE_TOPIC + robotIDRef.current) {
            let data = JSON.parse(message.toString())
            const joints = data.joints
            if (!receiveStateRef.current) {
              if (props.monitor === undefined) {
                mqttclient.unsubscribe(MQTT_ROBOT_STATE_TOPIC + robotIDRef.current)
                receiveStateRef.current = true;
              }
              set_input_rotate(joints)
              window.setTimeout(() => {
                if (target_p16_ref.current !== null) {
                  const obj = target_p16_ref.current
                  const p16_pos = obj.getWorldPosition(new THREE.Vector3())
                  const p16_quat = obj.getWorldQuaternion(new THREE.Quaternion())
                  const p16_euler = new THREE.Euler().setFromQuaternion(p16_quat, order)

                  // set_target_org({ x: p16_pos.x, y: p16_pos.y, z: p16_pos.z })
                  if (p16_pos && typeof p16_pos.x === 'number') {
                    set_target_org({ x: p16_pos.x, y: p16_pos.y, z: p16_pos.z });
                  }

                  set_wrist_rot_x(round(toAngle(p16_euler.x)))
                  set_wrist_rot_y(round(toAngle(p16_euler.y)))
                  set_wrist_rot_z(round(toAngle(p16_euler.z)))
                }
                if (props.monitor === undefined) {
                  publishMQTT("dev/" + robotIDRef.current, JSON.stringify({ controller: "browser", devId: idtopic }))
                }
              }, 500);
            }
          }
        })
      }
    }

    const handleBeforeUnload = () => {
      if (mqttclient != undefined) {
        publishMQTT("mgr/unregister", JSON.stringify({ devId: idtopic }));
      }
    }
    window.addEventListener('beforeunload', handleBeforeUnload);
    return () => {
      window.removeEventListener('beforeunload', handleBeforeUnload);
    }
  }, []);
}