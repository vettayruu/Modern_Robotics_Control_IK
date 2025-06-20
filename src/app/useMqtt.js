import { useEffect } from 'react';
import { connectMQTT, mqttclient, idtopic, subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'

export default function useMqtt({
  props,
  requestRobot,
  thetaBodyMQTT,
  thetaToolMQTT,
  robotIDRef,
  MQTT_DEVICE_TOPIC,
  MQTT_VR_TOPIC,
  MQTT_ROBOT_STATE_TOPIC,
}) {
  useEffect(() => {
  // connect to MQTT broker  
  if (typeof window.mqttClient === 'undefined') {
    window.mqttClient = connectMQTT(requestRobot);
    window.mqttClient.on('connect', () => {
      subscribeMQTT(MQTT_DEVICE_TOPIC);
    });
  }

  // define the joint handler for incoming messages
  const handler = (topic, message) => {
    let data;
    try {
      data = JSON.parse(message.toString());
    } catch (e) {
      console.warn("MQTT error:", message.toString());
      return;
    }
    
    if (topic === MQTT_DEVICE_TOPIC) {
      if (data.devId != undefined) {
        robotIDRef.current = data.devId;
        const vrTopic = MQTT_VR_TOPIC + data.devId;
        subscribeMQTT(vrTopic);
        // subscribeMQTT(MQTT_ROBOT_STATE_TOPIC + data.devId);
      }
      return;
    }

    // subscribe joints and tool angles
    if (props.viewer && topic === MQTT_VR_TOPIC + robotIDRef.current) {
      if (data.joints != undefined) {
        thetaBodyMQTT(prev => {
          if (JSON.stringify(prev) !== JSON.stringify(data.joints)) {
            return data.joints;
          }
          return prev;
        });
      }
      if (data.tool != undefined) {
        thetaToolMQTT(prev => {
          if (JSON.stringify(prev) !== JSON.stringify(data.tool)) {
            return data.tool;
          }
          return prev;
        });
      }
    }
    
    // if (!props.viewer && topic === MQTT_ROBOT_STATE_TOPIC + robotIDRef.current) { ... }
  };

  window.mqttClient.on('message', handler);

  const handleBeforeUnload = () => {
    if (mqttclient != undefined) {
      publishMQTT("mgr/unregister", JSON.stringify({ devId: idtopic }));
    }
  };
  window.addEventListener('beforeunload', handleBeforeUnload);

  return () => {
    window.mqttClient.off('message', handler);
    window.removeEventListener('beforeunload', handleBeforeUnload);
  };
}, []);}
