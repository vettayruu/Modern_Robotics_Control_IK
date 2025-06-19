import { useEffect } from 'react';
import { connectMQTT, mqttclient, idtopic, subscribeMQTT, publishMQTT, codeType } from '../lib/MetaworkMQTT'

export default function useMqtt({
  props,
  thetaBodyMQTT,
  robotIDRef,
  MQTT_REQUEST_TOPIC,
  MQTT_DEVICE_TOPIC,
  MQTT_VR_TOPIC,
  MQTT_ROBOT_STATE_TOPIC,
  receiveStateRef,
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

      /* VR Viewer Mode */
      if (props.viewer) {
        window.mqttClient.on('message', (topic, message) => {
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
              subscribeMQTT([
                MQTT_VR_TOPIC + data.devId,
              ]);
            }
            return;
          }

          if (topic === MQTT_VR_TOPIC + robotIDRef.current) {
            if (data.joints != undefined) {
              thetaBodyMQTT(prev => {
              if (JSON.stringify(prev) !== JSON.stringify(data.joints)) {
                console.log("Time", data.time, "useMqtt received setThetaBody:", data.joints, "from", topic);
                return data.joints;}
              return prev;});
            }
          }
        })

        // const handler = (topic, message) => {
        //   let data;
        //   try {
        //     data = JSON.parse(message.toString());
        //   } catch (e) {
        //     console.warn("MQTT error:", message.toString());
        //     return;
        //   }

        //   if (topic === MQTT_DEVICE_TOPIC) {
        //     if (data.devId != undefined) {
        //       robotIDRef.current = data.devId;
        //       subscribeMQTT([
        //         MQTT_VR_TOPIC + data.devId,
        //       ]);
        //     }
        //     return;
        //   }

        //   if (topic === MQTT_VR_TOPIC + robotIDRef.current) {
        //     if (data.joints != undefined) {
        //       thetaBodyMQTT(prev => {
        //         if (JSON.stringify(prev) !== JSON.stringify(data.joints)) {
        //           console.log("Time", data.time, "useMqtt received setThetaBody:", data.joints, "from", topic);
        //           return data.joints;
        //         }
        //         return prev;
        //       });
        //     }
        //   }
        // };

        // window.mqttClient.on('message', handler);

        // return () => {
        //   window.mqttClient.off('message', handler);
        // };
      } 
      else {
        window.mqttClient.on('message', (topic, message) => {
          if (topic === MQTT_DEVICE_TOPIC) {
            let data = JSON.parse(message.toString())
            if (data.devId === "none") {
            } else {
              robotIDRef.current = data.devId
              if (!receiveStateRef.current) {
                subscribeMQTT([
                  MQTT_ROBOT_STATE_TOPIC + robotIDRef.current
                ])
              }
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