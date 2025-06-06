import React from 'react';
import Assets from './Assets';
import { Select_Robot } from './Model';
import Line from './Line';
import Controller from './webcontroller.js';

export default function RobotScene(props) {
  const {
    rendered, target_error, box_vis,
    cursor_vis, robotProps, controllerProps, dsp_message,
    c_pos_x, c_pos_y, c_pos_z, c_deg_x, c_deg_y, c_deg_z, position_ee, euler_ee
  } = props;

  const rad2deg = rad => rad * 180 / Math.PI;
  const euler_ee_deg = euler_ee.map(rad2deg);

  if (!rendered) {
    return (
      <a-scene xr-mode-ui="XRMode: ar">
        <Assets viewer={props.viewer}/>
      </a-scene>
    );
    }

  return (
    <>
      <a-scene scene xr-mode-ui="XRMode: ar">
        <a-entity oculus-touch-controls="hand: right" vr-controller-right visible={true}></a-entity>
        <Assets viewer={props.viewer} monitor={props.monitor}/>
        <Select_Robot {...robotProps}/>

        {/* Light */}
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="-1 1 1"></a-entity>
        <a-entity light="type: directional; color: #EEE; intensity: 0.25" position="-1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #EFE; intensity: 0.1" position="0 -1 0"></a-entity>
        <a-entity id="rig" position={`${c_pos_x} ${c_pos_y} ${c_pos_z}`} rotation={`${c_deg_x} ${c_deg_y} ${c_deg_z}`}>

        {/* Camera */}
        <a-camera id="camera" cursor="rayOrigin: mouse;" position="0 0 0">
          <a-entity jtext={`text: ${dsp_message}; color: black; background:rgb(31, 219, 131); border: #000000`} position="0 0.7 -1.4"></a-entity>
        </a-camera>

        {/* End Effector */}
        </a-entity>
        <a-sphere position={`${position_ee[0]} ${position_ee[1]} ${position_ee[2]}`} scale="0.012 0.012 0.012" color={target_error ? "red" : "yellow"} visible={true}></a-sphere>
        <a-entity
          position={`${position_ee[0]} ${position_ee[1]} ${position_ee[2]}`}
          rotation={`${euler_ee_deg[0]} ${-euler_ee_deg[2]} ${-euler_ee_deg[1]} `}
        >
          <a-cylinder position="0      0     -0.015" rotation="90 0  0 " height="0.0250" radius="0.0015" color="red" /> 
          <a-cylinder position="-0.015      0     0" rotation="0  0  90" height="0.0250" radius="0.0015" color="green" />
          <a-cylinder position="0      0.025      0" rotation="0  90 0 " height="0.0550" radius="0.0015" color="blue" />
          
        </a-entity>

        {/* 世界坐标系参考轴 */}
        <Line pos1={{x:0,y:0,z:0}} pos2={{x:0,y:0,z:-0.2}} color="red" visible={true} /> 
        <Line pos1={{x:0,y:0,z:0}} pos2={{x:-0.2,y:0,z:0}} color="green" visible={true} />   
        <Line pos1={{x:0,y:0,z:0}} pos2={{x:0,y:0.2,z:0}}  color="blue" visible={true} /> 
         
      </a-scene>
      <Controller {...controllerProps}/>
      <div className="footer">
        <div>{`add information here`}</div>
      </div>
    </>
  );
}