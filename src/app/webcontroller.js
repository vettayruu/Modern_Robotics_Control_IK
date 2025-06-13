"use client";
import * as React from 'react'
import "./webcontroller.css";

// モード選択
const ModeSelector = ({ selectedMode, setSelectedMode }) => {

  const modes = [
    { id: 'control', label: 'Ctrl' }, // 通常の制御モード（VRゴーグル等で操作）
    { id: 'monitor', label: 'Mon' },  // 実ロボットをモニター
    { id: 'vr', label: 'fromVR' }     // 制御トピックをモニター(VRゴーグルからの制御)
  ];

  return (
      <div className="flex space-x-1 rounded-lg bg-gray-100 p-2">
        Mode: 
        {modes.map((mode) => (
          <button
            key={mode.id}
            onClick={() => setSelectedMode(mode.id)}
            className={`px-2 py-1 rounded-md ${
              selectedMode === mode.id
                ? 'bg-white shadow-sm text-blue-600'
                : 'text-gray-600 hover:text-gray-800'
            }`}
          >
            {mode.label}
          </button>
        ))}
      </div>
  );
};


export default function Controller(props) {
const {c_pos_x, c_pos_y, c_pos_z} = props
const {c_deg_x, c_deg_y, c_deg_z} = props
const {vr_mode} = props
const {theta_body} = props
const {theta_tool} = props
const {position_ee} = props
const {euler_ee} = props
// const {twist_ee} = props

const set_c_pos_x = (e)=>{
  let value = Number.parseFloat(e.target.value || 0)
  props.set_c_pos_x(value)
}

const set_c_pos_y = (e)=>{
  let value = Number.parseFloat(e.target.value || 0)
  props.set_c_pos_y(value)
}

const set_c_pos_z = (e)=>{
  let value = Number.parseFloat(e.target.value || 0)
  props.set_c_pos_z(value)
}

const set_c_deg_x = (e)=>{
  let value = Number.parseFloat(e.target.value || 0)
  props.set_c_deg_x(value)
}

const set_c_deg_y = (e)=>{
  let value = Number.parseFloat(e.target.value || 0)
  props.set_c_deg_y(value)
}

const set_c_deg_z = (e)=>{
  let value = Number.parseFloat(e.target.value || 0)
  props.set_c_deg_z(value)
}


// 定义每个关节的限制
const jointLimits = [
  { min: -150, max: 150 }, // theta_1
  { min: -90,  max: 90  }, // theta_2
  { min: 0,    max: 169 }, // theta_3
  { min: -99,  max: 99  }, // theta_4
  { min: 0,    max: 139 }, // theta_5
  { min: -120, max: 120 },  // theta_6
  { min: -1, max: 89 }  // theta_tool
];

const step = 1

return (
  <>
    {/* <div className="controller" >
      <div className="row mb-0">
        <ModeSelector selectedMode={selectedMode} setSelectedMode={setSelectedMode} />
      </div>
    </div> */}
    
    <div className="camera-controller" >
      {vr_mode?null:<><span>CAMERA</span>
      <div className="row mb-0">
        <div className="col-md-4"><label htmlFor="c_pos_x_number" className="form-label"><span className="form-control-plaintext">pos X</span></label></div>
        <div className="col-md-8"><input type="number" className="form-control" id="c_pos_x_number" value={c_pos_x} onChange={set_c_pos_x} step={0.01}/></div>
      </div>
      <div className="row mb-0">
        <div className="col-md-4"><label htmlFor="c_pos_y_number" className="form-label"><span className="form-control-plaintext">pos Y</span></label></div>
        <div className="col-md-8"><input type="number" className="form-control" id="c_pos_y_number" value={c_pos_y} onChange={set_c_pos_y} step={0.01}/></div>
      </div>
      <div className="row mb-2">
        <div className="col-md-4"><label htmlFor="c_pos_z_number" className="form-label"><span className="form-control-plaintext">pos Z</span></label></div>
        <div className="col-md-8"><input type="number" className="form-control" id="c_pos_z_number" value={c_pos_z} onChange={set_c_pos_z} step={0.01}/></div>
      </div>
      <div className="row mb-0">
        <div className="col-md-4"><label htmlFor="c_deg_x_number" className="form-label"><span className="form-control-plaintext">deg X</span></label></div>
        <div className="col-md-8"><input type="number" className="form-control" id="c_deg_x_number" value={c_deg_x} onChange={set_c_deg_x} step={0.1}/></div>
      </div>
      <div className="row mb-0">
        <div className="col-md-4"><label htmlFor="c_deg_y_number" className="form-label"><span className="form-control-plaintext">deg Y</span></label></div>
        <div className="col-md-8"><input type="number" className="form-control" id="c_deg_y_number" value={c_deg_y} onChange={set_c_deg_y} step={0.1}/></div>
      </div>
      <div className="row mb-2">
        <div className="col-md-4"><label htmlFor="c_deg_z_number" className="form-label"><span className="form-control-plaintext">deg Z</span></label></div>
        <div className="col-md-8"><input type="number" className="form-control" id="c_deg_z_number" value={c_deg_z} onChange={set_c_deg_z} step={0.1}/></div>
      </div>
      <div className="row mb-2">
      </div></>}
    </div>
        
    
      <span>Joint_Controll</span>
        <div className="joint controller">
          <div className="joint controller-controll-panel row">
            {theta_body.map((theta, idx) => (
              <div className="row mb-0" key={idx}>
                <div className="col-md-4">
                  <label htmlFor={`theta_${idx+1}`} className="form-label">
                    <span className="form-control-plaintext">{`theta_${idx+1}`}</span>
                  </label>
                </div>
                <div className="col-md-8">
                  <input
                    type="number"
                    className="form-control"
                    id={`theta_${idx+1}`}
                    value={theta.toFixed(5)}
                    onChange={e => {
                      const newTheta = [...theta_body];
                      newTheta[idx] = Number.parseFloat(e.target.value || 0);
                      props.setThetaBody(newTheta);
                    }}
                    step={0.5}
                    min={jointLimits[idx].min}
                    max={jointLimits[idx].max}
                  />
                </div>
              </div>
            ))}
            <div className="row mb-0">
              <div className="col-md-4">
                <label htmlFor="theta_tool" className="form-label">
                  <span className="form-control-plaintext">theta_tool</span>
                </label>
              </div>
              <div className="col-md-8">
                <input
                  type="number"
                  className="form-control"
                  id="theta_tool"
                  value={theta_tool.toFixed(5)}
                  onChange={e => {
                    const value = Number.parseFloat(e.target.value || 0);
                    props.setThetaTool(value);
                  }}
                  step={0.001}
                  min={jointLimits[6].min}
                  max={jointLimits[6].max}
                />
              </div>
            </div>
          </div>
        </div>       

        <span>Modern_Robot_Controll</span>
        <div className="IK controller"></div>
        <div className="modern-robot-controll-panel row">
          {/* Position */}
          <div className="col-md-6">
            <div className="mb-2 fw-bold">Position (cm)</div>
            {['X', 'Y', 'Z'].map((label, idx) => (
              <div className="mb-2" key={label}>
                <label className="form-label">{label}</label>
                <input
                  type="number"
                  className="form-control"
                  value={position_ee[idx].toFixed(5)}
                  // readOnly
                  onChange={e => {
                    const value = Number.parseFloat(e.target.value || 0);
                    const newPos = [...position_ee];
                    newPos[idx] = value;
                    props.setPositionEE(newPos);
                    props.onTargetChange(newPos, props.euler_ee);
                  }}
                  step = {0.01}                   
                />
              </div>
            ))}
          </div>

          {/* Euler */}
          <div className="col-md-6">
            <div className="mb-2 fw-bold">Euler (rad)</div>
            {['z_hat', 'y_hat', 'x_hat'].map((label, idx) => (
              <div className="mb-2" key={label}>
                <label className="form-label">{label}</label>
                <input
                  type="number"
                  className="form-control"
                  value={euler_ee[idx].toFixed(5)}
                  // readOnly
                  onChange={e => {
                    const value = Number.parseFloat(e.target.value || 0);
                        const newEuler = [...euler_ee];
                        newEuler[idx] = value;
                        // props.setEulerXYZ(newEuler);
                        props.setEuler(newEuler);
                        props.onTargetChange(props.position_ee, newEuler);
                  }}
                  step = {0.02}
                />
              </div>
            ))}
          </div>

          {/* Twist 列
          <div className="col-md-6">
            <div className="col-md-6">
              <div className="mb-2 fw-bold">Twist (EE)</div>
              {['omega hat (x)', 'omega hat (y)', 'omega hat (z)', 'theta'].map((label, idx) => (
                <div className="mb-2" key={label}>
                  <label className="form-label">{label}</label>
                  <input
                    type="number"
                    className="form-control"
                    value={twist_ee && twist_ee[0] && idx < 3 ? twist_ee[0][idx].toFixed(5) : (idx === 3 && twist_ee ? twist_ee[1].toFixed(5) : '')}
                    readOnly
                  />
                </div>
              ))}
            </div>
          </div>   */}

      </div>  
  </>
  )
}
