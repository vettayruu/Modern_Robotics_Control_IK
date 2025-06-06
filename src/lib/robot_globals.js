/**
 * 机械臂结构与运动学相关全局常量
 * 仅包含只读结构参数和常量，运行时状态请勿放在此处
 */

// 机械臂各关节的空间坐标（单位：米）
export const joint_pos = Object.freeze({
  base: { x: 0, y: 0, z: 0 },
  j1: { x: 0, y: 0, z: 0 },
  j2: { x: 0, y: 0.1212, z: 0 },
  j3: { x: 0, y: 0.28503, z: 0 },
  j4: { x: 0, y: 0.25, z: -0.02194 },
  j5: { x: 0, y: 0, z: 0 },
  j6: { x: 0, y: 0, z: 0 },
  j7: { x: 0, y: 0, z: 0.225 },
});

export const joint_pos_xyz = Object.freeze({
  base: { x: 0, y: 0, z: 0 },
  j1: { x: 0, y: 0, z: 0 },
  j2: { x: 0, y: 0, z: 0.123 },
  j3: { x: -0.2824, y: 0, z: 0.16162 },
  j4: { x: -0.03452, y: 0, z: 0.20536 },
  j5: { x: -0.03452, y: 0, z: 0.20536 },
  j6: { x: 0.05614, y: 0, z: 0.21319 },
  j7: { x: 0, y: -0.225, z: 0 },
});

// 夹爪长度
// export const p15_16_len = joint_pos.j7.z;
export const p15_16_len = 0.1358;

// 欧拉角顺序
export const order = 'ZYX';

// 基坐标系单位向量
export const x_vec_base = Object.freeze(new THREE.Vector3(1, 0, 0));
export const y_vec_base = Object.freeze(new THREE.Vector3(0, 1, 0));
export const z_vec_base = Object.freeze(new THREE.Vector3(0, 0, 1));

// 运动学相关常量
export const max_move_unit = 1 / 720;
export const target_move_speed = 1000 / 2;

// 各关节旋转轴向量（只读）
export const rotvec_table = Object.freeze([
  y_vec_base, x_vec_base, x_vec_base, y_vec_base, x_vec_base, z_vec_base
]);

// 关节修正值（只读）
export const j1_Correct_value = 180;
export const j2_Correct_value = 90 - 10.784;
export const j3_Correct_value = -180 + 10.784;
export const j4_Correct_value = 0.0;
export const j5_Correct_value = 90;
export const j6_Correct_value = 0.0;
export const j7_Correct_value = 0.0;

export const jointLimits = [
  { min: -165, max: 165, correct: j1_Correct_value, name: 'j1_rotate' },
  { min: -5,   max: 200, correct: j2_Correct_value, name: 'j2_rotate' },
  { min: -180, max: 5,   correct: j3_Correct_value, name: 'j3_rotate' },
  { min: -110, max: 110, correct: j4_Correct_value, name: 'j4_rotate' },
  { min: -80,  max: 80,  correct: j5_Correct_value, name: 'j5_rotate' }
];