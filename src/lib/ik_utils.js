import * as THREE from 'three'
import {x_vec_base, y_vec_base, z_vec_base, order} from '../lib/robot_globals.js'

// 角度与三维向量相关工具
export const round = (x, d = 5) => {
  const v = 10 ** (d | 0);
  return Math.round(x * v) / v;
};

export const normalize180 = (angle) => {
  if (Math.abs(angle) <= 180) return angle;
  const amari = angle % 180;
  if (amari === 0) return amari;
  if (amari < 0) return 180 + amari;
  return -180 + amari;
};

export const toAngle = (radian) => normalize180(radian * (180 / Math.PI));
export const toRadian = (angle) => normalize180(angle) * (Math.PI / 180);

export const pos_add = (pos1, pos2) => ({
  x: pos1.x + pos2.x,
  y: pos1.y + pos2.y,
  z: pos1.z + pos2.z,
});
export const pos_sub = (pos1, pos2) => ({
  x: pos1.x - pos2.x,
  y: pos1.y - pos2.y,
  z: pos1.z - pos2.z,
});
export const distance = (s_pos, t_pos) =>
  Math.sqrt(
      (t_pos.x - s_pos.x) ** 2 +
      (t_pos.y - s_pos.y) ** 2 +
      (t_pos.z - s_pos.z) ** 2
  );

// 三角形相关
export const degree3 = (side_a, side_b, side_c) => {
  const angle_A = toAngle(
    Math.acos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
  );
  const angle_B = toAngle(
    Math.acos((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c))
  );
  const angle_C = toAngle(
    Math.acos((side_a ** 2 + side_b ** 2 - side_c ** 2) / (2 * side_a * side_b))
  );
  return { angle_A, angle_B, angle_C };
};

export const calc_side_1 = (syahen, kakudo) => {
  const teihen =
    Math.abs(kakudo) === 90 ? 0 : syahen * Math.cos(toRadian(kakudo));
  const takasa =
    Math.abs(kakudo) === 180 ? 0 : syahen * Math.sin(toRadian(kakudo));
  return { a: teihen, b: takasa };
};

export const calc_side_2 = (teihen, takasa) => {
  const syahen = Math.sqrt(teihen ** 2 + takasa ** 2);
  const kakudo = toAngle(Math.atan2(teihen, takasa));
  return { s: syahen, k: kakudo };
};

export const calc_side_4 = (syahen, teihen) => {
  const wk_rad = Math.acos(teihen / syahen);
  const takasa = teihen * Math.tan(wk_rad);
  const kakudo = toAngle(wk_rad);
  return { k: kakudo, t: takasa };
};

// 四元数旋转向量
export function quaternionToRotation(q, v) {
  const q_original = new THREE.Quaternion(q.x, q.y, q.z, q.w)
  const q_conjugate = new THREE.Quaternion(q.x, q.y, q.z, q.w).conjugate()
  const q_vector = new THREE.Quaternion(v.x, v.y, v.z, 0)
  const result = q_original.multiply(q_vector).multiply(q_conjugate)
  return new THREE.Vector3(result.x, result.y, result.z)
}

// 四元数转角度和轴（已内置依赖，不再传参）
export function quaternionToAngle(q) {
  const wk_angle = 2 * Math.acos(round(q.w))
  if (wk_angle === 0) {
    return { angle: toAngle(wk_angle), axis: new THREE.Vector3(0, 0, 0) }
  }
  const angle = toAngle(wk_angle)
  const sinHalfAngle = Math.sqrt(1 - q.w * q.w)
  if (sinHalfAngle > 0) {
    const axisX = q.x / sinHalfAngle
    const axisY = q.y / sinHalfAngle
    const axisZ = q.z / sinHalfAngle
    return { angle, axis: new THREE.Vector3(axisX, axisY, axisZ) }
  } else {
    return { angle, axis: new THREE.Vector3(0, 0, 0) }
  }
}

// 四元数差
export function quaternionDifference(q1, q2) {
  return new THREE.Quaternion(q1.x, q1.y, q1.z, q1.w).invert().multiply(
    new THREE.Quaternion(q2.x, q2.y, q2.z, q2.w)
  )
}

// 方向角计算
export function direction_angle(vec) {
  const dir_sign1 = vec.x < 0 ? -1 : 1
  const xz_vector = new THREE.Vector3(vec.x, 0, vec.z).normalize()
  const direction = toAngle(Math.acos(xz_vector.dot(z_vec_base))) * dir_sign1
  const y_vector = new THREE.Vector3(vec.x, vec.y, vec.z).normalize()
  const angle = toAngle(Math.acos(y_vector.dot(y_vec_base)))
  return { direction, angle }
}

/**
 * 根据欧拉角生成手腕四元数
 * @param {number} rot_x
 * @param {number} rot_y
 * @param {number} rot_z
 * @param {string} order
 * @returns {THREE.Quaternion}
 */
export function get_j5_quaternion(rot_x, rot_y, rot_z, order) {
  return new window.AFRAME.THREE.Quaternion().setFromEuler(
    new window.AFRAME.THREE.Euler(
      toRadian(rot_x), toRadian(rot_y), toRadian(rot_z), order
    )
  );
}
