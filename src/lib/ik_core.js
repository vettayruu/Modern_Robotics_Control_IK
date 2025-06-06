import {
  round, normalize180, toAngle, toRadian,
  pos_add, pos_sub, distance,
  degree3, calc_side_1, calc_side_2, calc_side_4,
  quaternionToRotation, quaternionToAngle, quaternionDifference
} from './ik_utils';

import { joint_pos, p15_16_len, x_vec_base, y_vec_base, z_vec_base, order, joint_pos_xyz } from './robot_globals';

/**
 * 计算所有关节角度（逆运动学核心步骤）
 * @param {Object} final_target - 末端执行器的目标点坐标 {x, y, z}
 * @param {number} wrist_direction - 手腕方向角（平面旋转）
 * @param {number} wrist_angle - 手腕俯仰角
 * @param {THREE.Quaternion} j5q - 手腕目标四元数
 * @param {THREE.Vector3} p21_pos - 末端到手腕的偏移向量
 * @returns {Object} 
 *   {
 *     j1_rotate, j2_rotate, j3_rotate, j4_rotate, j5_rotate, j6_rotate, 
 *     dsp_message // 错误信息（如有）
 *   }
 */
export function get_all_rotate_core(
  final_target, wrist_direction, wrist_angle, j5q, p21_pos
) {
  let dsp_message = "";

  // 1. 计算末端目标点（ee(p16)）和手腕点（wrist(p15)）的位置
  const ee_pos = new THREE.Vector3(final_target.x, final_target.y, final_target.z);
  const wrist_pos = ee_pos.clone().sub(p21_pos);

  // 2. 判断是否需要反向解（肘下或翻腕解）
  let back = false;
  if (round(wrist_pos.x) !== round(ee_pos.x) || round(wrist_pos.z) !== round(ee_pos.z)) {
    let wk_angleC = toAngle(
      new THREE.Vector3(wrist_pos.x, 0, wrist_pos.z).sub(new THREE.Vector3()).angleTo(
        new THREE.Vector3(ee_pos.x, 0, ee_pos.z).sub(new THREE.Vector3()))
    );
    if (isNaN(wk_angleC)) wk_angleC = 0;
    wk_angleC = round(wk_angleC);
    if (wk_angleC > 90) {
      back = true;
    } else {
      const distance_wrist = distance({ x: 0, y: 0, z: 0 }, { x: wrist_pos.x, y: 0, z: wrist_pos.z });
      const distance_ee = distance({ x: 0, y: 0, z: 0 }, { x: ee_pos.x, y: 0, z: ee_pos.z });
      if (distance_wrist > distance_ee) back = true;
    }
  }

  // 3. 推导J2、J3角度
  const syahen_t15 = distance(joint_pos.j2, wrist_pos);
  const takasa_t15 = (wrist_pos.y - joint_pos.j2.y);
  const { k: angle_t15 } = calc_side_4(syahen_t15, takasa_t15);
  const distance_j3 = distance({ x: 0, y: 0, z: 0 }, joint_pos.j3);
  const distance_j4 = distance({ x: 0, y: 0, z: 0 }, joint_pos.j4);
  const result_t15 = get_J2_J3_rotate_core(angle_t15 * (back ? -1 : 1), distance_j3, distance_j4, syahen_t15);
  if (result_t15.dsp_message) {
    return { j1_rotate: 0, j2_rotate: 0, j3_rotate: 0, j4_rotate: 0, j5_rotate: 0, j6_rotate: 0, dsp_message: result_t15.dsp_message };
  }
  const wk_j2_rotate = result_t15.j2_rotate;
  const wk_j3_rotate = result_t15.j3_rotate;

  // 4. 计算J1角度
  const dir_sign_t15 = wrist_pos.x < 0 ? -1 : 1;
  const xz_vector_t15 = new THREE.Vector3(wrist_pos.x, 0, wrist_pos.z).normalize();
  const direction_t15 = (toAngle(z_vec_base.angleTo(xz_vector_t15))) * dir_sign_t15;
  if (isNaN(direction_t15)) {
    return { j1_rotate: 0, j2_rotate: wk_j2_rotate, j3_rotate: wk_j3_rotate, j4_rotate: 0, j5_rotate: 0, j6_rotate: 0, dsp_message: "direction_t15 指定可能範圍外！" };
  }
  let wk_j1_rotate = normalize180(direction_t15 + (back ? 180 : 0));
  if (isNaN(wk_j1_rotate)) {
    return { j1_rotate: 0, j2_rotate: wk_j2_rotate, j3_rotate: wk_j3_rotate, j4_rotate: 0, j5_rotate: 0, j6_rotate: 0, dsp_message: "wk_j1_rotate 指定可能範圍外！" };
  }

  // 5. 计算J4、J5角度
  const mtx_j2 = new THREE.Matrix4().multiply(
    new THREE.Matrix4().makeRotationY(toRadian(wk_j1_rotate)).setPosition(joint_pos.j1.x, joint_pos.j1.y, joint_pos.j1.z)
  ).multiply(
    new THREE.Matrix4().makeRotationX(toRadian(wk_j2_rotate)).setPosition(joint_pos.j2.x, joint_pos.j2.y, joint_pos.j2.z)
  );
  const mtx_j3 = mtx_j2.clone().multiply(
    new THREE.Matrix4().makeRotationX(toRadian(wk_j3_rotate)).setPosition(joint_pos.j3.x, joint_pos.j3.y, joint_pos.j3.z)
  );
  const mtx_j4 = mtx_j3.clone().multiply(
    new THREE.Matrix4().setPosition(joint_pos.j4.x, joint_pos.j4.y, joint_pos.j4.z)
  );

  // J5三角形角度
  const mtx_j3_wk = mtx_j3.clone().multiply(
    new THREE.Matrix4().setPosition(0, 0, joint_pos.j4.z)
  );
  const j3_pos_wk = new THREE.Vector3().applyMatrix4(mtx_j3_wk);
  const distance_13_16 = distance(j3_pos_wk, ee_pos);
  let j5_angle_C = 180;
  if ((joint_pos.j4.y + p15_16_len) > distance_13_16) {
    j5_angle_C = degree3(joint_pos.j4.y, p15_16_len, distance_13_16).angle_C;
  }
  if (isNaN(j5_angle_C)) {
    return { j1_rotate: wk_j1_rotate, j2_rotate: wk_j2_rotate, j3_rotate: wk_j3_rotate, j4_rotate: 0, j5_rotate: 0, j6_rotate: 0, dsp_message: "j5_angle_C 指定可能範圍外！" };
  }
  let j5_base = (180 - j5_angle_C);

  // 判断手腕朝向，决定J5正负
  const j3_arm_angle = round((wk_j2_rotate + wk_j3_rotate), 10);
  const judge_wrist_angle = round(wrist_angle, 10);
  if (j3_arm_angle < 90) {
    if (j3_arm_angle > judge_wrist_angle) j5_base = j5_base * -1;
  } else if (j3_arm_angle > 90) {
    if (j3_arm_angle < judge_wrist_angle) j5_base = j5_base * -1;
  }
  let wk_j5_rotate = normalize180((j5_base - 90));

  // J4角度
  const mtx_j5_zero = mtx_j4.clone().multiply(
    new THREE.Matrix4().makeRotationX(toRadian(wk_j5_rotate)).setPosition(joint_pos.j5.x, joint_pos.j5.y, joint_pos.j5.z)
  ).multiply(
    new THREE.Matrix4().setPosition(joint_pos.j6.x, joint_pos.j6.y, joint_pos.j6.z)
  ).multiply(
    new THREE.Matrix4().setPosition(joint_pos.j7.x, joint_pos.j7.y, p15_16_len)
  );
  const ee_zero_pos = new THREE.Vector3().applyMatrix4(mtx_j5_zero);
  const j5_tri_wk = calc_side_1(p15_16_len, Math.abs(j5_base));
  const mtx_j5_center = mtx_j4.clone().multiply(
    new THREE.Matrix4().setPosition(0, j5_tri_wk.a, 0)
  );
  const j5_center_pos = new THREE.Vector3().applyMatrix4(mtx_j5_center);
  const wk_j4_angle_C = toAngle(ee_zero_pos.clone().sub(j5_center_pos).angleTo(ee_pos.clone().sub(j5_center_pos)));
  const direction_offset = normalize180(wrist_direction - wk_j1_rotate);
  const j4_base = wk_j4_angle_C * (direction_offset < 0 ? -1 : 1);
  let wk_j4_rotate = normalize180(j4_base);

  // 6. 通过四元数差值，推导J6角度
  const baseq = new THREE.Quaternion().multiply(
    new THREE.Quaternion().setFromAxisAngle(y_vec_base, toRadian(wk_j1_rotate))
  ).multiply(
    new THREE.Quaternion().setFromAxisAngle(x_vec_base, toRadian(wk_j2_rotate))
  ).multiply(
    new THREE.Quaternion().setFromAxisAngle(x_vec_base, toRadian(wk_j3_rotate))
  ).multiply(
    new THREE.Quaternion().setFromAxisAngle(y_vec_base, toRadian(wk_j4_rotate))
  ).multiply(
    new THREE.Quaternion().setFromAxisAngle(x_vec_base, toRadian(wk_j5_rotate))
  ).multiply(
    new THREE.Quaternion().setFromAxisAngle(z_vec_base, Math.PI)
  );
  const p14_j5_diff = quaternionDifference(baseq, j5q);
  const p14_j5_diff_angle = quaternionToAngle(p14_j5_diff);
  const wk_j6_rotate = p14_j5_diff_angle.angle * ((p14_j5_diff_angle.axis.z < 0) ? -1 : 1);

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
 */
export function get_J2_J3_rotate_core(angle_base, side_a, side_b, side_c) {
  let dsp_message = undefined;

  // 1. 检查三边能否构成三角形
    const max_dis = side_a + side_b;
    const min_dis = Math.abs(side_a - side_b);
  
    let j2_rotate = 0;
    let j3_rotate = 0;
  
    if (min_dis > side_c) {
      // 目标点过近，三角形无法构成
      j2_rotate = angle_base;
      j3_rotate = 180;
      dsp_message = "目标点过近，三角形无法构成";
    } else if (side_c >= max_dis) {
      // 目标点过远，三角形无法构成
      j2_rotate = angle_base;
      j3_rotate = 0;
      dsp_message = "目标点过远，三角形无法构成";
    } else {
      // 2. 正常三角形，利用余弦定理计算各内角
      const result = degree3(side_a, side_b, side_c);
      let { angle_B, angle_C } = result;
  
      if (isNaN(angle_B)) angle_B = 0;
      if (isNaN(angle_C)) angle_C = 0;
  
      // 3. 推导J2、J3角度
      const angle_j2 = normalize180(angle_base - angle_B);
      const angle_j3 = normalize180(angle_C === 0 ? 0 : 180 - angle_C);
  
      if (isNaN(angle_j2)) {
        dsp_message = push("angle_j2 指定可能範圍外！");
        j2_rotate = 0;
      } else {
        j2_rotate = angle_j2;
      }
      if (isNaN(angle_j3)) {
        dsp_message = push("angle_j3 指定可能範圍外！");
        j3_rotate = 0;
      } else {
        j3_rotate = angle_j3;
      }
    }
  
    // 4. J3角度需加上末端连杆的偏移修正
    const j4_sabun = calc_side_2(-joint_pos.j4.z, joint_pos.j4.y);
    j3_rotate = j3_rotate + j4_sabun.k;

  return { j2_rotate, j3_rotate, dsp_message };
}


/**
 * 逆运动学主循环，尝试多次修正目标点以获得最优解
 * @param {number} wrist_direction 手腕方向角
 * @param {number} wrist_angle     手腕俯仰角
 * @param {THREE.Quaternion} j5q   手腕目标四元数
 * @param {THREE.Vector3} p21_pos  末端到手腕的偏移向量
 * @returns {Object} result_rotate 逆解结果
 */
export function target15_update_core(target, wrist_direction, wrist_angle, j5q, p21_pos) {
  let shift_target = {...target};
  let result_rotate = {};
  let save_distance = undefined;
  let save_rotate = undefined;
  let save_target = {...target};
  let all_messages = "";

  for(let i=0; i<10; i++){
    // set_test_pos({...shift_target});

    // 每个循环调取“get_all_rotate_core”进行迭代
    result_rotate = get_all_rotate_core(shift_target, wrist_direction, wrist_angle, j5q, p21_pos);

    if(result_rotate.dsp_message){
      all_messages = all_messages.concat(result_rotate.dsp_message);
      // set_target_error(true);
      break;
    }

    // 末端误差
    const base_m4 = new THREE.Matrix4()
      .multiply(new THREE.Matrix4().makeRotationY(toRadian(result_rotate.j1_rotate)).setPosition(joint_pos.j1.x,joint_pos.j1.y,joint_pos.j1.z))
      .multiply(new THREE.Matrix4().makeRotationX(toRadian(result_rotate.j2_rotate)).setPosition(joint_pos.j2.x,joint_pos.j2.y,joint_pos.j2.z))
      .multiply(new THREE.Matrix4().makeRotationX(toRadian(result_rotate.j3_rotate)).setPosition(joint_pos.j3.x,joint_pos.j3.y,joint_pos.j3.z))
      .multiply(new THREE.Matrix4().makeRotationY(toRadian(result_rotate.j4_rotate)).setPosition(joint_pos.j4.x,joint_pos.j4.y,joint_pos.j4.z))
      .multiply(new THREE.Matrix4().makeRotationX(toRadian(result_rotate.j5_rotate)).setPosition(joint_pos.j5.x,joint_pos.j5.y,joint_pos.j5.z))
      .multiply(new THREE.Matrix4().makeRotationZ(toRadian(result_rotate.j6_rotate)).setPosition(joint_pos.j6.x,joint_pos.j6.y,joint_pos.j6.z))
      .multiply(new THREE.Matrix4().setPosition(joint_pos.j7.x,joint_pos.j7.y,joint_pos.j7.z));

    const result_target = new THREE.Vector3().applyMatrix4(base_m4);
    const sabun_pos = pos_sub(target, result_target);
    const sabun_distance = sabun_pos.x**2 + sabun_pos.y**2 + sabun_pos.z**2;
    const wk_euler = new THREE.Euler().setFromRotationMatrix(base_m4, order);
    const sabun_angle = j5q.angleTo(new THREE.Quaternion().setFromEuler(wk_euler));

    // 收敛判定
    if(round(sabun_distance) <= 0 && round(sabun_angle,2) <= 0){
      save_target = {...result_target};
      break;
    }

    // 防止发散
    if(save_distance === undefined){
      save_distance = sabun_distance;
      save_rotate = {...result_rotate};
    }else{
      if(save_distance < sabun_distance){
        if(round(sabun_distance,4) <= 0){
          result_rotate = {...save_rotate};
          save_target = {...result_target};
          break;
        }
        all_messages = all_messages.concat("姿勢制御不可！");
        break;
      }
      save_distance = sabun_distance;
      save_rotate = {...result_rotate};
    }

    // NaN 检查
    if (
      isNaN(shift_target.x) || isNaN(shift_target.y) || isNaN(shift_target.z) ||
      isNaN(sabun_pos.x) || isNaN(sabun_pos.y) || isNaN(sabun_pos.z)
    ) {
      console.error('shift_target 或 sabun_pos 出现 NaN，提前退出循环', shift_target, sabun_pos);
      break;
    }

    // 误差修正
    shift_target = pos_add(shift_target, sabun_pos);
    console.log(`第 ${i + 1} 次迭代，目标点修正:`, shift_target, "误差:", sabun_pos, "距离:", sabun_distance, "角度:", sabun_angle);
  }

  return {result_rotate, shift_target, all_messages};
}