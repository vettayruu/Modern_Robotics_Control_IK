import { useEffect } from 'react';

export default function useVrController({
  rendered,
  vrModeRef,
  trigger_on,
  controller_object,
  start_pos,
  save_target,
  target,
  set_save_target,
  set_target,
  set_wrist_rot_x,
  set_wrist_rot_y,
  set_wrist_rot_z,
  round,
  toAngle,
  pos_sub,
  THREE,
  start_rotation,
  save_rotation,
  current_rotation,
  setNow
}) {
  // 位置控制
  useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on) {
      const move_pos = pos_sub(start_pos, controller_object.position);
      let target_pos;
      if (save_target === undefined) {
        set_save_target({ ...target });
        target_pos = pos_sub(target, move_pos);
      } else {
        target_pos = pos_sub(save_target, move_pos);
      }
      if (target_pos.y < 0.012) {
        target_pos.y = 0.012;
      }
      set_target({
        x: round(target_pos.x),
        y: round(target_pos.y),
        z: round(target_pos.z)
      });
    }
  }, [
    rendered,
    vrModeRef.current,
    trigger_on,
    controller_object.position.x,
    controller_object.position.y,
    controller_object.position.z
  ]);

  // 姿态控制
  useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on) {
      const quat_start = new THREE.Quaternion().setFromEuler(start_rotation);
      const quat_controller = new THREE.Quaternion().setFromEuler(controller_object.rotation);
      const quatDifference1 = quat_start.clone().invert().multiply(quat_controller);

      const quat_save = new THREE.Quaternion().setFromEuler(save_rotation);
      const quatDifference2 = quat_start.clone().invert().multiply(quat_save);

      const wk_mtx = quat_start.clone().multiply(quatDifference1).multiply(quatDifference2);
      current_rotation = new THREE.Euler().setFromQuaternion(wk_mtx, controller_object.rotation.order);

      wk_mtx.multiply(
        new THREE.Quaternion().setFromEuler(
          new THREE.Euler(
            (0.6654549523360951 * -1),
            Math.PI,
            Math.PI,
            controller_object.rotation.order
          )
        )
      );

      const wk_euler = new THREE.Euler().setFromQuaternion(wk_mtx, controller_object.rotation.order);
      set_wrist_rot_x(round(toAngle(wk_euler.x)));
      set_wrist_rot_y(round(toAngle(wk_euler.y)));
      set_wrist_rot_z(round(toAngle(wk_euler.z)));
    }
  }, [
    rendered,
    vrModeRef.current,
    trigger_on,
    controller_object.rotation.x,
    controller_object.rotation.y,
    controller_object.rotation.z
  ]);
}

this.el.addEventListener('triggerdown', () => {
  start_rotation.current = this.el.object3D.rotation.clone();
  const wk_start_pos = new window.AFRAME.THREE.Vector3().applyMatrix4(this.el.object3D.matrix);
  set_start_pos(wk_start_pos);
  set_trigger_on(true);
});
this.el.addEventListener('triggerup', () => {
  save_rotation.current = current_rotation.clone();
  set_save_target(undefined);
  set_trigger_on(false);
});