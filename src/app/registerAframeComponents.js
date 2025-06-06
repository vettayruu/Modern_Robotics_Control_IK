import { joint_pos } from '../lib/robot_globals.js'
import { calc_side_2 } from '../lib/ik_utils'

let registered = false;

export default function registerAframeComponents(options) {
  if (registered) return;
  registered = true;

  const {
    set_rendered,
    set_p14_maxlen,
    set_p11_object, set_p12_object, set_p13_object, set_p14_object, set_p15_object, set_p16_object,
    set_p20_object, set_p21_object, set_p22_object, set_p51_object,
    target_p16_ref,
    robotChange,
    set_controller_object,
    set_start_pos,
    set_trigger_on,
    set_save_target,
    set_c_pos_x, set_c_pos_y, set_c_pos_z,
    set_c_deg_x, set_c_deg_y, set_c_deg_z,
    vrModeRef,
    currentRotationRef,
    start_rotation,
    save_rotation,
    controller_object,
    order,
    props,
    onAnimationMQTT,
    onXRFrameMQTT,
    object3DtableREF // 新增：通过参数传递 ref
  } = options;

  setTimeout(() => set_rendered(true), 10);

  const teihen = joint_pos.j5.x;
  const takasa = joint_pos.j3.y + joint_pos.j4.y;
  const result = calc_side_2(teihen, takasa);
  set_p14_maxlen(result.s);

  AFRAME.registerComponent('robot-click', {
    init: function () {
      this.el.addEventListener('click', () => {
        robotChange();
        console.log('robot-click');
      });
    }
  });

  AFRAME.registerComponent('j_id', {
    schema: { type: 'number', default: 0 },
    init: function () {
      const idx = this.data;
      if (idx >= 1 && idx <= 6) object3DtableREF.current[idx - 1] = this.el.object3D;
      else if (idx === 11) set_p11_object(this.el.object3D);
      else if (idx === 12) set_p12_object(this.el.object3D);
      else if (idx === 13) set_p13_object(this.el.object3D);
      else if (idx === 14) set_p14_object(this.el.object3D);
      else if (idx === 15) set_p15_object(this.el.object3D);
      else if (idx === 16) {
        set_p16_object(this.el.object3D);
        target_p16_ref.current = this.el.object3D;
      }
      else if (idx === 20) set_p20_object(this.el.object3D);
      else if (idx === 21) set_p21_object(this.el.object3D);
      else if (idx === 22) set_p22_object(this.el.object3D);
      else if (idx === 51) set_p51_object(this.el.object3D);
    },
    remove: function () {
      if (this.data === 16) {
        set_p16_object(this.el.object3D);
      }
    }
  });

  AFRAME.registerComponent('vr-controller-right', {
    schema: { type: 'string', default: '' },
    init: function () {
      set_controller_object(this.el.object3D);
      this.el.object3D.rotation.order = order;
      this.el.addEventListener('triggerdown', () => {
        start_rotation.current = this.el.object3D.rotation.clone();
        const wk_start_pos = new window.AFRAME.THREE.Vector3().applyMatrix4(this.el.object3D.matrix);
        set_start_pos(wk_start_pos);
        set_trigger_on(true);
      });
      this.el.addEventListener('triggerup', () => {
        save_rotation.current = currentRotationRef.current.clone();
        set_save_target(undefined);
        set_trigger_on(false);
      });
    },
    update: function () {
      if (this.el.object3D !== controller_object) {
        set_controller_object(this.el.object3D);
        console.log("Trigger update!");
      }
    }
  });

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
      const bg = document.createElement('a-plane');
      bg.setAttribute('width', data.width);
      bg.setAttribute('height', data.height);
      bg.setAttribute('color', data.background);
      bg.setAttribute('position', '0 0 0.01');
      bg.setAttribute('opacity', '0.8');
      const text = document.createElement('a-entity');
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
      el.appendChild(bg);
      el.appendChild(text);
    },
    update: function (oldData) {
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
    }
  });

  AFRAME.registerComponent('scene', {
    init: function () {
      if (props.viewer) {
        window.requestAnimationFrame(onAnimationMQTT);
      }
      this.el.addEventListener('enter-vr', () => {
        vrModeRef.current = true;
        console.log('enter-vr');
        if (!props.viewer) {
          let xrSession = this.el.renderer.xr.getSession();
          xrSession.requestAnimationFrame(onXRFrameMQTT);
        }
        set_c_pos_x(0);
        set_c_pos_y(-0.6);
        set_c_pos_z(0.90);
        set_c_deg_x(0);
        set_c_deg_y(0);
        set_c_deg_z(0);
      });
      this.el.addEventListener('exit-vr', () => {
        vrModeRef.current = false;
        console.log('exit-vr');
      });
    }
  });
}