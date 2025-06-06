class RobotKinematics {
    static _builders = {};

    constructor(robot_id) {
        this.robot_id = robot_id;
        if (!(robot_id in RobotKinematics._builders)) {
            throw new Error(`Unsupported robot_id: ${robot_id}`);
        }
        const { M, Slist, jointLimits } = RobotKinematics._builders[robot_id]();
        this.M = M;
        this.Slist = Slist;
        this.jointLimits = jointLimits;
    }

    static register_robot(robot_id, builderFunc) {
        RobotKinematics._builders[robot_id] = builderFunc;
    }

    get_M() {
        return this.M;
    }

    get_Slist() {
        return this.Slist;
    }

    get_jointLimits() {
        return this.jointLimits;
    }
}

// 注册 piper_agilex 机器人
RobotKinematics.register_robot("piper_agilex", function build_piper_6dof() {
    const L_01 = 0.123, L_23 = 0.28503, L_34 = 0.25075, L_56 = 0.091, L_ee = 0.1358;
    const W_34 = 0.0219;

    const deg2rad = deg => deg * Math.PI / 180;

    const jointLimits = [
    { min: deg2rad(-150), max: deg2rad(150) },   // theta_1
    { min: deg2rad(-90),  max: deg2rad(90)  },   // theta_2
    { min: deg2rad(0),    max: deg2rad(169) },   // theta_3
    { min: deg2rad(-99),  max: deg2rad(99)  },   // theta_4
    { min: deg2rad(0),    max: deg2rad(139) },   // theta_5
    { min: deg2rad(-120), max: deg2rad(120) },   // theta_6
    ];

    const M = [
        [1, 0, 0, -W_34],
        [0, 1, 0, 0],
        [0, 0, 1, L_01 + L_23 + L_34 + L_56 + L_ee],
        [0, 0, 0, 1]
    ];

    function screw_axis(w, q) {
        // w, q: Array(3)
        // 返回长度为6的数组
        const cross = [
            w[1]*q[2] - w[2]*q[1],
            w[2]*q[0] - w[0]*q[2],
            w[0]*q[1] - w[1]*q[0]
        ];
        return w.concat([-cross[0], -cross[1], -cross[2]]);
    }

    const S1 = screw_axis([0, 0, 1], [0, 0, L_01]);
    const S2 = screw_axis([0, 1, 0], [0, 0, L_01]);
    const S3 = screw_axis([0, 1, 0], [0, 0, L_01 + L_23]);
    const S4 = screw_axis([0, 0, 1], [-W_34, 0, L_01 + L_23 + L_34]);
    const S5 = screw_axis([0, 1, 0], [-W_34, 0, L_01 + L_23 + L_34]);
    const S6 = screw_axis([0, 0, 1], [-W_34, 0, L_01 + L_23 + L_34 + L_56]);

    // Slist: 6x6，每列为一个关节的螺旋轴
    const Slist = [
        S1, S2, S3, S4, S5, S6
    ].map(col => col.slice()); // 6个长度为6的数组

    // 转置为 6x6，每列为一个关节
    const SlistT = Array.from({length: 6}, (_, i) => Slist.map(row => row[i]));

    return { M, Slist: SlistT, jointLimits };
});

// // 注册其他机器人示例
// RobotKinematics.register_robot("ur5", function ur5() {
//     // 自定义 UR5 的参数和姿态
//     // return { M, Slist };
// });

module.exports = RobotKinematics;
