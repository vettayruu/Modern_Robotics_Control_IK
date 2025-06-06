// 计算DH变换矩阵
function dhTransform(a, alpha, d, theta) {
    const sa = Math.sin(alpha), ca = Math.cos(alpha);
    const st = Math.sin(theta), ct = Math.cos(theta);

    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,      d],
        [0,        0,        0,      1]
    ];
}

// 矩阵乘法 (4x4)
function matMul4(A, B) {
    let C = [];
    for (let i = 0; i < 4; i++) {
        C[i] = [];
        for (let j = 0; j < 4; j++) {
            let sum = 0;
            for (let k = 0; k < 4; k++) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
    return C;
}

// 向量叉乘
function cross(a, b) {
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    ];
}

// 向量减法
function vecSub(a, b) {
    return a.map((v,i) => v - b[i]);
}

// 计算正向运动学，返回每个关节的变换矩阵数组
function forwardKinematics(DH_params, q) {
    let T = [
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ];
    let Ts = [];

    for (let i = 0; i < DH_params.length; i++) {
        const [a, alpha, d, theta_offset] = DH_params[i];
        const theta = q[i] + theta_offset;
        const T_i = dhTransform(a, alpha, d, theta);
        T = matMul4(T, T_i);
        Ts.push(T);
    }
    return Ts;
}

// 修改 forwardKinematics 使其支持绝对角度输入
// function forwardKinematics(DH_params, q_abs) {
//     let T = [
//         [1,0,0,0],
//         [0,1,0,0],
//         [0,0,1,0],
//         [0,0,0,1]
//     ];
//     let Ts = [];

//     for (let i = 0; i < DH_params.length; i++) {
//         const [a, alpha, d, theta_offset] = DH_params[i];
//         // 关键：用绝对角度减去偏置
//         const theta = q_abs[i] - theta_offset;
//         const T_i = dhTransform(a, alpha, d, theta);
//         T = matMul4(T, T_i);
//         Ts.push(T);
//     }
//     return Ts;
// }

// 计算雅可比矩阵，返回6 x n数组
function computeJacobian(DH_params, q) {
    const n = q.length;
    const Ts = forwardKinematics(DH_params, q);

    // 末端位置
    const p_end = [Ts[n-1][0][3], Ts[n-1][1][3], Ts[n-1][2][3]];

    // 初始化6 x n矩阵
    let J = [];
    for (let i = 0; i < 6; i++) {
        J[i] = new Array(n).fill(0);
    }

    for (let i = 0; i < n; i++) {
        // 上一个关节的变换矩阵
        let T_i = (i === 0) ? [
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]
        ] : Ts[i-1];

        const p_i = [T_i[0][3], T_i[1][3], T_i[2][3]];
        const z_i = [T_i[0][2], T_i[1][2], T_i[2][2]]; // z轴方向

        // 旋转关节雅可比列
        const J_linear = cross(z_i, vecSub(p_end, p_i));
        const J_angular = z_i;

        // 填入矩阵
        J[0][i] = J_linear[0];
        J[1][i] = J_linear[1];
        J[2][i] = J_linear[2];
        J[3][i] = J_angular[0];
        J[4][i] = J_angular[1];
        J[5][i] = J_angular[2];
    }

    return J;
}

// Piper机械臂 DH参数（单位：米，角度用弧度）
// [a_{i-1}, alpha_{i-1}, d_i, theta_offset]
const DH_params = [
    [0,         0,           0.123,   0     ],               // joint1: base_link -> link1
    [0,         Math.PI/2,   0,      -3.1416],               // joint2: link1 -> link2
    [0.28503,   0,           0,      -1.759 ],               // joint3: link2 -> link3
    [-0.021984, Math.PI/2,  -0.25075, 0     ],               // joint4: link3 -> link4
    [0,        -Math.PI/2,   0,       0     ],               // joint5: link4 -> link5
    [0,         Math.PI/2,  -0.091,   0     ],               // joint6: link5 -> link6
];

// 测试关节角度（单位：度）
const q_deg = [0, 34.38, -28.65, 0, 11.46, 2.86];
// 转为弧度
const q = q_deg.map(deg => deg * Math.PI / 180);

console.log("测试关节角度（度）:", q_deg.join("\t"));
console.log("测试关节角度（弧度）:", q.map(x => x.toFixed(4)).join("\t"));

const J = computeJacobian(DH_params, q);

console.log("雅可比矩阵 J:");
for (let i = 0; i < 6; i++) {
    console.log(J[i].map(x => x.toFixed(4)).join("\t"));
}

const Ts = forwardKinematics(DH_params, q);
const T_end = Ts[Ts.length - 1]; // 末端变换矩阵
const p_end = [T_end[0][3], T_end[1][3], T_end[2][3]]; // 末端位置

console.log("末端位置:", p_end);
