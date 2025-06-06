const mr = require('../modern_robotics/modern_robotics_core.js');

// *** BASIC HELPER FUNCTIONS ***
// console.log(mr.NearZero(-1e-7)); // true
// console.log(mr.Normalize([1,2,3])); // [0.267..., 0.534..., 0.801...]

// *** CHAPTER 3: RIGID-BODY MOTIONS ***
/* Function RotInv
// const R = [
//     [0, 0, 1],
//     [1, 0, 0],
//     [0, 1, 0]
// ];

// console.log("原始矩阵 R:");
// console.log(R);

// const R_inv = mr.RotInv(R);

// console.log("转置后的矩阵 R_inv:");
// console.log(R_inv);
*/

/* Function VecToso3
// const omg = [1, 2, 3];
// const so3mat = mr.VecToso3(omg);

// console.log("输入向量 omg:", omg);
// console.log("对应的so(3)反对称矩阵:");
// console.log(so3mat);
*/

// Function so3ToVec
// const so3mat = [
//     [0, -3, 2],
//     [3, 0, -1],
//     [-2, 1, 0]
// ];

// console.log("输入so(3)矩阵:");
// console.log(so3mat);

// const omg = mr.so3ToVec(so3mat);

// console.log("转换得到的向量 omg:");
// console.log(omg);

// Function AxisAng3
// const expc3 = [1, 2, 3];
// const [omghat, theta] = mr.AxisAng3(expc3);

// console.log("输入旋转指数坐标 expc3:", expc3);
// console.log("单位旋转轴 omghat:", omghat);
// console.log("旋转角度 theta:", theta);

// Function MatrixExp3
// const so3mat = [
//     [0, -3, 2],
//     [3, 0, -1],
//     [-2, 1, 0]
// ];

// console.log("输入so(3)矩阵:");
// console.log(so3mat);

// const R = mr.MatrixExp3(so3mat);

// console.log("MatrixExp3输出的旋转矩阵:");
// console.log(R);

// Function MatrixLog3
// const R = [
//     [0, 0, 1],
//     [1, 0, 0],
//     [0, 1, 0]
// ];

// console.log("输入旋转矩阵 R:");
// console.log(R);

// const logR = mr.MatrixLog3(R);

// console.log("MatrixLog3输出的so(3)反对称矩阵:");
// console.log(logR);

// Function Adjoint
// const T = [
//     [1, 0,  0, 0],
//     [0, 0, -1, 0],
//     [0, 1,  0, 3],
//     [0, 0,  0, 1]
// ];

// console.log("输入齐次变换矩阵 T:");
// console.log(T);

// const AdT = mr.Adjoint(T);

// console.log("Adjoint(T)输出的6x6矩阵:");
// console.log(AdT);


// // Function ScrewToAxis
// const q = [3, 0, 0];
// const s = [0, 0, 1];
// const h = 2;

// const screwAxis = mr.ScrewToAxis(q, s, h);

// console.log("输入参数:");
// console.log("q =", q);
// console.log("s =", s);
// console.log("h =", h);
// console.log("ScrewToAxis输出:");
// console.log(screwAxis); // 应输出: [0, 0, 1, 0, -3, 2]

// Function AxisAng6
// const expc6 = [1, 0, 0, 1, 2, 3];
// const [S, theta] = mr.AxisAng6(expc6);

// console.log("输入6维指数坐标 expc6:", expc6);
// console.log("归一化螺旋轴 S:", S);
// console.log("旋转/位移量 theta:", theta);

// Function MatrixExp6
// const se3mat = [
//     [0, 0, 0, 0],
//     [0, 0, -1.57079632, 2.35619449],
//     [0, 1.57079632, 0, 2.35619449],
//     [0, 0, 0, 0]
// ];

// console.log("输入se(3)矩阵:");
// console.log(se3mat);

// const T = mr.MatrixExp6(se3mat);

// console.log("MatrixExp6输出的4x4变换矩阵:");
// console.log(T);

// Function MatrixLog6
// const T = [
//     [1, 0,  0, 0],
//     [0, 0, -1, 0],
//     [0, 1,  0, 3],
//     [0, 0,  0, 1]
// ];

// console.log("输入SE(3)齐次变换矩阵 T:");
// console.log(T);

// const logT = mr.MatrixLog6(T);

// console.log("MatrixLog6输出的se(3)矩阵:");
// console.log(logT);


// Function ProjectToSO3
// const mat = [
//     [0.675,  0.150,  0.720],
//     [0.370,  0.771, -0.511],
//     [-0.630, 0.619,  0.472]
// ];

// console.log("输入矩阵 mat:");
// console.log(mat);

// const R = mr.ProjectToSO3(mat);

// console.log("ProjectToSO3输出的SO(3)矩阵:");
// console.log(R);

// Function ProjectToSE3
// const mat = [
//     [0.675,  0.150,  0.720,  1.2],
//     [0.370,  0.771, -0.511,  5.4],
//     [-0.630,  0.619,  0.472,  3.6],
//     [0.003,  0.002,  0.010,  0.9]
// ];

// console.log("输入矩阵 mat:");
// console.log(mat);

// const T = mr.ProjectToSE3(mat);

// console.log("ProjectToSE3输出的SE(3)矩阵:");
// console.log(T);

// Function DistanceToSO3
// const mat = [
//     [1.0,  0.0,   0.0],
//     [0.0,  0.1,  -0.95],
//     [0.0,  1.0,   0.1]
// ];

// console.log("输入矩阵 mat:");
// console.log(mat);

// const dist = mr.DistanceToSO3(mat);

// console.log("DistanceToSO3输出:");
// console.log(dist); // 应输出约 0.08835

// Function DistanceToSE3
// const mat = [
//     [1.0,  0.0,   0.0,   1.2 ],
//     [0.0,  0.1,  -0.95,  1.5 ],
//     [0.0,  1.0,   0.1,  -0.9 ],
//     [0.0,  0.0,   0.1,   0.98 ]
// ];

// console.log("输入矩阵 mat:");
// console.log(mat);

// const dist = mr.DistanceToSE3(mat);

// console.log("DistanceToSE3输出:");
// console.log(dist); // 应输出约 0.134931

/*** Chapter4 Foward Kinamatics ***/

// Function FKinBody
// const M = [
//     [-1, 0,  0, 0],
//     [ 0, 1,  0, 6],
//     [ 0, 0, -1, 2],
//     [ 0, 0,  0, 1]
// ];
// const Blist = [
//     [0, 0,  1],
//     [0, 0,  0],
//     [-1, 0,  0],
//     [2, 0,  0],
//     [0, 1,  0],
//     [0, 0, 0.1]
// ]; // 6x3，每列为一个关节的螺旋轴
// const thetalist = [Math.PI / 2.0, 3, Math.PI];

// console.log("输入末端初始位姿 M:");
// console.log(M);
// console.log("输入Blist:");
// console.log(Blist);
// console.log("输入thetalist:");
// console.log(thetalist);

// const T = mr.FKinBody(M, Blist, thetalist);

// console.log("FKinBody输出的末端位姿:");
// console.log(T);
// // 期望输出：
// // [
// //   [0, 1,  0,         -5],
// //   [1, 0,  0,          4],
// //   [0, 0, -1, 1.68584073],
// //   [0, 0,  0,          1]
// // ]


// Function FKinBody with numeric.js
// const M = [
//     [-1, 0,  0, 0],
//     [ 0, 1,  0, 6],
//     [ 0, 0, -1, 2],
//     [ 0, 0,  0, 1]
// ];
// const Slist = [
//     [0, 0,  0],
//     [0, 0,  0],
//     [1, 0,  -1],
//     [4, 0,  -6],
//     [0, 1,  0],
//     [0, 0, -0.1]
// ]; // 6x3，每列为一个关节的螺旋轴
// const thetalist = [Math.PI / 2.0, 3, Math.PI];

// console.log("输入末端初始位姿 M:");
// console.log(M);
// console.log("输入Slist:");
// console.log(Slist);
// console.log("输入thetalist:");
// console.log(thetalist);

// const T = mr.FKinSpace(M, Slist, thetalist);

// console.log("FKinSpace输出的末端位姿:");
// console.log(T);
// // 期望输出：
// // [
// //   [0, 1,  0,         -5],
// //   [1, 0,  0,          4],
// //   [0, 0, -1, 1.68584073],
// //   [0, 0,  0,          1]
// // ]


// const Blist = [
//     [0,   1, 0,   1],
//     [0,   0, 1,   0],
//     [1,   0, 0,   0],
//     [0,   2, 0, 0.2],
//     [0.2, 0, 2, 0.3],
//     [0.2, 3, 1, 0.4]
// ]; // 6x4，每列为一个关节的螺旋轴
// const thetalist = [0.2, 1.1, 0.1, 1.2];

// console.log("输入Blist:");
// console.log(Blist);
// console.log("输入thetalist:");
// console.log(thetalist);

// const Jb = mr.JacobianSpace(Blist, thetalist);

// console.log("JacobianBody输出:");
// console.log(Jb);
// 期望输出：
// [
//   [-0.04528405, 0.99500417,           0,   1],
//   [ 0.74359313, 0.09304865,  0.36235775,   0],
//   [-0.66709716, 0.03617541, -0.93203909,   0],
//   [ 2.32586047,    1.66809,  0.56410831, 0.2],
//   [-1.44321167, 2.94561275,  1.43306521, 0.3],
//   [-2.06639565, 1.82881722, -1.58868628, 0.4]
// ]

/*** CHAPTER 6: INVERSE KINEMATICS ***/
// const Blist = [
//     [0,  0,  0],
//     [0,  0,  0],
//     [-1, 0,  1],
//     [2,  0,  0],
//     [0,  1,  0],
//     [0,  0, 0.1]
// ]; // 6x3，每列为一个关节的螺旋轴

// const M = [
//     [-1, 0,  0, 0],
//     [ 0, 1,  0, 6],
//     [ 0, 0, -1, 2],
//     [ 0, 0,  0, 1]
// ];

// const T = [
//     [0, 1,  0,     -5],
//     [1, 0,  0,      4],
//     [0, 0, -1, 1.6858],
//     [0, 0,  0,      1]
// ];

// const thetalist0 = [1.5, 2.5, 3];
// const eomg = 0.01;
// const ev = 0.001;

// const [thetalist, success] = mr.IKinBody(Blist, M, T, thetalist0, eomg, ev);

// console.log("IKinBody输出:");
// console.log("thetalist:", thetalist);
// console.log("success:", success);
// // 期望输出：thetalist 约为 [1.57073819, 2.999667, 3.14153913], success 为 true

// const Slist = [
//     [0,  0,  0],
//     [0,  0,  0],
//     [1,  0,  -1],
//     [4,  0,  -6],
//     [0,  1,  0],
//     [0,  0, -0.1]
// ]; // 6x3，每列为一个关节的螺旋轴

// const M = [
//     [-1, 0,  0, 0],
//     [ 0, 1,  0, 6],
//     [ 0, 0, -1, 2],
//     [ 0, 0,  0, 1]
// ];

// const T = [
//     [0, 1,  0,     -5],
//     [1, 0,  0,      4],
//     [0, 0, -1, 1.6858],
//     [0, 0,  0,      1]
// ];

// const thetalist0 = [1.5, 2.5, 3];
// const eomg = 0.01;
// const ev = 0.001;

// const [thetalist, success] = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev);

// console.log("IKinSpace输出:");
// console.log("thetalist:", thetalist);
// console.log("success:", success);
// // 期望输出：thetalist 约为 [1.57073819, 2.999667, 3.14153913], success 为 true

// console.log("CubicTimeScaling(2, 0.6) =", mr.CubicTimeScaling(2, 0.6)); // 应输出 0.216
// console.log("CubicTimeScaling(2, 0.6) =", mr.QuinticTimeScaling(2, 0.6)); // 应输出 0.163

// Function JointTrajectory
// const thetastart = [1, 0, 0, 1, 1, 0.2, 0, 1];
// const thetaend = [1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1];
// const Tf = 4;
// const N = 6;
// const method = 3; // 3: cubic, 5: quintic

// const traj = mr.JointTrajectory(thetastart, thetaend, Tf, N, method);

// console.log("JointTrajectory输出:");
// traj.forEach((row, i) => {
//     console.log(`点${i}:`, row.map(x => +x.toFixed(4)));
// });

// Function ScrewTrajectory
// const Xstart = [
//     [1, 0, 0, 1],
//     [0, 1, 0, 0],
//     [0, 0, 1, 1],
//     [0, 0, 0, 1]
// ];
// const Xend = [
//     [0, 0, 1, 0.1],
//     [1, 0, 0,   0],
//     [0, 1, 0, 4.1],
//     [0, 0, 0,   1]
// ];
// const Tf = 5;
// const N = 4;
// const method = 3; // 3: cubic, 5: quintic

// const traj = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method);

// console.log("ScrewTrajectory输出:");
// traj.forEach((mat, i) => {
//     console.log(`点${i}:`);
//     mat.forEach(row => console.log(row.map(x => +x.toFixed(3))));
// });

// Function CartesianTrajectory
// const Xstart = [
//     [1, 0, 0, 1],
//     [0, 1, 0, 0],
//     [0, 0, 1, 1],
//     [0, 0, 0, 1]
// ];
// const Xend = [
//     [0, 0, 1, 0.1],
//     [1, 0, 0,   0],
//     [0, 1, 0, 4.1],
//     [0, 0, 0,   1]
// ];
// const Tf = 5;
// const N = 4;
// const method = 5; // 3: cubic, 5: quintic

// const traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method);

// console.log("CartesianTrajectory输出:");
// traj.forEach((mat, i) => {
//     console.log(`点${i}:`);
//     mat.forEach(row => console.log(row.map(x => +x.toFixed(3))));
// });

// const RobotKinematics = require('../modern_robotics/modern_robotics_Kinematics.js');

// // 创建 piper_agilex 机器人运动学对象
// const rk = new RobotKinematics("piper_agilex");

// // 获取 M 和 Slist
// const M = rk.get_M();
// const Slist = rk.get_Slist();

// console.log("M:");
// console.log(M);
// console.log("Slist:");
// console.log(Slist);

// const R_three = [[0, -1, 0],
//                    [0,  0, 1],
//                    [-1, 0, 0]]

// R_world = mr.RotInv(R_three);
// console.log("R_world:", R_world);

// const R_three = [[0, -1, 0],
//                 [0,  0, 1],
//                 [-1, 0, 0]]
// const pos_inital = {x:0.25,y:0,z:0.35}
// const pos_inital_three = mr.matDot(R_three, [pos_inital.x, pos_inital.y, pos_inital.z]);
// console.log("pos_inital_three:", pos_inital_three);

// pos_world = [1, 2, 3];
// pos_three = mr.matDot(R_three, pos_world);
// console.log("pos_world:", pos_three);

const M = [
    [-1, 0,  0, 0],
    [ 0, 1,  0, 6],
    [ 0, 0, -1, 2],
    [ 0, 0,  0, 1]
];

const Slist = [
    [0,  0,  0],
    [0,  0,  0],
    [1,  0,  -1],
    [4,  0,  -6],
    [0,  1,  0],
    [0,  0, -0.1]
]; // 6x3，每列为一个关节的螺旋轴

const Blist = mr.SlistToBlist(M, Slist); // Convert Slist to Blist
console.log("Blist", Blist);