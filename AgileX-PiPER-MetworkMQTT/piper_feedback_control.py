#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
from typing import (
    Optional,
)
import time
from piper_sdk import *
import numpy as np

from PiPER_Get_Joint_Feedback import get_joint_feedback

def enable_fun(piper: C_PiperInterface):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        print("使能状态:", enable_flag)
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        print("--------------------")
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print("超时....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if (elapsed_time_flag):
        print("程序自动使能超时,退出程序")
        exit(0)


if __name__ == "__main__":
    piper = C_PiperInterface("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    time.sleep(0.1)

    msg = piper.GetArmJointMsgs()
    factor = 57324.840764  # 1000*180/3.14

    Kp = 0.8
    position = [0, -0.27473 + np.radians(90), 1.44144 - np.radians(169.997), 0, 1.22586 - 0.03, 0]
    # position = np.array([0, 0, 0, 0, -0.03, 0])

    while True:
        joint_feedback = get_joint_feedback(msg, factor)
        print("joint_feedback", joint_feedback)

        joint_feedback = np.array(joint_feedback)
        error = position - joint_feedback
        mse = np.mean(error ** 2)  # 均方误差
        print(f"MSE: {mse:.8f}")

        if mse < 1e-6:  # 约等价于各关节误差均 < 1e-3 rad
            print("Movement completed. Stopping control loop.")
            break

        control_signal = joint_feedback + Kp * error

        joint_cmd = np.round(control_signal * factor).astype(int)
        piper.MotionCtrl_2(0x01, 0x01, 5, 0x00)

        print("send signal:", *joint_cmd.tolist())
        piper.JointCtrl(*joint_cmd.tolist())

        time.sleep(0.02)


