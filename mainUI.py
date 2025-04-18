
import rospy
import tkinter as tk
from robot_msgs.msg import messages, hardware
from pynput.keyboard import Key,Listener
import keyboard_


rospy.init_node('robot_console')
pub = rospy.Publisher('robot_command', messages, queue_size=3)
drivePub = rospy.Publisher('robot_control', hardware, queue_size=3)


def start():
    cmd = hardware()
    cmd.motor_mode = [3 for i in range(8)] 
    cmd.motor_control = [15 for i in range(8)] 
    cmd.target_velocity = [0 for i in range(8)]
    drivePub.publish(cmd)

def stop():
    cmd = hardware()
    cmd.motor_mode = [3 for i in range(8)] 
    cmd.motor_control = [6 for i in range(8)] 
    cmd.target_velocity = [0 for i in range(8)]
    drivePub.publish(cmd)

def _keyboard():
    rospy.loginfo('使用键盘')
    with Listener(on_press=keyboard_.on_press, 
            on_release=keyboard_.on_release) as listener:
        listener.join()

def zeroback():
    cmd = hardware()
    cmd.motor_mode = [6 for i in range(8)] 
    cmd.motor_control = [31 for i in range(8)] 
    drivePub.publish(cmd)

def lowerGripper():
    cmd = hardware()
    cmd.agent = 'lower'
    cmd.gripper_operation = 'grasp'
    drivePub.publish(cmd)
    rospy.sleep(1)
    cmd.gripper_operation = 'rollPos'
    drivePub.publish(cmd)
    rospy.sleep(2)
    cmd.gripper_operation = 'rollNeg'
    drivePub.publish(cmd)
    rospy.sleep(1)
    cmd.gripper_operation = 'release'
    drivePub.publish(cmd) 

def upperGripper():
    cmd = hardware()
    cmd.agent = 'upper'
    cmd.gripper_operation = 'grasp'
    drivePub.publish(cmd)
    rospy.sleep(1)
    cmd.gripper_operation = 'rollPos'
    drivePub.publish(cmd)
    rospy.sleep(2)
    cmd.gripper_operation = 'rollNeg'
    drivePub.publish(cmd)
    rospy.sleep(1)
    cmd.gripper_operation = 'release'
    drivePub.publish(cmd) 

def lowerCyinder():
    cmd = hardware()
    cmd.switch_operation = 'lower_extend'
    drivePub.publish(cmd)
    rospy.sleep(2)
    cmd.switch_operation = 'lower_retract'
    drivePub.publish(cmd)

def upperCyinder():
    cmd = hardware()
    cmd.switch_operation = 'upper_extend'
    drivePub.publish(cmd)
    rospy.sleep(2)
    cmd.switch_operation = 'upper_retract'
    drivePub.publish(cmd)

def plan():
    print("规划")

# 创建窗口
window = tk.Tk()
window.title("按钮示例")

# 创建按钮并分配函数
start_button = tk.Button(window, text="启动", command=start)
start_button.pack()

stop_button = tk.Button(window, text="停止", command=stop)
stop_button.pack()

execute_button = tk.Button(window, text="键盘", command=_keyboard)
execute_button.pack()

plan_button = tk.Button(window, text="回零点", command=zeroback)
plan_button.pack()

LG_button = tk.Button(window, text="上手爪", command=lowerGripper)
LG_button.pack()

UG_button = tk.Button(window, text="下手爪", command=upperGripper)
UG_button.pack()

LC_button = tk.Button(window, text="下气缸", command=lowerCyinder)
LC_button.pack()

UC_button = tk.Button(window, text="上气缸", command=upperCyinder)
UC_button.pack()

# 运行窗口
window.mainloop()
