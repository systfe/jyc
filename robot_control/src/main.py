import rclpy
import time
from DriveControl import PurePursuit
import random
rclpy.init()
drive = PurePursuit()

drive.Print_pose()
drive.Move(0,0,0)
time.sleep(5)
while 1:
    x=random.uniform(-1.2,1.2)
    y=random.uniform(-1.2,1.2)
    yaw=0#random.randint(0,360)
    print(f"目标：({x:.2f},{y:.2f},{yaw})")
    drive.Move(x,y,yaw)
    time.sleep(4)

drive.destroy_node()
rclpy.shutdown()