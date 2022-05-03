# -*- coding: utf-8 -*
import serial  # 串口通信库
import time
from pid import PID  # pid库，有关机械臂控制
import apriltag  # 二维码库
import cv2  # opencv视觉库
import numpy as np


def arm_init():  # 初始化机械臂
    global M, M_a, pan_pid, tilt_pid, dist_pid
    # 设置每个舵机的速度，将速度降为18°/s，下面数组中第三个参数代表第几个舵机，第四个参数为2代表速度：2*9°，就是18°/s,
    # 关于舵机控制板的通讯协议，详见文件16路舵机控制板使用说明V2.0中6.1.1的协议说明
    # (ser.write)十六进制字符串
    # bytearray() 方法返回一个新字节数组。这个数组里的元素是可变的，并且每个元素的值范围: 0 <= x < 256。
    ser.write(bytearray([0xff, 0x01, 0x00, 0x04, 0x00]))
    ser.write(bytearray([0xff, 0x01, 0x01, 0x04, 0x00]))
    ser.write(bytearray([0xff, 0x01, 0x02, 0x04, 0x00]))
    ser.write(bytearray([0xff, 0x01, 0x03, 0x08, 0x00]))

    # 四个舵机分别插在舵机控制板上的M0,M1,M2,M3。 
    # M0是底盘舵机控制左右旋转，M1是右舵机控制前后，M2是左舵机控制上下,M3是爪子舵机
    M = [(15, 180), (30, 130), (30, 130), (30, 120), (), (), (), (), (), (), ()]  # 四个舵机可旋转的角度范围
    M_a = [90, 60, 75, 45, 0, 0, 0, 0, 0, 0, 0]  # 声明数组，存储四个舵机的实时角度

    # 设定初始角度
    set_angle(90, 0)
    set_angle(60, 1)
    set_angle(75, 2)
    set_angle(45, 3)

    time.sleep(5)

 # 检查舵机角度是否在范围内，angle代表角度，CH代表通道，若是舵机M0，则输入CH=0
def check_angle(angle, CH): 
    M_a[CH] = angle
    if M_a[CH] < M[CH][0]:  # 不在范围内，就设置成允许的最大或最小度数
        M_a[CH] = M[CH][0]
    elif M_a[CH] > M[CH][1]:
        M_a[CH] = M[CH][1]


def set_angle(angle, CH):  # 设定角度函数，输入一个指定角度和指定通道
    check_angle(angle, CH)  # 先检查要设定的角度是否在范围内
    us = int(M_a[CH] * (2000 / 180.0)) + 500  # 见文件16路舵机控制板使用说明V2.0中6.1.2的协议说明
    DL = us & 0xff
    DH = us >> 8
    angle = bytearray([0xff, 0x02, CH, DL, DH])
    ser.write(angle)


# 留意
def get_distance():
    # 获取距离，因为距离传感器只能在python2的环境下运行，无法和本程序一起运行，
    # 所以让距离传感器单独在另外一个python2程序中运行，将距离存入txt文件，本程序则通过txt读取实时距离
    
    fo = open("/home/pi/VL53L0X/python/dis.txt", "r")
    dis = fo.read(10).replace(" ", "")
    
    return (dis)

# 处理图像数据，返回色块坐标
def get_colorxy(img, red, blue, green):  
    color_id = 0  # 颜色标记，1为红，2为蓝，3为绿
    for i in (red, blue, green):  # 依次取颜色，探测视野内是否存在色块
        color_id = color_id + 1
        lower = i[0:3]
        upper = i[3:6]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 将捕获的视频帧由RBG转HSV
        kernel = np.ones((3, 3), np.uint8)

        # 将处于lower_green 和upper_green 区间外的值全部置为0，区间内的值置为255
        mask = cv2.inRange(hsv, lower, upper)

        # 对mask图像进行腐蚀，将一些小的白色区域消除，将图像“变瘦”， iterations代表使用erode的次数
        # erode就是让图像中白色部分变小
        mask = cv2.erode(mask, kernel, iterations=2)

        # 开运算 （MORPH_OPEN）:先腐蚀再膨胀
        # 删除不能包含结构元素的对象区域，平滑图像的轮廓，使拐点的地方更加连贯，断开一些狭窄的链接，去掉细小的突出部分。
        # 在这里使用开运算就是为了使除笔头外的噪声区域尽量的消除
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # dilate膨胀就是将白色区域变大，黑色的区域减小
        mask = cv2.dilate(mask, kernel, iterations=5)

        '''
        findContours() 查找检测物体的轮廓。
        第一个参数是寻找轮廓的图像；
        第二个参数表示轮廓的检索模式，有四种（本文介绍的都是新的cv2接口）：
            cv2.RETR_EXTERNAL表示只检测外轮廓
            cv2.RETR_LIST检测的轮廓不建立等级关系
            cv2.RETR_CCOMP建立两个等级的轮廓，上面的一层为外边界，里面的一层为内孔的边界信息。
            如果内孔内还有一个连通物体，这个物体的边界也在顶层。
            cv2.RETR_TREE建立一个等级树结构的轮廓。
        第三个参数method为轮廓的近似办法
            cv2.CHAIN_APPROX_NONE存储所有的轮廓点，相邻的两个点的像素位置差不超过1，
            即max（abs（x1-x2），abs（y2-y1））==1
            cv2.CHAIN_APPROX_SIMPLE压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标，
            例如一个矩形轮廓只需4个点来保存轮廓信息
            cv2.findContours()函数返回两个值:contours：hierarchy，一个是轮廓本身，还有一个是每条轮廓对应的属性。
            findContours函数首先返回一个list(即contours)，list中每个元素都是图像中的一个轮廓，用numpy中的ndarray表示
        '''
        cnts, heir = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        #Python len() 方法返回对象（字符、列表、元组等）长度或项目个数
        if len(cnts) > 0:# 如果检测出了轮廓
            c = cnts[0]
            ((x1, y1), radius1) = cv2.minEnclosingCircle(c) #cv2.minEnclosingCircle：求最小包围圆的算法，返回：center - 圆心  (x, y)
                                                                                                           # radius - 半径  r
            
            for c in cnts: # 取最左上角的一个
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if x<x1 and y<y1:
                    x1 = x
                    y1 = y
                    radius1 = radius
                
                # 找出轮廓中最小的圆
            if radius1>20: # 半径大于20才返回
                return (x1, y1, radius1,color_id)  # 返回x坐标，y坐标，半径,颜色标记


def what_color(img,red,blue,green):  #检测是什么颜色，不同颜色给出不同返回值
    if get_colorxy(img,red,blue,green):
        color = get_colorxy(img,red,blue,green)[3]
        return color
    else:
        return None


red = np.array([149, 62, 0, 255, 255, 255]) # 前三个填最小值，后三个填最大值
blue = np.array([86, 69, 111, 255, 255, 255])
green = np.array([30, 107, 47, 83, 255, 255])

#记录色块总数
red_sum = 0
blue_sum = 0
green_sum = 0

#记录apriltag的id，红色放在id为1的二维码处，蓝色放在id为24的二维码处理，绿色放在25
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11  tag25h9')) # 声明apriltag探测器
tagid = 0


if __name__ == '__main__':
    ser = serial.Serial("/dev/ttyAMA0", 9600)  # 初始化串口
    arm_init()  # 初始化机械臂
    cap = cv2.VideoCapture(0)  # 初始化摄像头
    cx = 320  # 画面的中心坐标
    cy = 240

    while (1):
        ret, img = cap.read()  # 获取画面
       
        tag = get_colorxy(img,red,blue,green)  # 获取坐标，先抓红色
        
        dis = get_distance() #  获取距离
    

        # pid参数调节，我已经调好了，不要动他。如果非要调，参照调节教程https://blog.csdn.net/zyboy2000/article/details/9418257
        pan_pid = PID(p=0.005, i=0, imax=90)
        tilt_pid = PID(p=0.01, i=0, imax=90)
        dist_pid = PID(p=0.01, i=0, imax=90)

        if tag and dis:  # 如果色块存在
            # 获取色块的坐标和半径
            mx = int(tag[0])
            my = int(tag[1])
            radius = int(tag[2])
            color_id = tag[3] # 指明何种颜色
            
            if color_id == 1:
                tagid=1
                red_sum = red_sum+1
            elif color_id == 2:
                tagid=24
                blue_sum = blue_sum+1
            elif color_id ==3:
                tagid=25
                green_sum = green_sum+1
            
                
            # 画出色块
            cv2.rectangle(img, (int(mx) - 10, int(my) - 10), (int(mx + 10), int(my + 10)), (255, 0, 0), 1)
            cv2.circle(img, (int(mx), int(my)), int(radius), (0, 255, 255), 2)
            
            # 计算色块坐标和画面中心坐标的差值，以及距离传感器返回值与需要达到的距离的差值
            pan_error = mx - (cx-80) # 色块坐标和画面中爪子坐标的偏差，单位（像素）
            tilt_error = my - (cy+60)
            dist_error = int(dis) - 100 # 当前距离和目标距离100mm的差值

            # 将差值转化成要转的度数
            pan_output = pan_pid.get_pid(pan_error, 1)
            tilt_output = tilt_pid.get_pid(tilt_error, 1)
            dist_output = dist_pid.get_pid(dist_error, 1)

            # 设定角度
            set_angle(M_a[0] - pan_output, 0)
            set_angle(M_a[1] + dist_output, 1)
            set_angle(M_a[2] + tilt_output, 2)

            print(11, pan_error, tilt_error, dist_error)
            if abs(pan_error) < 5 and abs(tilt_error) < 5 and abs(dist_error) < 3: #如果差值在很小的范围内，认为已经达到夹取标准，跳入步骤2
                print(11, pan_error, tilt_error, dist_error)
                set_angle(100, 3)  # 设置爪子舵机105，即夹紧
                time.sleep(2)  # 等待4秒
                set_angle(75,1)
                time.sleep(1)
                set_angle(65,2)
                time.sleep(1)
                set_angle(160,0)
            
                time.sleep(2)
                
                step = 1
                while(step == 1): # 找二维码
                    pan_pid = PID(p=0.005, i=0, imax=90) #底盘M0舵机pid参数
                    tilt_pid = PID(p=0.005, i=0, imax=90) #左M1舵机pid参数
                    dist_pid = PID(p=0.01 ,i=0, imax=90) #右M2舵机pid参数
                    
                    ret, img = cap.read()
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    tags = at_detector.detect(gray)
                    
                    dis = get_distance() #  获取距离
                    
                    if tags and dis:
                        for tag in tags:
        
                            if tag.tag_id == tagid: 
                                mx = int((tag.corners[0].astype(int)[0] + tag.corners[2].astype(int)[0]) / 2)
                                my = int((tag.corners[0].astype(int)[1] + tag.corners[2].astype(int)[1]) / 2)
                    
                                cv2.line(img, (mx - 10, my), (mx + 10, my), (0, 255, 0), 2)
                                cv2.line(img, (mx, my - 10), (mx, my + 10), (0, 255, 0), 2)
                                cv2.circle(img, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
                                cv2.circle(img, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
                                cv2.circle(img, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
                                cv2.circle(img, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom
                                
                                pan_error = mx - (cx-70) # 色块坐标和画面中二维码坐标的偏差，单位（像素）
                                tilt_error = my - (cy-60)
                                dist_error = int(dis) - 120# 当前距离和目标距离115mm的差值

                                # 将差值转化成要转的度数
                                pan_output = pan_pid.get_pid(pan_error, 1)
                                tilt_output = tilt_pid.get_pid(tilt_error, 1)
                                dist_output = dist_pid.get_pid(dist_error, 1)

                                # 设定角度
                                set_angle(M_a[0] - pan_output, 0)
                                set_angle(M_a[1] + dist_output, 1)
                                set_angle(M_a[2] + tilt_output, 2)
                                
                                cv2.imshow("img", img)
                                k = cv2.waitKey(1)
                                if k == 27:
                                    break

                                print(22, pan_error, tilt_error, dist_error)
                                if abs(pan_error) <10  and abs(tilt_error) < 10 and abs(dist_error) < 5: #如果差值在很小的范围内，认为已经达到夹取标准，跳入步骤2
                                    print(22, pan_error, tilt_error, dist_error)
                                    set_angle(45, 3)  # 设置爪子舵机100，即夹紧
                                    time.sleep(2)  # 等待4秒
                                    set_angle(60, 1)
                                    time.sleep(1)
                                    set_angle(75, 2)
                                    time.sleep(1)
                                    set_angle(90, 0)
                                    time.sleep(2)
                                    
                                    step=0
                                
                

                    
                                
                            
        #elif not tag:
         #   break
                            
                
                           
        cv2.imshow("img", img)
        k = cv2.waitKey(1)
        if k == 27:
            break


print("红色：",red_sum)











   