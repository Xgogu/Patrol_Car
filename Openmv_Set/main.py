#调用现有一些库函数
import sensor, image, time
from pyb import Pin
#导入pybZ这个模块，里面包含基础外设的各种类
from pyb import LED  # 导入pyb包里面，LED这个模块
import pyb

# 常用的延时函数
pyb.delay(50) # 延时 50 毫秒
pyb.millis() # 获取从启动开始计时的毫秒数

#创建对象
led_red = LED(1)# LED1是红色的灯
led_green = LED(2)# LED2是绿色的灯
led_blue = LED(3)# LED3是蓝色的灯


# 输出LED的极性
led_red.toggle() #翻转红色的灯
led_red.off() # 关闭红色LED（LED1）
led_red.on()# 开启红色LED（LED1）

led_green.toggle() #翻转绿色的灯
led_green.off() # 关闭绿色LED（LED2）
led_green.on()# 开启绿色LED（LED2）

led_blue.toggle() #翻转蓝色的灯
led_blue.off() # 关闭蓝色LED（LED3）
led_blue.on()# 开启蓝色LED（LED3）

#初始化p0引脚
pin7 = Servo(1)
pin8 =Pin('P8', Pin.IN, Pin.PULL_DOWN)
pin9 =Pin('P9', Pin.IN, Pin.PULL_DOWN)
pin0 =Pin('P0', Pin.OUT_PP, Pin.PULL_DOWN)
#待调参数：

target_threshold =(10, 88, -73, -19, -36, 30)#标定颜色，标定前请解除“  #调参时打开：”底下那行的注释
target_threshold_yellow =(10, 88, -73, -19, -36, 30)
target_threshold_red =(44, 65, -54, 109, -67, -23)
target_threshold_blue =(40, 85, -65, 84, -67, -17)





K = 0.0357  #标定大小系数K，调试前请解除“  #调参时打开：”底下那行的注释



#'''
#当测另一个物体的大小时，需要将该物体放在求比例系数K时的那个距离处，此处我们使用的是10cm处
#实际大小 = 直径的像素 * K。标定时将小球放在距离摄像头10cm处的位置，测得Lm的大小之后，K = 实际大小 / Lm
#比如在10cm处时，直径的像素为43，小球实际大小为3cm，则K = 3 / 43 = 0.07，具体情况以你的实际情况为准
#'''


# 初始化函数
def init_setup():
    global sensor, clock                   # 设置为全局变量
    sensor.reset()                         # 初始化感光元件
    sensor.set_pixformat(sensor.RGB565)    # 设置感光元件图像色彩格式为 RGB565 (RGB565为彩图，GRAYSCALE为灰度图)
    sensor.set_framesize(sensor.QVGA)      # 设置感光元件分辨率大小为 QVGA (QVGA是320x240)
    sensor.skip_frames(time=2000)          # 等待2秒，使以上的设置生效
    sensor.set_auto_gain(False)            # 关闭自动增益。在颜色识别中，需要关闭自动增益
    sensor.set_auto_whitebal(False)        # 关闭白平衡。白平衡是默认开启的，在颜色识别中，需要关闭白平衡
    clock = time.clock()                   # 创建时钟对象

def main():
    while 1:
        pin0.value(0)

        size = 0 #将五块大小设置为0
        img = sensor.snapshot()           # 拍一张照片并返回图像


        blobs_r = img.find_blobs([target_threshold_red],x_stride=100,y_stride=100)  # 找到图像中的目标区域
        blobs_y = img.find_blobs([target_threshold_yellow],x_stride=100,y_stride=100)
        blobs_b = img.find_blobs([target_threshold_blue],x_stride=100,y_stride=100)
        b=[]

        if pin8.value()==True：
            if len(blobs_r)==1:       #识别结果为红
                b = blobs_r[0]
                pin0.value(True)        #将p0置为高电平
                led_green.off()
                led_red.on()
                led_blue.off()

            if len(blobs_y) == 1:       #识别结果为绿
                b = blobs_y[0]
                pin0.value(True)
                led_green.on()
                led_red.off()
                led_blue.on()
            if b != []:
                img.draw_rectangle(b[0:4])    # 将目标区域框出来
                img.draw_cross(b[5], b[6])    # 在目标中心位置画十字

       if pin9.value()==True：
          if len(blobs_b) == 1 :       #识别结果为红
                b = blobs_b[0]
                pin0.value(True)        #将p0置为高电平
                led_green.off()
                led_red.off()
                led_blue.on()

          if len(blobs_y) == 1:       #识别结果为绿
                b = blobs_y[0]
                pin0.value(True)
                led_green.on()
                led_red.off()
                led_blue.on()

          if b != []:
                img.draw_rectangle(b[0:4])    # 将目标区域框出来
                img.draw_cross(b[5], b[6])    # 在目标中心位置画十字


        print((pin0.value(),size)) #查看p0口电平和测量结果值
        print((pin8.value(),size))
        print((pin9.value(),size))
        time.sleep_ms(50)



# 程序入口
if __name__ == '__main__':
    init_setup()  # 执行初始化函数
    main()    # 执行主函数
