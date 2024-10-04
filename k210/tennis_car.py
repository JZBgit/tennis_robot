import sensor, image, time, lcd
from pid import PID
from fpioa_manager import fm
from Maix import GPIO
from machine import Timer,PWM
import KPU as kpu
import gc, sys

#######################配置IO口引脚###############################
fm.register(31, fm.fpioa.GPIO0)           #毛刷轮电机IO口
besom=GPIO(GPIO.GPIO0, GPIO.OUT)
besom.value(0)

fm.register(16, fm.fpioa.GPIOHS0)         #按键IO口
key = GPIO(GPIO.GPIOHS0, GPIO.PULL_UP)

fm.register(13, fm.fpioa.GPIOHS1)#pin9    #红外测距IO口
car_l = GPIO(GPIO.GPIOHS1, GPIO.PULL_DOWN)
fm.register(14, fm.fpioa.GPIOHS2)#pin8
car_r = GPIO(GPIO.GPIOHS2, GPIO.PULL_DOWN)
fm.register(15, fm.fpioa.GPIOHS3)#pin7
tennis_in = GPIO(GPIO.GPIOHS3, GPIO.PULL_DOWN)
fm.register(32, fm.fpioa.GPIOHS4)#pin6
tennis_out = GPIO(GPIO.GPIOHS4, GPIO.PULL_DOWN)

tim0 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)   #电机IO口
ch0 = PWM(tim0, freq=20000, duty=0, pin=12)#pin10
tim1 = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
ch1 = PWM(tim1, freq=20000, duty=0, pin=11)#pin11
tim2 = Timer(Timer.TIMER0, Timer.CHANNEL2, mode=Timer.MODE_PWM)
ch2 = PWM(tim2, freq=20000, duty=0, pin=10)#pin12
tim3 = Timer(Timer.TIMER0, Timer.CHANNEL3, mode=Timer.MODE_PWM)
ch3 = PWM(tim3, freq=20000, duty=0, pin=3)#pin13

######################初始化LCD屏幕和摄像头######################
lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False) # 颜色跟踪必须关闭自动增益
sensor.set_auto_whitebal(False) # 颜色跟踪必须关闭白平衡

#######################初始化PID、设置阈值、初始化变量###############################
x_pid = PID(p=0.7, i=1.3, imax=100)#0.5 1
h_pid = PID(p=0.02, i=0.1, imax=50)#0.05 0.1

size_threshold = 8000

input_size = (224, 224)
labels = ['tennis']
anchors = [3.84, 5.38, 4.12, 5.56, 3.97, 5.06, 3.63, 5.09, 3.78, 4.66]

global state,t,num
state=5
num=0


################################寻找网球函数######################################
def limit(num):
    if int(abs(num))<10:
        out=10
    elif int(abs(num))>100:
        out=100
    else:
        out=int(abs(num))
    return out
def run(left_speed, right_speed):
    if left_speed < 0:
        ch2.duty(0)
        ch3.duty(limit(left_speed))
    else:
        ch3.duty(0)
        ch2.duty(limit(left_speed))
    if right_speed < 0:
        ch0.duty(0)
        ch1.duty(limit(right_speed))
    else:
        ch1.duty(0)
        ch0.duty(limit(right_speed))

################################获取网球函数######################################
def get_tennis():
    global state
    global t
    global num
    run(45,45)
    # 避障
    if car_l.value()==0:
        run(-40,-40)
        time.sleep(2)
        run(40,-40)
        time.sleep(2)
        state=0
        x_pid.reset_I()
        h_pid.reset_I()
        t = time.ticks_ms()
    if car_r.value()==0:
        run(-40,-40)
        time.sleep(2)
        run(-40,40)
        time.sleep(2)
        state=0
        x_pid.reset_I()
        h_pid.reset_I()
        t = time.ticks_ms()
    # 回收网球
    if tennis_in.value()==0:
        besom.value(1)
        time.sleep(2)
        state=0
        x_pid.reset_I()
        h_pid.reset_I()
        besom.value(0)
        num=num+1
        t = time.ticks_ms()
    if (time.ticks_ms()-t)>2500:
        state=0
        t = time.ticks_ms()

################################结束函数######################################
def finish():
    global state
    global t
    global num
    run(0,0)
    img = image.Image(size=(320, 240))
    # 显示获取到的网球
    img.draw_string(0, 0, "End of ball picking", color=(255, 255, 255), scale=2)
    img.draw_string(0, 30, "Pick up %d balls in total"%(num), color=(255, 255, 255), scale=2)
    img.draw_string(0, 60, "Press the key to continue", color=(255, 255, 255), scale=2)
    lcd.display(img)
    if key.value() == 0:
        time.sleep_ms(30)
        while key.value() == 0:
            state=0
            lcd.rotation(1)
            x_pid.reset_I()
            h_pid.reset_I()
            num=0
            t = time.ticks_ms()

################################开始函数######################################
def begin():
    global state
    global t
    # 显示信息
    img = image.Image(size=(320, 240))
    img.draw_string(0, 0, "Welcome to the intelligent", color=(255, 255, 255), scale=2)
    img.draw_string(0, 30, "tennis robot", color=(255, 255, 255), scale=2)
    img.draw_string(0, 60, "Made by GUET 1800201815-JZB", color=(255, 255, 255), scale=2)
    lcd.display(img)
    # 等待按键按下
    if key.value() == 0:
        time.sleep_ms(30)
        while key.value() == 0:
            state=0
            lcd.rotation(1)
            t = time.ticks_ms()


def lcd_show_except(e):
    import uio
    err_str = uio.StringIO()
    sys.print_exception(e, err_str)
    err_str = err_str.getvalue()
    img = image.Image(size=input_size)
    img.draw_string(0, 10, err_str, scale=1, color=(0xff,0x00,0x00))
    lcd.display(img)

def main(anchors, labels = None, model_addr="/sd/m.kmodel", sensor_window=input_size, lcd_rotation=0, sensor_hmirror=False, sensor_vflip=False):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(sensor_window)
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.run(1)

    lcd.init(type=1)
    lcd.rotation(lcd_rotation)
    lcd.clear(lcd.WHITE)

    if not labels:
        with open('labels.txt','r') as f:
            exec(f.read())
    if not labels:
        print("no labels.txt")
        img = image.Image(size=(320, 240))
        img.draw_string(90, 110, "no labels.txt", color=(255, 0, 0), scale=2)
        lcd.display(img)
        return 1
    try:
        img = image.Image("startup.jpg")
        lcd.display(img)
    except Exception:
        img = image.Image(size=(320, 240))
        img.draw_string(90, 110, "loading model...", color=(255, 255, 255), scale=2)
        lcd.display(img)

    try:
        task = None
        task = kpu.load(model_addr)
        kpu.init(task, 0.5, 0.3, 1, anchors) # threshold:[0,1], nms_value: [0, 1]
        while(True):
            if state==0:
                img = sensor.snapshot()
                t = time.ticks_ms()
                # 模型输出结果
                objects = kpu.forward(task, img)
                t = time.ticks_ms() - t
                if objects:
                    for obj in objects:
                        pos = obj.rect()
                        img.draw_rectangle(pos)
                        img.draw_string(pos[0], pos[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 0, 0))
                        # 计算网球位置及其误差
                        tennis_x=(pos[0]+pos[2])/2
                        x_error = tennis_x-img.width()/2
                        h_error = (pos[2]-pos[0])(pos[3]-pos[1])-size_threshold

                        # 计算pid输出
                        x_output=x_pid.get_pid(x_error,1)
                        h_output=h_pid.get_pid(h_error,1)

                        # 避障检测
                        if car_l.value()==0:
                            run(-40,-40)
                            time.sleep(2)
                            run(40,-40)
                            time.sleep(2)
                        if car_r.value()==0:
                            run(-40,-40)
                            time.sleep(2)
                            run(-40,40)
                            time.sleep(2)
                        # 电机运行
                        run(-h_output+x_output,-h_output-x_output)
                        t = time.ticks_ms()
                        # 走到指定位置后转变状态
                        if abs(-h_output+x_output)<20 and abs(-h_output-x_output)<20:
                            state=1
                    #comm.send_detect_result(objects, labels)
                else:
                    # 没有检测到网球时，转圈
                    run(45,-45)
                    x_pid.reset_I()
                    h_pid.reset_I()
                    # 转一圈没有网球，就结束
                    if (time.ticks_ms()-t)>7000:
                        state=2
                        lcd.rotation(0)
                img.draw_string(0, 200, "t:%dms" %(t), scale=2, color=(255, 0, 0))
                lcd.display(img)
            elif state==1:
                get_tennis()
            elif state==2:
                finish()
            else:
                begin()
    except Exception as e:
        raise e
    finally:
        if not task is None:
            kpu.deinit(task)


if __name__ == "__main__":
    try:
        # main(anchors = anchors, labels=labels, model_addr=0x300000, lcd_rotation=0)
        main(anchors = anchors, labels=labels, model_addr="/sd/yolov3_model.kmodel")
    except Exception as e:
        sys.print_exception(e)
        lcd_show_except(e)
    finally:
        gc.collect()

        
