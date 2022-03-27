import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
from wt_imu import WT_IMU

#列出所有当前的com口
port_list = list(serial.tools.list_ports.comports())
port_list_name = []

def show_all_com():
    if len(port_list) <= 0:
        print("the serial port can't find!")
    else:
        for itms in port_list:
            port_list_name.append(itms.device)


class SerialPort:
    def __init__(self, port, buand):
        self.port = serial.Serial(port, buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self, date):
        print("send:",date)
        self.port.write(date.encode())
        time.sleep(1)

    def read_data(self):
        while True:
            count = self.port.inWaiting()
            if count > 0:
                rec_str = self.port.read(count)
                print("receive:",rec_str.decode())

    def send_test(self, imu=None):
        #date = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        while True:
            imu_data = display_VOFA(wt_imu)
            date = datetime.now().strftime('%H:%M:%S.%f')[:-3]+"\n"
            print("send:",imu_data)
            self.port.write(imu_data.encode())
            # time.sleep(1)

    def read_test(self):
        while True:
            count = self.port.inWaiting()
            if count > 0:
                rec_str = self.port.read(count)
                print("receive:",rec_str.decode())


def display_VOFA(wt_imu):
    return _update_canvas(wt_imu)


def _update_canvas(wt_imu):
    try:
        imu_datas = wt_imu.getData()
    except Exception as e:
        print("---串口异常---：", e)
        wt_imu.ser.close()  # 关闭串口
        wt_imu._is_open = False

    data_a = [*imu_datas[0]] # 加速度
    data_w = [*imu_datas[1]] # 角速度
    data_angle = [*imu_datas[2]] # 角度

    # 请按照实际 VOFA上位机串口通讯协议更改！ 此处是基于FireWater协议发送
    return "channels: "+str(data_a[0])+","+str(data_a[1])+","+str(data_a[2])+","\
                       +str(data_w[0])+","+str(data_w[1])+","+str(data_w[2])+","\
                       +str(data_angle[0])+","+str(data_angle[1])+","+str(data_angle[2])\
                       +"\n"



# 这里提供了一个测试用例，提供串口的收发
# 可创建本地两个虚拟串口（windows安装虚拟串口.zip里面的程序即可），通过开启收发线程循环执行收发。
if __name__ == '__main__':
    baunRate = 115200
    serialPort_r = "COM5"          # 接受IMU数据的端口
    serialPort_w = "COM4"          # 向VOFA发送数据的端口（本地虚拟）
    wt_imu = WT_IMU(port=serialPort_r, bps=baunRate)
    # data = display_VOFA(wt_imu)

    print("1.list all com")
    show_all_com()
    print(port_list_name)

    print("2.open write port ",serialPort_w)
    mSerial_w = SerialPort(serialPort_w,baunRate)

    print("3.start write thread")
    t1 = threading.Thread(target=mSerial_w.send_test(wt_imu))
    t1.setDaemon(True)
    t1.start()

    #do something else, make main thread alive there
    while True:
        time.sleep(10)





