#-*- coding: UTF-8 -*-
from os import tcsetpgrp
import rclpy
from rclpy.node import Node
import serial
import time
import numpy as np 
from std_msgs.msg import Float32MultiArray


SERIAL_PORT0 = '/dev/ttyUSB0'	# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT1 = '/dev/ttyUSB1'	# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT2 = '/dev/ttyUSB2'	# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT3 = '/dev/ttyUSB3'	# this port address is for the serial tx/rx pins on the GPIO header

SERIAL_RATE = 115200	# be sure to set this to the same rate used on the Arduino

def connectserial():
    try :
        esp32 = serial.Serial(port=SERIAL_PORT0, baudrate=SERIAL_RATE, timeout=.2)
        return esp32
        print('serial port USB0 found \n')
    except:
        print('no serial port USB0 found \n ')
        try : 
            esp32 = serial.Serial(port=SERIAL_PORT1, baudrate=SERIAL_RATE, timeout=.2)
            return esp32
        except:
            print('no serial port USB1 found \n')
            try :
                esp32 = serial.Serial(port=SERIAL_PORT2, baudrate=SERIAL_RATE, timeout=.2)
                return esp32
                print('serial port USB2 found')
            except:
                print('no serial port USB2 found \n')
                try :
                    esp32 = serial.Serial(port=SERIAL_PORT3, baudrate=SERIAL_RATE, timeout=.2)
                    return esp32
                    print('serial port USB3 found')
                except:
                    print('no serial port USB2 found \n')
        



T0 = time.time()

#MPUDATA FORMAT : ACCX ACCY ACCZ GYROX GYROY GYROZ MAGX MAGY MAGZ DT
MPUDATA_0 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_1 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_2 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_3 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_4 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_5 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_6 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_7 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_8 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_9 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_10 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_11 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
MPUDATA_12 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]

LAST_READING_TIME_0 = 10**-5
LAST_READING_TIME_1 = 10**-5
LAST_READING_TIME_2 = 10**-5
LAST_READING_TIME_3 = 10**-5
LAST_READING_TIME_4 = 10**-5
LAST_READING_TIME_5 = 10**-5
LAST_READING_TIME_6 = 10**-5
LAST_READING_TIME_7 = 10**-5
LAST_READING_TIME_8 = 10**-5
LAST_READING_TIME_9 = 10**-5
LAST_READING_TIME_10 = 10**-5
LAST_READING_TIME_11 = 10**-5
LAST_READING_TIME_12 = 10**-5


class RosNode(Node):
    publishers = {} 

    def __init__(self,name):
        super().__init__(name)

    def add_publisher(self,type,name,frequency):
        pub = self.create_publisher(type,name,frequency)
        self.publishers[name] = pub

    def add_subscriber(self,type,name,callback,frequency):
        self.create_subscription(type,name,callback,frequency)

    def publish(self,type,name,data):
        pub = self.publishers[name] 
        msg = type()
        msg.data = data
        pub.publish(msg)


def publish_imu_raw_data(node,MPU_number,MPU_data):
    global MPUDATA_12 , MPUDATA_11  , MPUDATA_11 , MPUDATA_10
    global MPUDATA_9 , MPUDATA_8 , MPUDATA_7 , MPUDATA_6
    global MPUDATA_5 , MPUDATA_4 , MPUDATA_3 , MPUDATA_2
    global MPUDATA_1 , MPUDATA_0
    global LAST_READING_TIME_0 , LAST_READING_TIME_1 , LAST_READING_TIME_2 , LAST_READING_TIME_3
    global LAST_READING_TIME_4 , LAST_READING_TIME_5 , LAST_READING_TIME_6 , LAST_READING_TIME_7
    global LAST_READING_TIME_8 , LAST_READING_TIME_9 , LAST_READING_TIME_10 , LAST_READING_TIME_11
    global LAST_READING_TIME_12
    
    t = time.time() - T0
    if MPU_number == 0 :
        MPUDATA_0[0:8] = MPU_data
        MPUDATA_0[9] = t - LAST_READING_TIME_0 
        node.publish(Float32MultiArray,"MPU0_rawdata" , MPUDATA_0)
        LAST_READING_TIME_0 = time.time() - T0
    if MPU_number == 1 :
        MPUDATA_1[0:8] = MPU_data
        MPUDATA_1[9] = t - LAST_READING_TIME_1
        node.publish(Float32MultiArray,"MPU1_rawdata" , MPUDATA_1)
        LAST_READING_TIME_1 = time.time() - T0
    if MPU_number == 2 :
        MPUDATA_2[0:8]  = MPU_data
        MPUDATA_2[9] = t - LAST_READING_TIME_2
        node.publish(Float32MultiArray,"MPU2_rawdata" , MPUDATA_2)
        LAST_READING_TIME_2 = time.time()- T0
    if MPU_number == 3 :
        MPUDATA_3[0:8]  = MPU_data
        MPUDATA_3[9] = t - LAST_READING_TIME_3
        node.publish(Float32MultiArray,"MPU3_rawdata" , MPUDATA_3)
        LAST_READING_TIME_4 = time.time()- T0
    if MPU_number == 4 :
        MPUDATA_4[0:8] = MPU_data
        MPUDATA_4[9] = t - LAST_READING_TIME_4    
        node.publish(Float32MultiArray,"MPU4_rawdata" , MPUDATA_4)
        LAST_READING_TIME_4 = time.time() - T0
    if MPU_number == 5 :
        MPUDATA_5[0:8] = MPU_data
        MPUDATA_5[9] = t - LAST_READING_TIME_5
        node.publish(Float32MultiArray,"MPU5_rawdata" , MPUDATA_5)
        LAST_READING_TIME_5 = time.time()- T0
    if MPU_number == 6 :
        MPUDATA_6[0:8] = MPU_data
        MPUDATA_6[9] = t - LAST_READING_TIME_6
        node.publish(Float32MultiArray,"MPU6_rawdata" , MPUDATA_6)
        LAST_READING_TIME_6 = time.time()- T0
    if MPU_number == 7 :
        MPUDATA_7[0:8] = MPU_data
        MPUDATA_7[9] = t - LAST_READING_TIME_7
        node.publish(Float32MultiArray,"MPU7_rawdata" , MPUDATA_7)
        LAST_READING_TIME_7 = time.time()- T0
    if MPU_number == 8 :
        MPUDATA_8[0:8] = MPU_data
        MPUDATA_8[9] = t - LAST_READING_TIME_8
        node.publish(Float32MultiArray,"MPU8_rawdata" , MPUDATA_8)
        LAST_READING_TIME_8 = time.time()- T0
    if MPU_number == 9 :
        MPUDATA_9[0:8] = MPU_data
        MPUDATA_9[9] = t - LAST_READING_TIME_9
        node.publish(Float32MultiArray,"MPU9_rawdata" , MPUDATA_9)
        LAST_READING_TIME_9 = time.time()- T0
    if MPU_number == 10 :
        MPUDATA_10[0:8] = MPU_data
        MPUDATA_10[9] = t - LAST_READING_TIME_10
        node.publish(Float32MultiArray,"MPU10_rawdata" , MPUDATA_10)
        LAST_READING_TIME_10 = time.time()- T0
    if MPU_number == 11 :
        MPUDATA_11[0:8] = MPU_data
        MPUDATA_11[9] = t - LAST_READING_TIME_11
        node.publish(Float32MultiArray,"MPU11_rawdata" , MPUDATA_11)
        LAST_READING_TIME_11 = time.time()- T0
    if MPU_number == 12 :
        MPUDATA_12[0:8] = MPU_data
        MPUDATA_12[9] = t - LAST_READING_TIME_12
        node.publish(Float32MultiArray,"MPU12_rawdata" , MPUDATA_12)
        LAST_READING_TIME_12 = time.time()- T0
    else:
        pass
    
    MPUDATA_0 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_1 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_2 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_3 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_4 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_5 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_6 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_7 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_8 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_9 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_10 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_11 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]
    MPUDATA_12 = [.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0 ]


def create_raw_data_dispatcher(node):
        node.add_publisher(Float32MultiArray,"MPU0_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU1_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU2_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU3_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU4_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU5_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU6_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU7_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU8_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU9_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU10_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU11_rawdata" , 50)
        node.add_publisher(Float32MultiArray,"MPU12_rawdata" , 50)

def get_data_from_esp32():
    global MPUDATA_12 , MPUDATA_11  , MPUDATA_11 , MPUDATA_10
    global MPUDATA_9 , MPUDATA_8 , MPUDATA_7 , MPUDATA_6
    global MPUDATA_5 , MPUDATA_4 , MPUDATA_3 , MPUDATA_2
    global MPUDATA_1 , MPUDATA_0 
    
    reading = esp32.readline().decode('UTF-8',"strict")
    print(f"data from esp  {reading}")
    if reading != "":
        try:
            MPU_number,MPU_data = get_array(reading)
            return MPU_number,MPU_data
        except:
            print('bad format')

def get_array(reading):    
    reading = reading.replace("\n",'')
    reading = reading.replace("\r",'')
    reading = reading.replace(" ",'')
    stringarray = reading.split("\t")
    data = np.zeros(10)
    for i,string in enumerate(stringarray) :
        string = string.replace("\n",'')
        string = string.replace("\r",'')
        string = string.replace(" ",'')
        try:
            data[i] = float(string)
        except:
            print("error in conversion to float , bad format")
    data = data.tolist()
    MPU_number = data[0]
    MPU_data = data[1:10]
    return MPU_number,MPU_data


def main(args=None):
    global esp32
    rclpy.init(args=args)
    esp32 = connectserial()
    node = RosNode('ESP32_SerialInterface')
    create_raw_data_dispatcher(node)

    while True:
        try:
            MPU_number,MPU_data = get_data_from_esp32()   
            publish_imu_raw_data(node,MPU_number,MPU_data)
        except:
            print('error, data not procesed')
            time.sleep(5)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

