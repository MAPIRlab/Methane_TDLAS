import rclpy
from rclpy.node import Node
import serial
from .mtdef import MID
import time
import struct
from sensor_msgs.msg import Imu,MagneticField
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler,euler_from_matrix, quaternion_from_matrix,rotation_matrix, quaternion_multiply
import numpy
from collections import deque

import math

class XsensDriver(Node):
    ENU = numpy.identity(3)
    NWE = numpy.array([[1, 0, 0], [ 0, -1, 0], [0, 0, 1]])
    NED = numpy.array([[1, 0, 0,0], [0, 1, 0,0], [0, 0,  1,0],[0,0,0,1]])
 
    def __init__(self):
        super().__init__('serial_publisher')
        self.hist_imu = deque(maxlen=20)
        self.var = numpy.zeros(9)
        self.declare_parameter("verbose",False)
        self.verbose = self.get_parameter("verbose").value
        
        self.declare_parameter("port","/dev/ttyUSB0")
        self.serial_port = self.get_parameter('port').value  # Cambia esto al puerto serial que estés usando
        
        self.declare_parameter("baudrate",115200)
        self.serial_baudrate = self.get_parameter("baudrate").value  # Cambia esto a la velocidad de baudios adecuada
        
        self.declare_parameter("sim_time",False)
        self.sim_time = self.get_parameter("sim_time").value
        
        self.declare_parameter("outputMode",2)
        self.outMode = self.get_parameter("outputMode").value
        self.get_logger().info(f"outmode: {self.outMode}")
        
        self.declare_parameter("pub_magnetometer",True)
        self.pub_mag = self.get_parameter("pub_magnetometer").value
        
        self.declare_parameter("pub_rpy",True)
        self.pub_rpy = self.get_parameter("pub_rpy")
        
        self.declare_parameter("imu_topic","/mti/imu")
        self.imu_topic = self.get_parameter("imu_topic").value
        self.declare_parameter("mag_topic","/mti/mag")
        self.mag_topic = self.get_parameter("mag_topic").value
                
        self.imu_pub = self.create_publisher( Imu,self.imu_topic, 10) #IMU message
        self.mag_pub = self.create_publisher(MagneticField,self.mag_topic,10) # Magnetometer message
        if(self.sim_time):
            self.clock_sim_tim = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        if(self.pub_rpy):
            self.rpy_pub = self.create_publisher( Point,"imu/rpy", 10) #IMU message

        self.sim_clock_time = Clock()
        self.timeout = 0.4
        
        self.serial = serial.Serial(self.serial_port, self.serial_baudrate)

        # while not(self.GoToConfig()):
        #     self.GoToConfig()
        # self.get_logger().fatal('pasado el config')
        # while not(self.Configure()):
        #     self.Configure()
        # self.get_logger().fatal('pasado el configure')

        # while not(self.GoToMeasure()):
        #     self.GoToMeasure()
        # self.get_logger().fatal('pasado el meassure')

        
        hex_sequence = "FA FF 30 00 D1"
        hex_bytes = bytes.fromhex(hex_sequence.replace(' ', ''))
        self.serial.write(hex_bytes)
        while not(self.check_ack(0x30)):
            self.check_ack(0x30)
        hex_sequence = "FA FF D0 02 00 06 29"
        hex_bytes = bytes.fromhex(hex_sequence.replace(' ', ''))
        self.serial.write(hex_bytes)
        while not(self.check_ack(0xD0)):
            self.check_ack(0xD0)
        hex_sequence = "FA FF D2 04 00 00 00 09 22"
        hex_bytes = bytes.fromhex(hex_sequence.replace(' ', ''))
        #print("%i"%(0xFF&(-(sum(hex_bytes[1:])))))
        self.serial.write(hex_bytes)
        while not(self.check_ack(0xD2)):
            self.check_ack(0xD2)
        # print("%i"%(0xFF&(-(sum(hex_bytes[1:])))))
        hex_sequence = "FA FF 10 00 F1"
        hex_bytes = bytes.fromhex(hex_sequence.replace(' ', ''))
        self.serial.write(hex_bytes)
        while not(self.check_ack(0x10)):
            self.check_ack(0x10)
        self.get_logger().info("COnfiguracion completada con exito")
        
    def clock_callback(self,msg):
        self.sim_clock_time = msg
        
    def spin_once(self):
        data = self.read_msg()
        imu_data = self.data_decode(data)
        matrix = []
        # self.get_logger().info('%s'%(str(imu_data[8:])))
        if imu_data:
            imu_msg = Imu()
            mag_msg = MagneticField()
            rpy_msg = Point()
            mag_msg.header.frame_id = "mti_imu_frame"
            imu_msg.header.frame_id = "mti_imu_frame"
            if self.sim_time:
                mag_msg.header.stamp = self.sim_clock_time.clock
                imu_msg.header.stamp = self.sim_clock_time.clock
            else:
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.linear_acceleration.x = imu_data[0]
            imu_msg.linear_acceleration.y = imu_data[1]
            imu_msg.linear_acceleration.z = imu_data[2]
                        
            imu_msg.angular_velocity.x = imu_data[3]
            imu_msg.angular_velocity.y = imu_data[4]
            imu_msg.angular_velocity.z = imu_data[5]
            
            mag_msg.magnetic_field.x = imu_data[6]
            mag_msg.magnetic_field.y = imu_data[7]
            mag_msg.magnetic_field.z = imu_data[8]
            
            if self.outMode == 0:
                imu_msg.orientation.x = imu_data[9]
                imu_msg.orientation.y = imu_data[10]
                imu_msg.orientation.z = imu_data[11]
                imu_msg.orientation.w = imu_data[12]
                
                imu_msg.orientation
                r,p,y = euler_from_quaternion(imu_msg.orientation)
                rpy_msg.x = r
                rpy_msg.y = p
                rpy_msg.z = y
            elif self.outMode == 1:
                quaternion = Quaternion()
                rpy_msg.x = imu_data[9]
                rpy_msg.y = imu_data[10]
                rpy_msg.z = imu_data[11]
                quaternion.x, quaternion.y, quaternion.z, quaternion.w = quaternion_from_euler(rpy_msg.x, rpy_msg.y, rpy_msg.z)
  

                imu_msg.orientation = quaternion
            elif self.outMode == 2:
                matrix = numpy.matrix( [[imu_data[9],imu_data[10],imu_data[11],0],
                            [imu_data[12],imu_data[13],imu_data[14],0],
                            [imu_data[15],imu_data[16],imu_data[17],0],
                            [0,0,0,1]])
                # matrix =    [[imu_data[9],imu_data[10],imu_data[11],0],
                #             [imu_data[12],imu_data[13],imu_data[14],0],
                #             [imu_data[15],imu_data[16],imu_data[17],0],
                #             [0,0,0,1]]
                m = XsensDriver.NED.dot(matrix.transpose())
                # print(m)
                quaternion = Quaternion()   
                q_turn = quaternion_from_euler(0,0,math.pi/2)
                quaternion.x, quaternion.y, quaternion.z, quaternion.w  = quaternion_from_matrix(m)
                quaternion.x, quaternion.y, quaternion.z, quaternion.w  = quaternion_multiply(q_turn,[quaternion.x, quaternion.y, quaternion.z, quaternion.w ])
                # quaternion.x, quaternion.y, quaternion.z, quaternion.w = quaternion_from_euler(r, p,(0 - y)+ math.pi/3)
                r,p,y = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

                imu_msg.orientation = quaternion
                rpy_msg.x = r * (180/math.pi)
                rpy_msg.y = p* (180/math.pi)
                rpy_msg.z = y * (180/math.pi)
            else:
                self.get_logger().error('Select a correct output Mode (0, 1, 2)')

           
            self.hist_imu.append(imu_msg)
            if len(self.hist_imu) > 20:
                self.hist_imu.popleft()
                
            if len(self.hist_imu) == 20:
               
                matrix = numpy.zeros((20, 9))

                for i, imu_msg in enumerate(self.hist_imu):
                   
                    r,p,y = euler_from_quaternion([imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w])

                    matrix[i, 0] = r
                    matrix[i, 1] = p
                    matrix[i, 2] = y

                    matrix[i, 3] = imu_msg.angular_velocity.x
                    matrix[i, 4] = imu_msg.angular_velocity.y
                    matrix[i, 5] = imu_msg.angular_velocity.z

                    matrix[i, 6] = imu_msg.linear_acceleration.x
                    matrix[i, 7] = imu_msg.linear_acceleration.y
                    matrix[i, 8] = imu_msg.linear_acceleration.z

                mean = numpy.mean(matrix, axis=0)
                # print(matrix)
                for j in range(9):
                    self.var[j] = numpy.sum((matrix[:, j] - mean[j]) ** 2)
                for j in range(9):
                    self.var[j] = self.var[j]/mean[j]
                imu_msg.orientation_covariance[0] = 0.5 *(math.pi/180)
                imu_msg.orientation_covariance[4] = 0.5*(math.pi/180)
                imu_msg.orientation_covariance[8] = 0.5*(math.pi/180)

                imu_msg.angular_velocity_covariance[0] = 0.005
                imu_msg.angular_velocity_covariance[4] = 0.005
                imu_msg.angular_velocity_covariance[8] = 0.005
                
                imu_msg.linear_acceleration_covariance[0] = 0.008
                imu_msg.linear_acceleration_covariance[4] = 0.008
                imu_msg.linear_acceleration_covariance[8] = 0.008
                # print(self.var)
                self.imu_pub.publish(imu_msg)
                self.mag_pub.publish(mag_msg)
                self.rpy_pub.publish(rpy_msg)
            
            
        
    def data_decode(self,data):
        decode = []
        for i in range(0, len(data), 4):
            valor = struct.unpack('!f', data[i:i+4])[0]  # Decodificamos como float
            decode.append(valor)
        return decode
        
    def read_msg(self):
        start = time.time()
        while (time.time()-start)<self.timeout:
            new_start = time.time()
            response = self.serial.read()
            while (response == 0):
                response = self.serial.read()
                
            if ord(response)!=0xFA :
                continue
            self.waitfor(3,new_start)
            response = self.serial.read()
            if ord(response)!=0xFF :
                continue
            self.waitfor(3,new_start)
            response = self.serial.read()
            if ord(response)!=0x32 :
                continue
            self.waitfor(3,new_start)
            response = self.serial.read()
            if ord(response)!=0x4A :
                continue
            if self.outMode == 0:
                length = 52 + 3
            elif self.outMode == 1:
                length = 48 + 3
            elif self.outMode == 2:
                length = 72 + 3
            else:
                self.get_logger().error('Select a correct output Mode (0, 1, 2)')
                return
            response = self.serial.read(length)
            
            checksum = ord(chr((response[-1])))
            data = struct.unpack('!%dB'%(length-1), response[:-1])
            
            if self.verbose:
                print ("MT: data bytes: [%s]"%(' '.join("%i"% v for v in response)))
                self.get_logger().info('%s'%(str(data)))
                self.get_logger().info('%s'%(checksum))
                
            if 0xFF&sum(data, 0xFF+0x32+0x4A+checksum):
                self.get_logger().error("invalid checksum; discarding data and waiting for next message.\n")
                continue    
            return response[:-3]
            
    def check_ack(self,mid_ack):
        start = time.time()
        while (time.time()-start)<self.timeout:
            new_start = time.time()
            response = self.serial.read()

            while (response == 0):
                response = self.serial.read()
            if ord(response) != 0XFA:
                continue
            self.waitfor(3,new_start)
            response = self.serial.read()
            if ord(response) != 0xFF:
                continue
            response = self.serial.read(3)
            mid = ord(chr(response[0]))
            length = ord(chr(response[1]))
            cs = ord(chr(response[2]))
            if 0x01&(0xFF+mid+length+cs):
                continue
            if mid == mid_ack + 1:
                return True
   
    def waitfor(self,size=1,new_start=0):
                while self.serial.in_waiting < size:
                    if time.time()-new_start >= self.timeout:
                        self.get_logger("waitfor").warn('timeout waiting for message')

    def write_msg(self, mid, data=[]):
        length = len(data)
        preamble = 0x01
        if length>254:
            lendat = [0xFF, 0xFF&length, 0xFF&(length>>8)]
        else:
            lendat = [length]
        if mid == MID.GoToConfig:
            preamble = 0xFF
        packet = [0xFA, preamble, mid] + lendat + list(data)
        packet.append(0xFF&(-(sum(packet[1:]))))
        msg = struct.pack('%dB'%len(packet), *packet)
        start = time.time()
        while (time.time()-start)<self.timeout and self.serial.read():
            pass
        self.serial.write(msg)

    def GoToConfig(self):
        self.write_msg(MID.GoToConfig)
        return self.check_ack(MID.GoToConfig)

    def Configure(self):
        self.write_msg(MID.SetOutputMode,[0,0,0,6])
        if self.check_ack(MID.SetOutputMode):
            data = []
            if self.outMode == 0:
                data = [0,0,0,0,0,0,0,0]
            elif self.outMode == 1:
                data = [0,0,0,0,0,0,0,4]
            elif self.outMode == 2:
                data = [0,0,0,0,0,0,0,9]
            else:
                self.get_logger().error('Select a correct output Mode (0, 1, 2)')
                return
            self.write_msg(MID.SetOutputSettings,data)
            return self.check_ack(MID.SetOutputSettings)
        
    def GoToMeasure(self):
        self.write_msg(MID.GoToMeasurement)
        return self.check_ack(MID.GoToMeasurement)
        
def main(args=None):
    rclpy.init(args=args)
    driver = XsensDriver()
    while rclpy.ok():
        driver.spin_once()
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()