import rclpy
from rclpy.node import Node  
import time
import os
import socket
import threading
import struct
import binascii
from collections import deque

# iot_udp 노드는 udp 통신을 이용해 iot로 부터 uid를 얻어 접속, 제어를 하는 노드입니다.
# sub1,2 에서는 ros 메시지를 이용해 쉽게 제어했지만, advanced iot 제어에서는 정의된 통신프로토콜을 보고 iot와 직접 데이터를 주고 받는 형식으로 제어하게 됩니다.
# 통신 프로토콜은 명세서를 참조해주세요.


# 노드 로직 순서
# 1. 통신 소켓 생성
# 2. 멀티스레드를 이용한 데이터 수신
# 3. 수신 데이터 파싱
# 4. 데이터 송신 함수
# 5. 사용자 메뉴 생성 
# 6. iot scan 
# 7. iot connect
# 8. iot control

# 통신프로토콜에 필요한 데이터입니다. 명세서에 제어, 상태 프로토콜을 참조하세요. 
params_status = {
    (0xa,0x25) : "IDLE" ,
    (0xb,0x31) : "CONNECTION",
    (0xc,0x51) : "CONNECTION_LOST" ,
    (0xb,0x37) : "ON",
    (0xa,0x70) : "OFF",
    (0xc,0x44) : "ERROR"
}


params_control_cmd= {
    "TRY_TO_CONNECT" : (0xb,0x31 )  ,
    "SWITCH_ON" : (0xb,0x37 ) ,
    "SWITCH_OFF" : (0xa,0x70),
    "RESET" : (0xb,0x25) ,
    "DISCONNECT" : (0x00,0x25) 
}


class iot_udp(Node):

    def __init__(self):
        super().__init__('iot_udp')

        self.ip='127.0.0.1'
        self.port=7502
        self.send_port=7401

        # 로직 1. 통신 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip,self.port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        self.parsed_data_deque = deque(maxlen=50)
        
        # 로직 2. 멀티스레드를 이용한 데이터 수신
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

        self.is_recv_data=False
        self.scaned_data=[]

        while True:
            os.system('cls')
            '''
            로직 5. 사용자 메뉴 생성
            '''
            print('Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures] ')
            menu = int(input())

            if menu == 0:
                self.scan()
            elif menu == 1:
                self.connect()
            elif menu == 2:
                self.control()
            elif menu == 3:
                self.disconnect()
            elif menu == 4:
                self.all_procedures()

            os.system('pause')

    def data_parsing(self, raw_data) :
        # print(raw_data) # len(raw_data) = 57 byte
        #b'#Appliances-Status$\x14\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00g\x0bk\xbaF\x8dC\x14\xb6\xe9\xfar\x05Il<\n%\np\r\n'
        
        '''
        로직 3. 수신 데이터 파싱
        '''
        header = raw_data[:19].decode() # 14 byte, #Appliances-Status$
        data_length = raw_data[19:23] # 4 byte, \x14\x00\x00\x00
        aux_data = raw_data[23:35] # 12 byte, \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00
        # data_length[0]: int형식(4 byte)으로 끊어서 확인
        if header == "#Appliances-Status$" and data_length[0] == 20:
            uid_pack = raw_data[35:51] # 16 byte
            uid = self.packet_to_uid(uid_pack) # 16 byte

            # params_status와 맞추기 위해 1byte씩 사용, status = (number, number) 이므로 int로 변환
            network_status = (int(raw_data[51]), int(raw_data[52])) # 2 byte
            device_status = (int(raw_data[53]), int(raw_data[54])) # 2 byte
            
            self.is_recv_data = True
            self.recv_data = [uid, network_status, device_status]
            self.parsed_data_deque.append(tuple(self.recv_data))
        
 
    def send_data(self, uid, cmd):
        '''
        로직 4. 데이터 송신 함수 생성
        '''
 
        header = '#Ctrl-command$'.encode()
        # 같은 의미. data_length = (18).to_bytes(4, byteorder="little")
        data_length = struct.pack('i', 18)
        aux_data = bytes(12)
        self.upper = header + data_length + aux_data
        self.tail = bytes([0x0D, 0x0A])

        uid_pack = self.uid_to_packet(uid)
        cmd_pack = bytes([cmd[0], cmd[1]])

        send_data = self.upper + uid_pack + cmd_pack + self.tail
        # print(send_data, len(send_data))
        self.sock.sendto(send_data, (self.ip, self.send_port))


    def recv_udp_data(self):
        while True :
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)


            
    def uid_to_packet(self,uid):
        uid_pack=binascii.unhexlify(uid)
        return uid_pack

        
    def packet_to_uid(self,packet):
        # packet: b'g\x0bk\xbaF\x8dC\x14\xb6\xe9\xfar\x05Il<\n%\np'
        uid=""
        for data in packet:
            # data는 1byte(\x__ 이거나 g, k 같은 char) 씩 판별
            if len(hex(data)[2:4])==1: # ascii code가 있는 경우 char로 변환되므로 앞에 0 추가
                uid+="0"
            
            uid+=hex(data)[2:4]
            
            
        return uid


    def scan(self):
        
        print('SCANNING NOW.....')
        print('BACK TO MENU : Ctrl+ C')
        '''
        로직 6. iot scan

        주변에 들어오는 iot 데이터(uid,network status, device status)를 출력하세요.

        문제점: 수신되는 데이터가 있어야만 queue가 갱신됨
        수신되는 데이터가 없으면 마지막 상태 저장
        '''
        try:
            tmp = 0
            prev_data = set()
            while True:
                cur_data = set(self.parsed_data_deque)
                if prev_data == cur_data:
                    continue
                
                for data in cur_data:
                    print("uid: {}\tnetwork status: {}\tdevice status: {}"
                        .format(
                            data[0],
                            params_status[data[1]],
                            params_status[data[2]]
                        )
                    )
                prev_data = cur_data
        except KeyboardInterrupt:
            pass
            


    def connect(self):
        
        '''
        로직 7. iot connect

        iot 네트워크 상태를 확인하고, CONNECTION_LOST 상태이면, RESET 명령을 보내고,
        나머지 상태일 때는 TRY_TO_CONNECT 명령을 보내서 iot에 접속하세요.

        '''
        if self.is_recv_data:
            while params_status[self.recv_data[1]] != 'CONNECTION':
                if params_status[self.recv_data[1]] == "CONNECTION_LOST":
                    self.send_data(self.recv_data[0], params_control_cmd["RESET"])
                else:
                    self.send_data(self.recv_data[0], params_control_cmd["TRY_TO_CONNECT"])

        

    
    def control(self):
        '''
        로직 8. iot control
        
        iot 디바이스 상태를 확인하고, ON 상태이면 OFF 명령을 보내고, OFF 상태면 ON 명령을 보내서,
        현재 상태를 토글시켜주세요.
        '''
        if self.is_recv_data:
            if params_status[self.recv_data[1]] != 'CONNECTION': 
                return
            cur_status = params_status[self.recv_data[2]]
            if cur_status == "ON":
                while params_status[self.recv_data[2]] != 'OFF':
                    self.send_data(self.recv_data[0], params_control_cmd["SWITCH_OFF"])
            elif cur_status == "OFF":
                while params_status[self.recv_data[2]] != 'ON':
                    self.send_data(self.recv_data[0], params_control_cmd["SWITCH_ON"])
                

    def disconnect(self):
        if self.is_recv_data:
            while params_status[self.recv_data[1]] != 'IDLE':
                self.send_data(self.recv_data[0],params_control_cmd["DISCONNECT"])
        

    def all_procedures(self):
        self.connect()
        time.sleep(0.5)
        self.control()
        time.sleep(0.5)
        self.disconnect()


           
    def __del__(self):
        self.sock.close()
        print('del')



def main(args=None):
    rclpy.init(args=args)
    iot = iot_udp()
    rclpy.spin(iot)
    iot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
