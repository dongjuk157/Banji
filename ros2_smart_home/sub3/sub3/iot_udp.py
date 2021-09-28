
import rclpy
from rclpy.node import Node  
import time
import os
import socket
import threading
import struct
import binascii

from struct import pack

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
    (0xa,0x25 ) : "IDLE" ,
    (0xb,0x31 ) : "CONNECTION",
    (0xc,0x51) : "CONNECTION_LOST" ,
    (0xb,0x37) : "ON",
    (0xa,0x70) : "OFF",
    (0xc,0x44) : "ERROR"
}

# 제어 명령 ( 문자열 -> 바이트 )
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
        # socket.AF_INET : 사용되는 주소 및 프로토콜 (패밀리)를 나타낸다
        # AF_INET : IPv4 주소체계를 사용하는 인터넷 망 접속

        # socket.SOCK_DGRAM : 소켓 유형을 나타낸다
        # DGRAM : UDP통신은 데이터그램으로 한다
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip,self.port)

        # bind => 호스트이름과 포트번호를 인자로 보낸다
        # 위의 두 정보를 보내 연결한다는 의미로 보면 될 것 같다
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[] # 수신된 IoT기기들을 모으는 부분?
        
        # 로직 2. 멀티스레드를 이용한 데이터 수신
        # target는 실행할 함수를 정하는 것 
        thread = threading.Thread(target=self.recv_udp_data)

        # daemon 스레드로 지정할 경우 메인 스레드가 종료되면 서브도 다 종료된다. 
        thread.daemon = True 

        # thread활동 시작
        thread.start() 

        self.is_recv_data=False

        os.system('cls')
        while True:
            print('Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures ] ')
            menu = int(input())

            if menu == 0:
                self.scan()
            elif menu == 1:
                self.connect()
            elif menu == 2:
                # thread?
                self.control()

            elif menu == 3:
                self.disconnect()
            elif menu == 4:
                self.all_procedures()
           
            '''
            로직 5. 사용자 메뉴 생성
            print('Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures ] ')
            menu=??

            if menu == ?? :
                채워 넣기
            
            '''

    # byte형태로 raw_data를 받기 때문에 decode() 메서드를 쓰면 바로 될줄 알았으나 
    # 되지 않는다. 그래서 16진수로 바꾸고 인덱스 슬라이싱을 통해 해결
    def data_parsing(self,raw_data) :

        # Header 19Byte
        header = raw_data[:19].decode()

        # Data Length => 4Byte
        data_length = raw_data[19:23]

        # raw_data는 인덱스 하나하나를 각 바이트로 인식, AUX_DATA : 예비 데이터 공간(12Byte)
        aux_data = raw_data[23:35]

        # 시뮬레이터 -> 유저 정보를 받았을 때 헤더와 데이터 길이 체크 
        if header == '#Appliances-Status$' and data_length[0] == 20:
            # (IoT 아이디 포함 네트워크, 해당 디바이스 상태 체크)
            uid_pack = raw_data[35:55] # 20Byte
            uid = self.packet_to_uid(uid_pack[:16]) # IoT 아이디 16자리(string)

            # hex()사용 시 16진수 형태로 문자열로 나타내진다
            network_status = (hex(uid_pack[16]), hex(uid_pack[17])) # 네트워크 상태(정상, 연결 상태, 연결 종료)
            device_status = (hex(uid_pack[18]), hex(uid_pack[19])) # 디바이스 상태(ON, OFF, ERROR)
            self.is_recv_data = True

            # 수신받은 패킷 내 데이터 부분 저장
            # 네트워크, 디바이스 상태는 튜플로 묶여져 있고, hexstr 형태이므로
            # 추후에 상태 확인 시 정수로 바꿔 비교해서 봐야한다
            self.recv_data = [uid, network_status, device_status]

            # scan on, off 구분 and 중복 방지 ( 완벽 scan은 아님 )
            if len(self.parsed_data) != 0:
                if self.recv_data[0] not in self.parsed_data:
                    self.parsed_data.append(self.recv_data)
                else:
                    for data in self.parsed_data:
                        if data[0] == self.recv_data[0]:
                            if data[2] == self.recv_data[2]:
                                continue
                            else:
                                self.recv_data[2] = data[2]
            else:
               self.parsed_data.append(self.recv_data) 


            # self.parsed_data = self.recv_data
        '''
        로직 3. 수신 데이터 파싱

        header=?
        data_length=?
        aux_data=?


        if header == ?? and data_length[0] == ??:
            uid_pack=??
            uid=self.packet_to_uid(uid_pack)
        
            network_status=??
            device_status=??
            
            self.is_recv_data=True
            self.recv_data=[uid,network_status,device_status]
        '''
 
    # 제어를 하는 통신 
    def send_data(self,uid,cmd):
        # byte형태로 보내야한다.
        # $표시가 아니라 #
        header = '#Ctrl-command$'.encode()
        # struct모듈 내 pack함수 이용하여 bytes로 변환한다
        # 참고로 to_bytes()에 비해서는 속도가 좀 더 빠르다
        # 통신 프로그램을 작성할 때, 필요한 구조체 형식의 데이터를 다루는데 아주 적합
        data_length = pack('i', 18) # 4 바이트(숫자 18을 4바이트인 int형으로 바이트 type으로 바꾼다)

        # 12bytes 0으로 된 것 모음 
        aux_data = bytes(12)

        # self.upper는 아래와 같이 패킷 내 세 부분을 합한 것 
        self.upper = header + data_length + aux_data

        # 정해진 0x0D0A 를 바이트로 변환해준다 
        self.tail = bytes([0x0D, 0x0A])

        # uid_pack => IoT 기기의 아이디 
        uid_pack = self.uid_to_packet(uid)

        # 제어 메세지 받은 거를 바이트로 변환해준다 
        cmd_pack = bytes([cmd[0],cmd[1]])

        # 구한 데이터들을 모아 하나의 패킷으로 만들어서 보낸다 
        send_data = self.upper + uid_pack + cmd_pack + self.tail

        self.sock.sendto(send_data,(self.ip,self.send_port))
        '''
        로직 4. 데이터 송신 함수 생성

 
        header=?
        data_length=?
        aux_data=?
        self.upper=?
        self.tail=?

        uid_pack=self.uid_to_packet(uid)
        cmd_pack=bytes([cmd[0],cmd[1]])

        send_data=self.upper+uid_pack+cmd_pack+self.tail
        self.sock.sendto(send_data,(self.ip,self.send_port))
        '''


    def recv_udp_data(self):
        while True :
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)
            

    def uid_to_packet(self,uid):
        # binascii 모듈에는 바이너리와 
        # 다양한 ASCII 인코딩 바이너리 표현 사이를 변환하는 여러 가지 방법이 포함되어 있다
        # unhexlify => 16진수 문자열 hexstr로 표현된 바이너리 데이터를 반환
        uid_pack=binascii.unhexlify(uid)
        return uid_pack

        
    def packet_to_uid(self,packet):
        uid=""
        for data in packet:
            if len(hex(data)[2:4])==1:
                uid+="0"
            
            uid+=hex(data)[2:4]
            
        return uid


    def scan(self):
        
        print('SCANNING NOW.....')
        print('BACK TO MENU : Ctrl+ C')
        '''
        로직 6. iot scan

        주변에 들어오는 iot 데이터(uid,network status, device status)를 출력하세요.

        '''
        try:
            while True:
                if len(self.parsed_data) != 0:
                    for data in self.parsed_data:
                        print('uid : ',  data[0], 
                            ' ,network status: ', params_status[(int(data[1][0], 16), int(data[1][1], 16))], 
                            ' ,device status : ', params_status[(int(data[2][0], 16), int(data[2][1], 16))]
                        )
                time.sleep(1)
                    # 16진수가 문자열 형태로 되어있기 때문에 이를 정수로 바꾸어서 상태 파라미터 값이 맞는지 확인
        except KeyboardInterrupt:
            pass

        

    # 한 개씩 키고 끄는 작업을 실행해야 함 .
    # 각각 디바이스의 ID를 가지고 있고, ON 버튼 눌렀을 때 해당 아이디를 찾아가야 할듯 (추후 웹과 연결 시)
    def connect(self):
        # network 상태가 CONNECTION_LOST 상태 시 RESET 시킴 
        # TRY_TO_CONNECT 명령을 보내 접속해줌
        if self.is_recv_data:
            if params_status[(int(self.recv_data[1][0], 16), int(self.recv_data[1][1], 16))] == "CONNECTION_LOST":
                while params_status[(int(self.recv_data[1][0], 16), int(self.recv_data[1][1], 16))] == "CONNECTION_LOST":
                    self.send_data(self.recv_data[0], params_control_cmd["RESET"])
            else:
                while params_status[(int(self.recv_data[1][0], 16), int(self.recv_data[1][1], 16))] == "IDLE":
                    self.send_data(self.recv_data[0], params_control_cmd["TRY_TO_CONNECT"])
        else:
            print('연결된 디바이스가 없습니다. 디바이스 근처로 가주세요')

        '''
        로직 7. iot connect

        iot 네트워크 상태를 확인하고, CONNECTION_LOST 상태이면, RESET 명령을 보내고,
        나머지 상태일 때는 TRY_TO_CONNECT 명령을 보내서 iot에 접속하세요.

        '''

    
    def control(self):
        # 한번에 켜지거나 안꺼질 때 thread이용해보라? => 비슷함... 
        if self.is_recv_data:
            if params_status[(int(self.recv_data[1][0], 16), int(self.recv_data[1][1], 16))] == "CONNECTION":
                # 켜져있으면 끄고
                if params_status[(int(self.recv_data[2][0], 16), int(self.recv_data[2][1], 16))] == "ON":
                    while params_status[(int(self.recv_data[2][0], 16), int(self.recv_data[2][1], 16))] == "ON":
                        self.send_data(self.recv_data[0], params_control_cmd["SWITCH_OFF"])
                # 꺼져있으면 킨다.
                else:
                    while params_status[(int(self.recv_data[2][0], 16), int(self.recv_data[2][1], 16))] == "OFF":
                        self.send_data(self.recv_data[0], params_control_cmd["SWITCH_ON"])
        '''
        로직 8. iot control
        
        iot 디바이스 상태를 확인하고, ON 상태이면 OFF 명령을 보내고, OFF 상태면 ON 명령을 보내서,
        현재 상태를 토글시켜주세요.
        '''


    # 연결 해제
    def disconnect(self):
        if self.is_recv_data==True :
            while params_status[(int(self.recv_data[1][0], 16), int(self.recv_data[1][1], 16))] == "CONNECTION":
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