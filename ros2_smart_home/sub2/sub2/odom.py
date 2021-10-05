import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi,cos,sin
import tf2_ros
import geometry_msgs.msg
import time

# odom 노드는 로봇의 상태메세지(/turtlebot_status)의 절대위치로 로봇의 위치를 추정하는 노드입니다.
# Imu센서(/imu) 메시지를 받아서 로봇의 위치를 추정

# 노드 로직 순서
# 1. publisher, subscriber, broadcaster 만들기
# 2. publish, broadcast 할 메시지 설정
# 3. 로봇 위치 추정
# 4. 추정한 로봇 위치를 메시지에 담아 publish, broadcast

class odom(Node):

    def __init__(self):
        super().__init__('odom')
        
        # 로직 1. publisher, subscriber, broadcaster 만들기
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 로봇의 pose를 저장해 publish 할 메시지 변수 입니다.
        self.odom_msg=Odometry()
        # 각 좌표계? 들의 관계 transform
        # Map -> base_link 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.base_link_transform=geometry_msgs.msg.TransformStamped()
        # base_link -> laser 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.laser_transform=geometry_msgs.msg.TransformStamped()
        self.is_status=False
        self.is_imu=False
        self.is_calc_theta=False
        # x,y,theta는 추정한 로봇의 위치를 저장할 변수 입니다.        
        # 로봇의 초기위치를 맵 상에서 로봇의 위치와 맞춰줘야 합니다.
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        # imu_offset은 초기 로봇의 orientation을 저장할 변수 입니다.
        self.imu_offset=0
        self.prev_time=0

        
        '''
        로직 2. publish, broadcast 할 메시지 설정
        self.odom_msg.header.frame_id=
        self.odom_msg.child_frame_id=

        self.base_link_transform.header.frame_id = 
        self.base_link_transform.child_frame_id = 

        self.laser_transform.header.frame_id = 
        self.laser_transform.child_frame_id =      
        self.laser_transform.transform.translation.x = 
        self.laser_transform.transform.translation.y = 
        self.laser_transform.transform.translation.z = 
        self.laser_transform.transform.rotation.w = 
        '''
        # transform(tf) : 이동(translation), 회전(rotation)에 대한 값들을 넣어 계산할 수 있게한다.
        # odom
        # frame_id => 'map'좌표계 위에 odometry를 표시하기 때문에 
        # frame_id는 'map', child_frame_id는 로봇을 나타내는 frame인 'base_link'로 표시

        # base_link_transform
        # frame_id => 'map'좌표계 위에 odometry를 표시하기 때문에 
        # frame_id는 'map', child_frame_id는 로봇을 나타내는 frame인 'base_link'로 표시

        # laser_transform
        # frame_id => 'base_link' 좌표계 위에 라이다센서 있음 
        # child_frame_id = 'laser', 라이다를 나타내는 frame을 넣음

        # 정리해볼 때 header frame_id는 바탕? 베이스를 뜻하는 frame이 들어가고
        # child_frame_id는 해당 좌표계를 넣는다 
        # 음... 그니까 child가 본체고 header가 이제 어디에 있는지를 의미하는 것 같음
        # 예를 들어? 책상이 header frame_id이고, 노트북이 child_frame
        self.odom_msg.header.frame_id = 'map'
        self.odom_msg.child_frame_id = 'base_link'

        self.base_link_transform.header.frame_id = 'map'
        self.base_link_transform.child_frame_id = 'base_link'

        # laser_transform은 'base_link'에 달려있는 라이다를 의미
        # translation은 base_link로부터 얼마나 떨어져 이동되어 있는지 의미
        # base_link와 laser간 translation(이동), rotation은 변하지 않아서 초기에 설정한다
        self.laser_transform.header.frame_id = 'base_link'
        self.laser_transform.child_frame_id = 'laser'

        # 로봇에 달려있는 라이다가 똑같은 곳에 고정되어 있어서 미리 값을 줘서 설정
        # 터틀봇으로부터 1m 위에 달려있음을 의미 (좌표계)
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 1.0
        self.laser_transform.transform.rotation.w = 1.0
    def imu_callback(self,msg):

        '''
        로직. IMU 에서 받은 quaternion을 euler angle로 변환해서 사용
        '''
        # 회전 각을 위해 IMU사용을 한다 

        if self.is_imu ==False :    
            self.is_imu=True
            imu_q= Quaternion.to_euler(msg.orientation)
            # imu_q는 오일러각(x, y, z)
            # imu_offset은 orientation을 의미하는 것으로 보이므로 z값을 넣는다. 방향
            self.imu_offset=imu_q[2]
        else :
            imu_q= Quaternion.to_euler(msg.orientation)
            self.theta=imu_q[2]

    def listener_callback(self, msg):
        # print('linear_vel : {}  angular_vel : {}'.format(msg.twist.linear.x,-msg.twist.angular.z))        
        if self.is_imu ==True:
            if self.is_status == False :
                self.is_status=True
                self.prev_time=rclpy.clock.Clock().now()
                # 로봇의 초기 위치 지정(twist=> 하나의 토픽. 데이터 형태임. 일종의 자료형)
                self.x = msg.twist.angular.x
                self.y = msg.twist.angular.y
            else :
                self.current_time=rclpy.clock.Clock().now()
                # 계산 주기를 저장한 변수 입니다. 단위는 초(s)
                self.period=(self.current_time-self.prev_time).nanoseconds/1000000000
                # 로봇의 선속도, 각속도를 저장하는 변수, 시뮬레이터에서 주는 각 속도는 방향이 반대이므로 (-)를 붙여줍니다.
                linear_x=msg.twist.linear.x
                angular_z=-msg.twist.angular.z
                '''
                로직 3. 로봇 위치 추정
                (테스트) linear_x = 1, self.theta = 1.5707(rad), self.period = 1 일 때
                self.x=0, self.y=1 이 나와야 합니다. 로봇의 헤딩이 90도 돌아가 있는
                상태에서 선속도를 가진다는 것은 x축방향이 아니라 y축방향으로 이동한다는 뜻입니다. 
                #절대위치 사용
                self.x+=
                self.y+=
                self.theta+=
                '''
                # self.x=msg.twist.angular.x  
                # self.y=msg.twist.angular.y
                # self.theta=(msg.twist.linear.z)*pi/180
                # 절대위치 사용(좌표가 변하지 않는다.(맵을 불러왔기 때문))
                self.x += linear_x * cos(self.theta) * self.period
                self.y += linear_x * sin(self.theta) * self.period
                self.theta += angular_z * self.period
                self.base_link_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                self.laser_transform.header.stamp =rclpy.clock.Clock().now().to_msg()
                
                '''
                로직 4. 추정한 로봇 위치를 메시지에 담아 publish, broadcast
                '''
                q = Quaternion.from_euler(0, 0, self.theta)
                
                self.base_link_transform.transform.translation.x = self.x
                self.base_link_transform.transform.translation.y = self.y
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                self.base_link_transform.transform.rotation.w = q.w
                
                self.odom_msg.pose.pose.position.x=self.x
                self.odom_msg.pose.pose.position.y=self.y
                self.odom_msg.pose.pose.orientation.x=q.x
                self.odom_msg.pose.pose.orientation.y=q.y
                self.odom_msg.pose.pose.orientation.z=q.z
                self.odom_msg.pose.pose.orientation.w=q.w
                self.odom_msg.twist.twist.linear.x=linear_x
                self.odom_msg.twist.twist.angular.z=angular_z

            
                # 추정한 로봇의 위치(transform)를 publish한다
                # broadcaster는 좌표계를 만들어주는 역할, 
                
                self.broadcaster.sendTransform(self.base_link_transform)
                self.broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)
                self.prev_time=self.current_time

        
def main(args=None):
    rclpy.init(args=args)

    sub2_odom = odom()

    rclpy.spin(sub2_odom)


    sub2_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




       
   
