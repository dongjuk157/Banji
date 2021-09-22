# SUB PJT 3 

알고리즘 고도화 및 스마트홈 서비스 완성

## 환경

1. ROS

   - ROS eloquent - 20200124 release
   - python 3.7.5
   - openssl 1.0.2u
   - choco 0.10.15
   - opencv 3.4.6
   - rti 5.3.1
   - opensplice 6.9.190403

2. tensorflow

   - tnesorflow 1.15
   - CUDA Toolkit 10.0
   - cuDNN 7.6.4

   

## 목표

1. 사용자 인터페이스 구현

2. socket.io를 활용한 서버/클라이언트 알고리즘

3. 딥뉴럴 네트워크를 이용한 object인식

4. UDP 통신을 이용한 Advanced IoT 제어

5. 예제 프로젝트

   - Custom Object를 옮기는 Multiplay Mini-Game Project

   - 스마트홈 경비 서비스

6. 심화 프로젝트



## 필수 지식 학습

### Tensorflow

TF는 뉴럴 네트워크의 구조와 입출력에 대한 Loss만 정의되면, 데이터를 가지고 자동 미분으로 back propagation을 수행할수 있음.

TensorRT를 사용해서 학습한 모델을 inference용으로 실제 적용할수 있음



**TF object Detection API**

Classfication, Localization이 합쳐진 문제 + bounding box

Faster R-CNN

- 속도 문제
- 해결하기 위한 모델
  - Single Shot Multi-box detector(SSD)
  - YOLO 시리즈
  - EfficientDet



Object Detection을 위해서 Feature extractor와 Pretrain 모델을 미리 만들어야함.

초보자나 단순히 응용하는 사람은 모델을 만드는게 어려우므로 TF object detection api 를 사용함



### Node.js

CommonJS

V8엔진

Asynchronous I/O

blocking/nonblocking I/O


### socket.io

### UDP 통신

TCP Transfer Control Protocol: 신뢰성 프로토콜

UDP User Datagram Protocol: 비신뢰성 프로토콜

-  비연결형 서비스: ip와 포트만 맞추면 통신가능
- 헤더의 checksum 필드를 통해 최소한의 오류만 검출
- 연속성이 중요한 서비스에서 사용



### Mapping, Localization

로봇의 위치와 센서들을 기반으로 맵을 그려내는 것

1) Landmark/Feature

   센서 데이터를 그대로 쓰지 않고 주변의 특징점을 가지고 맵을 만드는 방식

   카메라의 이미지를 기반으로한 descriptor 사용

   QR코드나 SURF, SIFT등 사용

   거리 정보까지 정확히 구하기 힘듦

2) Grid Map

   공간을 grid cell로 표현하며 물체가 존재하지 않는 영역과 존재하는 영역을 직접 cell에 표시하는 방식

   각 cell에 2D 라이다의 레이저가 지나간곳은 0 벽이나 물체가 있으면 1로 채움. 지나가지 않은 공간은 0.5로 작성

**Bresenham algorithm**

마킹된 현재 픽셀로부터 다음 픽셀을 어떻게 선택할지 결정하는 알고리즘

**Particle Filter**

정규분포가 아닌 노이즈나 기존 상태나 측정치 등에 대한 확률정보가 없을 때 사용

importance sampling을 통해서 정규분포가 아닌 분포에서 샘플링 가능

로봇의 이동을 예측하는 모델들은 비선형으로 정의된 경우가 많아서 위치추정에 많이 사용됨

Monte Carlo localization을 구현하는데 사용됨

**Prediction Model**

파티클 필터 기반 위치 추정시 사용되는 예측 모델

- odometry base model,  velocity based model

Thrun, FOx, Burgard의 particle motion model 사용

**Weighting**

**Resampling**



## 개발 및 결과

