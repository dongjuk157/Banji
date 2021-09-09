## IoT제어 프로젝트

* 시뮬레이터 및 프로젝트 관련 파일 다운로드
  - https://drive.google.com/drive/folders/1rp54qL31ZIoHet7A9BlvpoDCCdGVsvLK?usp=sharing

(위 경로에 위치한 프로그램 및 문서는 SSAFY 과정 내에서만 사용할 수 있으며 무단 복제 및 반출, 배포를 금합니다.)

<br>

# Sub2

## 스켈레톤 프로젝트 실행 예시

### 1. 맵 기반 절대경로 생성 및 경로 추종(Req 1)

#### 1. 맵 읽어오기(Req 1-1)

- ros2 run sub2 load_map

- rviz2 - Add - By topic /map

  ![image-20210906100139513](README.assets/image-20210906100139513.png)

#### 2. 최단경로 탐색 및 경로 추종(Req 1-2)

- ros2 run sub2 odom

- ros2 run sub2 load_map

- ros2 run sub2 a_star

- rviz2 - Fixed Fram : map - 2D Goal Pose를 누르고 맵의 아무 곳을 찍는다.

  ![image-20210906101317078](README.assets/image-20210906101317078.png)

  - goal pose 메시지가 publish 되면서 콘솔창에 메시지가 출력된다.

  ![image-20210906101347185](README.assets/image-20210906101347185.png)

  - rviz2에 출력되는 모습

  ![image-20210906101708847](README.assets/image-20210906101708847.png)

### 2. 인지 프로젝트(Req 2)

#### 1. Extrinsic Calibration(Req 2-1)

- 시뮬레이터 내 카메라와 라이다를 세팅한 후 노드 실행

- ros2 run sub2 ex_calib

  - 초기 실행 모습
  - ros 메시지의 테이터가 callback 함수를 통해 처리되지 않은 상태라서 wating for msg라는 형태로 나온다.

  ![image-20210906102223452](README.assets/image-20210906102223452.png)

#### 사람인지(Req 2-2)

- 시뮬레이터 내 카메라를 세팅한 후 노드 실행

- ros2 run sub2 human_detector

  - 수신한 카메라의 원본을 보여주는 창이 생성된다.

  ![image-20210906102502462](README.assets/image-20210906102502462.png)

#### 3. 소지품 인지(Req 2-3)

- 시뮬레이터 내 카메라 세팅에서 ground truth 항목에 semantic을 선택하고 connect 한 후 노드 실행

- ros2 run sub2 seq_binarizer

  - 초기 실행 모습으로 시뮬레이터의 카메라 세팅이 Ground Truth : Semantic일 때 rgb 이미지가 아닌 segmented 이미지를 보여주는 창이 생성된다.

  ![image-20210906103843655](README.assets/image-20210906103843655.png)

<br>