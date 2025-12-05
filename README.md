# FLIR + Vectornav ROS2 Record
**Gemini 3**로 작성되었습니다.

두 센서를 동시에 실행하고, **타임스탬프를 동기화**(정확히는 동일한 시스템 시간 기준 기록)하여 하나의 rosbag 파일로 저장하는 과정을 안내해 드립니다.

## ⚠️ 개발환경
**HW**: FLIR Lepton 2.5 + Pure Termal 보드, Vectornav-100T

**환경**: Ubuntu 22.04 + ROS2 Humble

## 📂 통합 실행 패키지 만들기 (추천)
두 개의 센서를 매번 따로 실행(`ros2 run`, `ros2 launch`)하는 것은 번거롭고 시간 차이가 발생할 수 있습니다. 
하나의 런치 파일로 두 센서를 동시에 켜는 패키지를 만드는 것이 정석입니다.

**▶ 통합 워크스페이스 생성:**

기존 워크스페이스(`flir_ws`, `vn_ws`)를 합치거나 새로운 워크스페이스를 만듭니다. 여기서는 편의상 `sensor_ws`를 새로 만듭니다.
```bash
mkdir -p ~/sensor_ws/src
cd ~/sensor_ws/src

# 기존에 만들었던 FLIR 패키지와 VectorNav 패키지를 이곳으로 복사하거나 심볼릭 링크를 겁니다.
# (예시: 복사하는 경우)
cp -r ~/flir_ws/src/flir_lepton_ros .
cp -r ~/vn_ws/src/vectornav .
```

## 🛠️ 통합 런처 패키지 생성
`sensor_integration` 패키지 안에 두 센서를 동시에 실행하는 스크립트를 작성합니다.

**▶ launch 폴더 생성:**
```bash
mkdir -p ~/sensor_ws/src/sensor_integration/launch
cd ~/sensor_ws/src/sensor_integration/launch
code .
```

**▶** `combined_sensors.launch.py` **내용 작성:**

이 파일은 VectorNav의 런치 파일을 불러오고, FLIR 노드를 직접 실행하며, 두 센서 간의 좌표계(TF)를 연결해줍니다.
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. VectorNav 패키지 경로 및 런치 파일 설정
    vn_pkg = get_package_share_directory('vectornav')
    vn_launch_path = os.path.join(vn_pkg, 'launch', 'vectornav.launch.py')

    # 2. FLIR Lepton 노드 설정
    # (이전 단계에서 만든 flir_lepton_ros 패키지)
    lepton_node = Node(
        package='flir_lepton_ros',
        executable='lepton_node',
        name='lepton_camera',
        output='screen',
        parameters=[{'device_id': 0}] # /dev/video0 인 경우
    )

    # 3. VectorNav 런치 파일 포함
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vn_launch_path)
    )

    # 4. TF (Transform) 설정 (중요: 타임싱크 및 센서 퓨전용)
    # base_link(로봇 중심) -> vectornav_link (IMU)
    # base_link(로봇 중심) -> lepton_frame (카메라)
    # 예시: 카메라가 IMU 바로 위에 있다고 가정 (x, y, z, r, p, y)
    
    # IMU TF (VectorNav 드라이버가 보통 자체적으로 publish 하지만, 없다면 추가)
    # Camera TF
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.05', '0', '0', '0', '0', '0', 'vectornav_link', 'lepton_frame']
    )

    return LaunchDescription([
        vectornav_launch,
        lepton_node,
        tf_camera
    ])
```

**▶** `setup.py` **등록:**

`~/sensor_ws/src/sensor_integration/setup.py` 파일을 열어 `data_files` 부분에 launch 폴더를 등록해야 합니다.

```python
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 아래 줄 추가
        ('share/' + package_name + '/launch', ['launch/combined_sensors.launch.py']),
    ],
```

## 🤖 빌드(Build) & 실행확인
**▶ 빌드:**

```
cd ~/sensor_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install
```
**▶ 실행:**

이제 명령어 한 줄로 두 센서를 켭니다.
```bash
source ~/sensor_ws/install/setup.bash
ros2 launch sensor_integration combined_sensors.launch.py
```

**▶ 확인:**

다른 터미널에서 토픽이 둘 다 나오는지 확인합니다.

```bash
ros2 topic list
# /vectornav/imu
# /vectornav/magnetic
# /thermal/image_raw
# /thermal/image_color
# /tf
# ... 등이 보여야 함
```

## 💾 Rosbag 기록 (Time Sync 저장)
이제 하나의 Bag 파일에 두 데이터를 담습니다. ROS 2는 메시지가 발행될 때의 PC 시간(System Clock)을 기준으로 Bag 파일에 기록하므로, 나중에 재생할 때 두 데이터의 시간 축이 맞게 됩니다.

**▶ 기록 명령어:**

```bash
# 저장할 폴더로 이동
cd ~/Documents

# ros2 bag record [토픽명1] [토픽명2] ...
# -o 옵션으로 파일명 지정
ros2 bag record -o my_dataset_01 /vectornav/imu_uncompensated /vectornav/magnetic /thermal/image_raw /tf_static
```

## 💡 타임싱크(Time Synchronization)에 대한 조언
위 방식은 **"Software Synchronization (Approximate Time)"** 방식입니다.

- **원리:** USB 케이블을 타고 PC에 데이터가 도착한 순간의 시간을 기록합니다.

- **오차:** USB 통신 지연, OS 스케줄링 등으로 인해 수 밀리초(ms) 정도의 지터(Jitter)가 발생할 수 있습니다.

- **활용:** 일반적인 VIO 알고리즘(VINS-Mono 등)이나 딥러닝 학습용으로는 충분합니다.
