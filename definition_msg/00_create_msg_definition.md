# 메세지 정의 만들기

## 메세지 정의를 위한 별도의 패키지 만들어주기
<br>

**메세지 정의를 위한 별도의 패키지를 따로 만드는 이유**

- **메세지를 정의하기 위해선 CMakeLists.txt 파일이 필요**
    
    현재 나는 ament_python 타입으로 패키지를 생성했다. 이 타입은 파이썬 노드 실행을 위한 구조로 만들어지기 때문에 메세지 정의를 만들기 위해 필요한 CMakeLists.txt 파일이 존재하지 않는다.<br>
    -> 즉, **CMakeLists.txt 파일이 존재하는 ament_cmake 타입으로 패키지를 따로 만들어야 한다**.
    
    (빌드 타입을 ament_python 으로 하고 직접 CMakeLists.txt와 일부 파일 내용을 수정해서 추가하는 방법이 있긴 하지만 까다롭다)
    <br><br>

- **따로 구성해두면 불필요한 의존성 제거 가능**
    
    원격 로봇 패키지가 있다고 할 때, 대부분의 패키지들은 로봇 HW 제어나 SLAM과 같은 무거운 기능이 포함되어 있을 거다. 이때 로봇의 상태를 시각화하는 패키지를 추가로 만든다면 HW, SLAM 기능은 필요하지 않다. 시각화에 필요한 것은 로봇에서 오는 데이터 구조(메세지 정의) 뿐이므로, 메세지 정의만 가져와 별도 패키지로 구성하면 된다.
    
    즉 메세지 정의 패키지를 따로 구성해 두면 편하게 작업이 가능하다.
<br><br>

**패키지 만들기**

```bash
ros2 pkg create --build-type ament_cmake my_first_package_msgs
```
<br>

- **tree 명령어로 확인해보기**
    
    ```python
    .
    ├── my_first_package
    │   ├── my_first_package
    │   │   ├── __init__.py
    │   │   ├── my_first_node.py
    │   │   ├── my_publisher.py
    │   │   └── my_subscriber.py
    │   ├── package.xml
    │   ├── resource
    │   │   └── my_first_package
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── test
    │       ├── test_copyright.py
    │       ├── test_flake8.py
    │       └── test_pep257.py
    └── my_first_package_msgs
        ├── CMakeLists.txt
        ├── include
        │   └── my_first_package_msgs
        ├── package.xml
        └── src
    ```
    
    - ament_python 타입인 my_first_package는 CMakeLists.txt가 없고, ament_cmake 타입인 my_first_package_msgs는 CMakeLists.txt는 있지만 setup.py가 없다.
<br><br>

## 메세지 정의 만들기

**메세지 정의 폴더 구조**

```bash
my_first_package_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
		├── msg1.msg
		├── msg2.msg
    └── msg3.msg

```

- msg/ 폴더 안에 메세지 정의 파일들을 위치시킨다.
<br><br>

**메세지 정의 만들기** 

```bash
float32  cmd_vel_linear
float32  cmd_vel_angular

float32  pose_x
float32  pose_y
float32  linear_vel
float32  angular_vel
```
<br>

**패키지 빌드하기**

**1. CMakeLists.txt 파일에 아래 내용 추가해주기**
    
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CmdAndPoseVel.msg"
)
```

msg 폴더 안의 CmdAndPoseVel.msg 파일을 찾아 메세지로 등록해서 빌드하라는 설정을 추가하는 거다.<br><br>
    
**2. package.xml 파일에 아래 내용 추가해주기**
    
```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
<br>
    
**3. 재빌드 & 설정 파일 불러오기**
    
```
colcon build
source install/setup.bash
```
<br>    

**4. 확인해보기**

- **메세지 정의 확인하기**
    
    ```
    ros2 interface show my_first_package_msgs/msg/CmdAndPosVel
    ```
    
    ```
    # Output
    
    float32 cmd_vel_linear
    float32 cmd_vel_angular
    
    float32 pose_x
    float32 pose_y
    float32 linear_vel
    float32 angular_vel
    ```
<br>

- **패키지 메세지 리스트 확인하기**
    
    ```
    ros2 interface list | grep my_first_package_msgs
    ```
    
    ```
    # Output
    
    my_first_package_msgs/msg/CmdAndPoseVel
    ```