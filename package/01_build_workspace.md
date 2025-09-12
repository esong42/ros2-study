# [Package] 생성한 패키지 빌드하기

## package 빌드하기

**colcon 이란**<br>
ros2의 여러 패키지를 한 번에 빌드, 설치, 관리해주는 빌드 툴이다.<br>
ros1의 catkin_make 역할을 대신함<br><br>

**colcon build 명령어 사용해서 빌드해보기**

```bash
colcon build
```

위의 명령어를 입력하면:

1. workspace 안의 src/ 폴더를 자동으로 찾아 패키지 빌드
2. 빌드 과정을 저장할 폴더가 생성되고 내용이 저장됨<br>
    | **폴더** | 역할 |
    | --- | --- |
    | **build/** | 빌드 설정 파일 저장 |
    | **install/** | 실행 파일 저장 |
    | **log/** | 빌드할 때 생성된 log 저장 |
<br>

**colcon 주요 명령어**

- **`colcon build`**: src 내의 전체 패키지 빌드
- **`colcon build --package-select <pkg-name>`**: 특정 패키지만 빌드
- **`colcon build --base-paths <pkg-path>`**: src가 아닌 패키지 경로 지정
- **`colcon build --symlink-install`**: 코드 수정 없이 바로 반영
    - 빌드할 때 옵션을 주어 빌드를 하면 심볼릭 링크 구조가 만들어지기 때문에 install 폴더의 파일이 src 폴더 파일을 가리키는 심볼릭 링크가 됨
    - 이 구조가 만들어지면 이후에는 src 파일을 수정할 때마다 바로 install에 반영이 되기 때문에 재빌드(colcon build)할 필요가 없음
- **`colcon list`**: src 내의 패키지 목록 확인
- **`colcon test`**: 빌드 후 테스트 실행
<br><br>

## 빌드한 패키지 실행해보기

**1. 쉘 환경에 패키지 경로 등록해주기**<br>
패키지 빌드 후 패키지의 노드를 실행해보면 해당 패키지를 찾을 수 없다는 에러가 발생한다. 패키지 빌드가 끝나면 install 폴더 내에 패키지가 설치되지만, 쉘 환경에는 아직 경로가 등록되어 있지 않기 때문에 문제가 발생하는 거다. 

패키지 실행을 위해선 노드가 어디에 있는지, 메세지 타입이나 라이브러리가 어디에 있는지를 알아야 함.<br>
→ **`source` 명령어로 install 폴더에 생성된 설정 파일 실행하여 경로를 환경 변수에 등록해주면 패키지와 노드를 찾을 수 있게 됨**

```bash
source install/setup.bash
```

- 터미널 새로 열 때마다 `source` 다시 해주어야 하니, 자주 쓴다면 `.bashrc`에 추가해서 자동화해주기
<br><br>

**2. 패키지의 노드 실행**

```bash
ros2 run <pkg-name> <node-name>
```

