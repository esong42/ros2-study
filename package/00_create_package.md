# [Package] 패키지 생성하기

## ros2의 workspace
**workspace 란**<br>
ros2 에서 개발자가 패키지를 만들고, 빌드하고, 실행하는 모든 작업이 이루어지는 디렉토리를 말한다.<br>
소스코드와 해당 소스코드를 빌드하고 나온 결과물(build, install, log)로 구성된다<br><br>

**구조**
```python
ros2_ws/
├── src/          
├── build/       
├── install/     
└── log/ 
```

| **폴더** | **역할** |
| --- | --- |
| **src/** | 만든 패키지 저장 |
| **build/** | [colcon이 생성]: 빌드 설정 파일 저장 |
| **install/** | [colcon이 생성]: 실행 파일 저장 |
| **log/** | [colcon이 생성]: 빌드할 때 생성된 log 저장 |
<br>

## 패키지 생성하기
**패키지 생성하기 (ros2 pkg cerate)**

```bash
ros2 pkg create --build-type ament_python <pkg-name>
```
<br>

- **--build-type 옵션**
    
    빌드는 소스코드를 실행 가능한 형태로 만드는 과정이다.
    
    코드 언어에 따라서 빌드 방식이 다르다. c++는 기계어로 바꾸어야 하기 때문에 빌드 시에 컴파일이 필요하고, python 코드는 인터프리터 언어라서 따로 기계어로 변환하지 않아도 된다.
    
    | **c++** | 컴파일 필요 o → 기계어로 바꿔야 실행됨 |
    | --- | --- |
    | **python** | 컴파일 필요 x → 인터프리터 언어라서 기계어로 변환하지 않아도 실행됨 |
    <br>
    
    위와 같이 **빌드 방식이 다르기 때문에 ros2 에서도 c++ 패키지와 python 패키지를 처리하는 방식(=빌드 방식)이 다르다.** 
    
    **→ ros2 에서 패키지를 생성할 때 해당 패키지의 소스 코드를 어떤 방식으로 빌드할지 정해주어야 한다.**
    
    | **언어** | **--build-type** |
    | ---- | --- |
    | **c++** | ament_make |
    | **python** | ament_python |
<br>

- **--node-name 옵션을 통해 노드도 함께 생성해주기**
    
    ```bash
    ros2 pkg create --build-type ament_python --node-name <node-name> <pkg-name>
    ```
<br>    

**생성한 패키지 구조 살펴보기 (tree)**

- **tree 명령어**
    
    ```bash
    sudo apt install tree # 없으면 install 해주기 
    ```
    
    ```bash
    tree
    ```
    
    - `tree` 는 명령이 실행된 위치에서 폴더와 파일 구조를 보여줌<br><br>
         ```
         .
        └── my_first_package
            ├── my_first_package
            │   ├── __init__.py
            │   └── my_first_node.py
            ├── package.xml
            ├── resource
            │   └── my_first_package
            ├── setup.cfg
            ├── setup.py
            └── test
                ├── test_copyright.py
                ├── test_flake8.py
                └── test_pep257.py

        4 directories, 9 files
         ```      
<br>

## 생성된 패키지 살펴보기
**생성한 노드 코드 살펴보기**

```bash
def main():
		print('Hi from my_first_package.')
	
if __name__ == '__main__':
		main()
```
<br>

- `if __name__ == ‘__main__’` 사용하는 이유
    
    python은 강제 시작점이 없다. c/c++ 는 프로그램 실행 시에 main 함수가 시작점으로 지정이 되어있어서 자동으로 호출이 된다. 하지만 python은 c/c++와 같은 강제 시작점이 없고, 프로그램 실행 시 위에서부터 순차적으로 코드가 실행된다.
    
    main 함수 자체가 특별한 의미를 가지는 게 아니라 그냥 일반 함수 이름으로 판단이 된다.
    
    그래서 스크립트가 직접 실행되었을 경우에 main 함수를 호출하도록 하기 위해서 던더 name 변수를 사용하는 거다. import 로 실행하지 않고 스크립트를 직접 실행하면 던더 name 변수는 __main__이 저장된다.
<br>    

