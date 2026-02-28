# piper_gripper_cpp_repo

Standalone C++ gripper module extracted from `piper_sdk`.

## Build

```bash
cmake -S . -B build -DPIPER_GRIPPER_BUILD_EXAMPLES=ON
cmake --build build -j4
```

## Run Examples

```bash
./build/test_gripper_cpp
./build/test_gripper_effort_cpp
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## 工控机设置can波特率，已设置完成

```bash
命令行配置：
robot@robot:~$ sudo pkill -2 emucd_64
[sudo] password for robot: 
robot@robot:~$ sudo ip link set can0 down 2>/dev/null || true
robot@robot:~$ sudo ip link set can1 down 2>/dev/null || true
robot@robot:~$ sudo emucd_64 -s9 -e0 /dev/ttyACM0 can0 can1
robot@robot:~$ sudo ip link set can0 txqueuelen 1000
robot@robot:~$ sudo ip link set can1 txqueuelen 1000
robot@robot:~$ sudo ip link set can0 up
robot@robot:~$ sudo ip link set can1 up
robot@robot:~$ ps -ef | grep emucd_64 | grep -v grep
root        4515       1  0 21:54 ?        00:00:00 emucd_64 -s9 -e0 /dev/ttyACM0 can0 can1

开机启动文件配置：
/etc/init.d/run_emucd
修改内容：baudrate=9   
```