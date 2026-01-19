nmea_navsat_driver
===============

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver


# UM982 RTK Driver for ROS 2 Jazzy (LubanCat 5)

这是基于 `nmea_navsat_driver` 修改的 UM982 驱动，专配 **鲁班猫5 (Ubuntu 24.04 + ROS 2 Jazzy)**。

**主要修复与功能：**
1.  **自动初始化**：驱动启动时会自动发送命令激活 UM982 的 GNGGA/RMC/HDT 输出，防止板子“哑巴”。
2.  **全星座支持**：修改了源码，完美兼容北斗 `$GNGGA` 协议头，解决了 ROS 驱动只认 `$GPGGA` 导致 Status -1 的问题。
3.  **抗干扰**：禁用了 RMC 数据对 Status 的干扰，解决了 Status 在 4 和 -1 之间跳变的问题。
4.  **波特率锁死**：默认配置为 115200。

## 环境要求
* **Hardware**: LubanCat 5 (v2)
* **OS**: Ubuntu 24.04 Noble
* **ROS Distro**: ROS 2 Jazzy
* **Sensor**: Unicore UM982 (RTK Module)

## 快速开始

### 1. 安装依赖
```bash
# 安装 Python 串口库
pip3 install pyserial

# 安装 ROS 消息依赖
sudo apt update
sudo apt install ros-jazzy-nmea-msgs ros-jazzy-geographic-msgs
```

### 2. 下载源码
创建工作空间并 clone 本仓库：
```bash
mkdir -p ~/rtk_ws/src
cd ~/rtk_ws/src
git clone https://github.com/EcoHarry8341/um982_driver_ros2_jazzy.git
```

### 3. 编译
```bash
cd ~/rtk_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. 运行
确保 UM982 已连接到 `/dev/ttyUSB0` (代码里映射了别名，或者直接改 launch 文件)。

```bash
# 启动驱动
ros2 launch nmea_navsat_driver um982.launch.py
```

### 5. 验证
```bash
# 查看状态 (Status 2 代表 RTK 固定/浮点解，Status 1 代表普通差分)
ros2 topic echo /fix
```

## 注意事项
* 如果在室内测试，Status 可能是 0 或 -1，但只要有数据流就是正常的。
* Launch 文件中 `useRMC` 已默认设为 `False` 以避免状态冲突。
