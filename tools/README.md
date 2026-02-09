附赠工具：动态精度分析脚本

本仓库提供了一个 Python 脚本，用于读取 rosbag 并自动计算 RTK 闭环误差、绘制轨迹图。

 1. 准备工作
确保安装绘图库：
```bash
sudo apt install python3-matplotlib python3-numpy

2. 使用方法
录制一段包含 /fix 话题的数据包后，运行：
# 运行脚本 (在 tools 目录下)
python3 tools/analyze_bag.py path/to/your/bag_file


