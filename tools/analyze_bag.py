import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

# ================= 配置区域 =================
# 如果你的bag文件路径不一样，请在这里修改，或者通过命令行参数传入
DEFAULT_BAG_PATH = 'test_dynamic_03' 
TOPIC_NAME = '/fix'
# ===========================================

def latlon_to_xy(lat, lon, lat0, lon0):
    """
    将经纬度转换为局部XY坐标（单位：米）
    简单投影法，适用于小范围（几公里内）
    """
    R = 6378137.0 # 地球半径
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    lat_avg = math.radians((lat + lat0) / 2.0)
    
    x = R * d_lon * math.cos(lat_avg) # 东向距离
    y = R * d_lat                     # 北向距离
    return x, y

def get_status_color(status):
    """
    根据ROS状态返回颜色
    Status 2 (GBAS) -> RTK Fixed -> 绿色
    Status 1 (SBAS) -> DGPS -> 黄色
    Status 0 (FIX)  -> Single -> 红色
    """
    if status == 2:
        return 'green', 'RTK Fixed (2)'
    elif status == 1:
        return 'gold', 'DGPS (1)'
    else:
        return 'red', 'Single/Float (0)'

def analyze_bag(bag_path):
    print(f"📂 正在读取数据包: {bag_path} ...")
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        # 尝试使用 sqlite3 再次打开 (旧版本默认是 sqlite3)
        try:
            storage_options.storage_id = 'sqlite3'
            reader.open(storage_options, converter_options)
        except:
            print(f"❌ 无法打开数据包，请检查路径是否正确: {bag_path}")
            return

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if TOPIC_NAME not in type_map:
        print(f"❌ 数据包中未找到话题 {TOPIC_NAME}")
        return

    xs, ys, statuses = [], [], []
    lats, lons = [], []
    
    lat0, lon0 = None, None
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == TOPIC_NAME:
            msg = rclpy.serialization.deserialize_message(data, NavSatFix)
            
            # 过滤无效数据
            if msg.status.status < 0:
                continue
                
            lat = msg.latitude
            lon = msg.longitude
            status = msg.status.status # 这里对应我们驱动里的映射：2=RTK, 1=DGPS
            
            # 设置原点 (第一帧)
            if lat0 is None:
                lat0 = lat
                lon0 = lon
            
            x, y = latlon_to_xy(lat, lon, lat0, lon0)
            
            xs.append(x)
            ys.append(y)
            statuses.append(status)
            lats.append(lat)
            lons.append(lon)

    if not xs:
        print("⚠️ 没有读取到有效的定位数据！")
        return

    # ================= 数据分析 =================
    total_points = len(xs)
    fixed_points = statuses.count(2)
    fixed_ratio = (fixed_points / total_points) * 100
    
    # 闭环误差计算 (距离原点的偏差)
    # 我们取最后 5 个点的平均值作为“终点”，减少单点跳动影响
    end_x = np.mean(xs[-5:])
    end_y = np.mean(ys[-5:])
    
    # 起点就是 (0,0)
    loop_error = math.sqrt(end_x**2 + end_y**2)
    
    print("-" * 30)
    print("📊 分析报告 (Analysis Report)")
    print("-" * 30)
    print(f"📍 数据点总数: {total_points}")
    print(f"✅ RTK固定率 : {fixed_ratio:.2f}% (Status=2)")
    print(f"📏 行驶总时长: {(t - 0)/1e9:.1f} 秒 (估算)") # 粗略估算
    print("-" * 30)
    print(f"🎯 闭环误差 (Loop Closure Error):")
    print(f"   >>> {loop_error * 100:.2f} cm <<<")
    print("-" * 30)
    
    if loop_error < 0.05:
        print("🏆 评级: 顶级精度 (Master Class) < 5cm")
    elif loop_error < 0.10:
        print("🥇 评级: 优秀 (Excellent) < 10cm")
    elif loop_error < 0.20:
        print("🥈 评级: 良好 (Good) < 20cm")
    else:
        print("⚠️ 评级: 需检查 (Check Required) > 20cm")

    # ================= 绘图 =================
    plt.figure(figsize=(10, 8))
    plt.title(f"Trajectory Analysis\nLoop Error: {loop_error*100:.2f} cm | Fixed Rate: {fixed_ratio:.1f}%")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # 绘制轨迹点，根据状态变色
    colors = [get_status_color(s)[0] for s in statuses]
    plt.scatter(xs, ys, c=colors, s=10, alpha=0.6, label='Path Points')
    
    # 标记起点和终点
    plt.scatter(0, 0, c='blue', marker='+', s=200, linewidth=3, label='Start (Origin)')
    plt.scatter(end_x, end_y, c='purple', marker='x', s=200, linewidth=3, label='End (Stop)')
    
    # 画连接线
    plt.plot(xs, ys, c='gray', alpha=0.3, linewidth=1)
    
    # 创建图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', label='RTK Fixed (2)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='gold', label='DGPS (1)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', label='Single (0)'),
        Line2D([0], [0], marker='+', color='blue', markersize=10, label='Start Point'),
        Line2D([0], [0], marker='x', color='purple', markersize=10, label='End Point'),
    ]
    plt.legend(handles=legend_elements)
    
    print("📈 正在显示轨迹图...")
    plt.show()

if __name__ == "__main__":
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG_PATH
    if not os.path.exists(bag_path):
        print(f"❌ 文件不存在: {bag_path}")
        print("用法: python3 analyze_bag.py <bag_file_path>")
    else:
        analyze_bag(bag_path)