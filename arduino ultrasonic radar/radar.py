import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# 串口初始化
def init_serial(port='COM10', baudrate=115200):
    return serial.Serial(port, baudrate, timeout=1)

# 从串口读取数据
def read_data(serial_conn):
    try:
        if serial_conn.in_waiting > 0:
            data = serial_conn.readline().decode('utf-8').strip()
            angle, distance = map(float, data.split(','))
            return angle, distance
    except:
        pass
    return None, None

# 初始化雷达显示
def init_radar_plot():
    fig = plt.figure(facecolor='black')
    ax = fig.add_subplot(111, polar=True, facecolor='black')
    ax.set_ylim(0, 100)
    ax.set_xlim(0, np.pi)
    ax.set_facecolor('black')
    ax.grid(True, color='gray')
    ax.tick_params(colors='white')
    return fig, ax

# 更新透明度，距离超出范围时设置为None不显示
def update_data(data, angle, distance):
    if 0 <= distance <= 100 and 0 <= angle <= 180:
        data['distance'][int(angle)] = distance
        data['alpha'][int(angle)] = 1  # 新数据设置为最高透明度
    data['alpha'] = [max(alpha - 0.005, 0) for alpha in data['alpha']]  # 每次减少0.5%

# 初始化0-180度的数据
def init_data():
    return {
        'distance': [None] * 181,
        'alpha': [0] * 181
    }

# 绘制雷达扫描线（根据时间渐变，控制最近的线数量）
def draw_scan_line(ax, angle, scan_lines, max_lines=120):
    scan_angle_rad = np.deg2rad(angle)

    # 新的扫描线加入队列
    line, = ax.plot([scan_angle_rad, scan_angle_rad], [0, 100], color='green', alpha=1)
    scan_lines.append(line)

    # 控制扫描线的数量，超出数量的线从列表中移除并从图中删除
    if len(scan_lines) > max_lines:
        old_line = scan_lines.pop(0)  # 移除最早的扫描线
        old_line.remove()  # 从图中删除

    # 更新每根线的透明度，最近的线最清晰，越早的线越透明
    for i, line in enumerate(scan_lines):
        alpha =(i / max_lines)  # 透明度从1逐渐降到0
        line.set_alpha(alpha)

# 更新雷达显示
def update_radar(frame, serial_conn, scatter, data, ax, scan_lines):
    angle, distance = read_data(serial_conn)
    
    if angle is not None and distance is not None:
        update_data(data, angle, distance)
        draw_scan_line(ax, angle, scan_lines)  # 绘制或更新扫描线

    angles = np.deg2rad(np.arange(0, 181))
    distances = [d if a > 0 else None for d, a in zip(data['distance'], data['alpha'])]
    alphas = data['alpha']

    # 更新散点图
    scatter.set_offsets(np.c_[angles, distances])
    scatter.set_alpha(alphas)

    return scatter,

# 主程序
def main(refresh_interval=0, max_lines=30):  # 设置刷新时间和扫描线数量
    port = 'COM10'
    baudrate = 115200
    serial_conn = init_serial(port, baudrate)
    
    fig, ax = init_radar_plot()

    # 设置绿色散点
    scatter = ax.scatter([], [], c='lime', s=50, cmap='Greens', alpha=1)

    data = init_data()

    # 储存扫描线的列表
    scan_lines = []

    ani = FuncAnimation(fig, update_radar, fargs=(serial_conn, scatter, data, ax, scan_lines), interval=refresh_interval)
    
    plt.show()

if __name__ == "__main__":
    main(refresh_interval=5, max_lines=30)  # 调整刷新速度和扫描线数量
