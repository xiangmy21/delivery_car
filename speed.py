import matplotlib.pyplot as plt

# 读取输入数据并转换为四个速度列表
speed_data = []
while True:
    try:
        line = input()
        speeds = list(map(float, line.strip().split()))
        speed_data.append(speeds)
    except EOFError:
        break
print(speed_data[0])
speed_wheel1 = [speeds[1] for speeds in speed_data]
speed_wheel2 = [speeds[2] for speeds in speed_data]
speed_wheel3 = [speeds[3] for speeds in speed_data]
speed_wheel4 = [speeds[4] for speeds in speed_data]

# 创建一个新的图形
fig = plt.figure()

# 添加一个子图
ax = fig.add_subplot(1, 1, 1)

# 绘制四个轮子的速度序列
ax.plot(speed_wheel1, label='Wheel 1')
ax.plot(speed_wheel2, label='Wheel 2')
ax.plot(speed_wheel3, label='Wheel 3')
ax.plot(speed_wheel4, label='Wheel 4')

# 添加标题和图例
plt.title('Speed of Four Wheels')
plt.legend()

# 显示图形
plt.show()
