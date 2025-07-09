import matplotlib.pyplot as plt
import numpy as np

# 1. 创建一个图像画布
# 设置图像大小，例如 5x5 英寸
image_width = 5
image_height = 5
dpi = 100  # 每英寸的点数，影响图像的清晰度和文件大小

# 创建一个 Figure 和 Axes 对象
fig, ax = plt.subplots(figsize=(image_width, image_height), dpi=dpi)

# 2. 将画布填充为白色
# 设置背景颜色
fig.patch.set_facecolor('white')
ax.set_facecolor('white')

# 也可以直接创建一个白色画布的 numpy 数组
# image_size = (200, 200) # 设置图像的像素尺寸
# image = np.ones((image_size[0], image_size[1], 3), dtype=np.uint8) * 255 # 创建全白色图像 (RGB)
# ax.imshow(image)

# 3. 在画布上绘制一个黑色的十字
# 定义十字的中心位置和大小
center_x = image_width / 2
center_y = image_height / 2
cross_size = 0.4 # 十字的臂的长度 (与图像大小相关的比例)
line_width = 0.4# 十字线条的宽度 (与图像大小相关的比例)

# 绘制水平线
ax.axhline(y=center_y, color='red', linewidth=line_width * dpi)

# 绘制垂直线
ax.axvline(x=center_x, color='red', linewidth=line_width * dpi)

# 调整坐标轴的显示范围，使其充满整个画布
ax.set_xlim(0, image_width)
ax.set_ylim(0, image_height)

# 隐藏坐标轴上的刻度和标签，只显示图形
ax.set_xticks([])
ax.set_yticks([])
ax.set_xticklabels([])
ax.set_yticklabels([])

# 移除坐标轴的边框
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)

# 确保图像是正方形的
ax.set_aspect('equal', adjustable='box')

# 在靠近中心十字的左下角和右下角添加带边框的数字
# 定义边框样式
bbox_props = dict(boxstyle='square,pad=0.3', facecolor='white', edgecolor='black', linewidth=1)
# 参数: x坐标, y坐标, 文本, 字体大小, 颜色, 水平对齐, 垂直对齐, 边框
ax.text(center_x - 0.4, center_y - 0.4, '3', fontsize=80, color='black', ha='right', va='top', fontfamily='cursive')
ax.text(center_x + 0.4, center_y - 0.4, '4', fontsize=80, color='black', ha='left', va='top', fontfamily='cursive')

# 显示图形
plt.show()
