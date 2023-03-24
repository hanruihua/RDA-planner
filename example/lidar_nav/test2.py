import cv2
import numpy as np
from sklearn.cluster import DBSCAN

# 假设已经订阅到激光扫描数据并存储在scan_data变量中
# scan_data应该是一个长度为n的列表，每个元素是一个长度为2的列表，包含了一个激光点的极坐标(r, theta)。

# 将极坐标转换为笛卡尔坐标系
cartesian_points = []
for scan in scan_data:
    r, theta = scan
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    cartesian_points.append([x, y])

# 将点云转换为numpy数组
points_array = np.array(cartesian_points)

# DBSCAN聚类算法，聚类半径为0.5米，最小样本数为5个点
dbscan = DBSCAN(eps=0.5, min_samples=5)
labels = dbscan.fit_predict(points_array)

# 将不同的簇绘制成不同的颜色
colors = [[0, 0, 255], [0, 255, 0], [255, 0, 0], [255, 255, 0], [0, 255, 255], [255, 0, 255]]
for label in set(labels):
    if label == -1:
        # 不属于任何一个簇的点，用黑色表示
        color = [0, 0, 0]
    else:
        # 属于某个簇的点，使用颜色数组中的颜色表示
        color = colors[label % len(colors)]
    mask = (labels == label)
    cluster_points = points_array[mask]
    
    # 获取簇的最小外接矩形
    rect = cv2.minAreaRect(cluster_points)

    # 获取矩形的4个角点
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # 在原始点云上绘制矩形框和点云
    points = np.vstack([cluster_points, box])
    img = np.zeros((800, 800, 3), dtype=np.uint8)
    img[400:, 400:] = 255
    points = (points + [4.0, 4.0]) * 50.0
    for i in range(points.shape[0] - 4):
        cv2.circle(img, tuple(points[i]), 2, color, -1)
    cv2.drawContours(img, [box], 0, color, 2)
    cv2.imshow('clusters', img)
    cv2.waitKey(0)
