import cv2
import numpy as np
import matplotlib.pyplot as plt

cnt = np.array([[2,3],[4,1],[2,18],[33,1]]) # 必须是array数组的形式
rect = cv2.minAreaRect(cnt) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
# box = np.int0(box)
# 画出来
# box

plt.plot(cnt[:,0],cnt[:,1],'ro')
plt.plot(box[:,0],box[:,1],'b-')
plt.plot([box[3, 0], box[0, 0]], [box[3, 1], box[0, 1]],'b-')

plt.show()