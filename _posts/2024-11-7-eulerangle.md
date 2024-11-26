---
title: 'euler angle and quaternion and how to do superposition '
date: 2024-11-10
permalink: /posts/2024/11/blog-post-2/euler-angle
tags:
  - euler angle, how to realize superposition
  - quaternion
  - ROS, tf
---

![image](https://github.com/user-attachments/assets/2446cdf2-1379-48b2-b832-ce35ba58665b)

首先普及基础知识 欧拉角 and quaternion，fianlly do superposition rotation.

欧拉角，Tait–Bryan angles 也被称为Cardan angles, nautical angles, (heading, elevation, and bank),(yaw, pitch, and roll). 我们接触的比较多的是yaw(偏航), pitch(俯仰), roll(横滚).三个变量一般对应(车体,飞行器的)z,y,x三个坐标轴

[网站：https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/](https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/)


一般对于旋转矩阵(3*3),旋转向量/角轴(3*1),四元数(4*1),我们给定一串数字,就能表示清楚一个姿态/旋转.比如这里给出一个旋转矩阵R:

![image](https://github.com/user-attachments/assets/4af91354-58cb-40c4-b7f0-05eae3f1fd8f)

# The most important things:

## euler angle 是基于顺序旋转的，z转完了以后，y再接着转，最后是x，所以最后的outcome一定要注意旋转顺序

#最常用的欧拉角旋转顺序是 **Z-Y-X**，即：

1. **首先绕 Z 轴旋转**（Yaw），
2. **然后绕新的 Y 轴旋转**（Pitch），
3. **最后绕新的 X 轴旋转**（Roll）。

这个顺序在机器人学、航空航天和计算机图形学中非常常见，因为它能够有效地描述三维空间中的方向和姿态变化。

- **Z-Y-X (Yaw-Pitch-Roll)**：最常见于无人机、飞机、机器人臂等，需要在三维空间中自由移动的应用中。

使用 Z-Y-X 的好处是，它能避免部分情况下的万向锁问题（即Pitch等于 ±90° 时出现的奇点）。在 ROS 和一些三维图形工具中，默认的欧拉角顺序一般就是 Z-Y-X。


### **旋转矩阵推导**
假设欧拉角为 \((\theta_z, \theta_y, \theta_x)\)，旋转矩阵 \(R\) 是按顺序乘积的结果：
\[
R = R_x(\theta_x) \cdot R_y(\theta_y) \cdot R_z(\theta_z)
\]
#### 矩阵是从右边往左乘的，所以这里是先Z，再Y，最后X


### **总结**
- 欧拉角 \(Z-Y-X\) 的顺序是：**Z旋转 → Y旋转 → X旋转**。
- 对应的旋转矩阵是按照 \(R_x \cdot R_y \cdot R_z\) 的顺序进行矩阵乘法。
其中每个分量旋转矩阵为：
- 绕 \( Z \)-轴的旋转矩阵 \( R_z(\alpha) \)：
- 
$$
  R_z(\alpha) = 
  \begin{bmatrix}
  \cos \alpha & -\sin \alpha & 0 \\
  \sin \alpha & \cos \alpha & 0 \\
  0 & 0 & 1
  \end{bmatrix}
$$

- 绕 \( Y \)-轴的旋转矩阵 \( R_y(\beta) \)：
- 
$$
  R_y(\beta) = 
  \begin{bmatrix}
  \cos \beta & 0 & \sin \beta \\
  0 & 1 & 0 \\
  -\sin \beta & 0 & \cos \beta
  \end{bmatrix}
$$

- 绕 \( X \)-轴的旋转矩阵 \( R_x(\gamma) \)：
- 
$$
  R_x(\gamma) = 
  \begin{bmatrix}
  1 & 0 & 0 \\
  0 & \cos \gamma & -\sin \gamma \\
  0 & \sin \gamma & \cos \gamma
  \end{bmatrix}
$$

### 组合后的旋转矩阵 \( R \)

将三个旋转矩阵相乘，即：

$$R = 
\begin{bmatrix}
\cos \alpha & -\sin \alpha & 0 \\
\sin \alpha & \cos \alpha & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
\cos \beta & 0 & \sin \beta \\
0 & 1 & 0 \\
-\sin \beta & 0 & \cos \beta
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos \gamma & -\sin \gamma \\
0 & \sin \gamma & \cos \gamma
\end{bmatrix}
$$

经过矩阵乘法展开，我们得到最终的旋转矩阵：

$$
R = 
\begin{bmatrix}
\cos \alpha \cos \beta & \cos \alpha \sin \beta \sin \gamma - \sin \alpha \cos \gamma & \cos \alpha \sin \beta \cos \gamma + \sin \alpha \sin \gamma \\
\sin \alpha \cos \beta & \sin \alpha \sin \beta \sin \gamma + \cos \alpha \cos \gamma & \sin \alpha \sin \beta \cos \gamma - \cos \alpha \sin \gamma \\
-\sin \beta & \cos \beta \sin \gamma & \cos \beta \cos \gamma
\end{bmatrix}
$$

这个矩阵 \( R \) 就是对应给定欧拉角 \( (\alpha, \beta, \gamma) \) 的旋转矩阵。

# 其次是内旋or 外旋

内旋,外旋的区别在于: 内旋的旋转frame是旋转后的物体本身的frame，而外旋一直是世界坐标的frame

在转β(第二个转角)时:

内旋按照旋转后物体的坐标y轴,也就是 
 旋转.

外旋按照世界坐标系中的Y轴旋转.

旋转最后一个角度时亦然.



因此, 增加了这两个概念(旋转顺序, 内外旋)后,我们描述一个能表示确定姿态/旋转的欧拉角,应该这样:

旋转角度(α,β,γ),旋转顺序(z->y->x),外旋.

或者:

旋转角度(α,β,γ),旋转顺序(x->y->z),内旋.

等等, 三个元素缺一不可.


# 回到问题本身，如何实现旋转叠加叠加

我们先把熟悉的欧拉角需要叠加上的转化成四元数，随后把需要叠加的四元数直接乘到当前的四元数上就可以了。

因为四元数相乘就等于旋转叠加。
ROS moveit中代码实现: `tf.transformations.quaternion_from_euler`
```javascript
import tf

def rotation_plus_euler(self, euler_added):
    """
    maintain the current pose, but only change the euler angle
    """        
    # 获取当前的位姿
    current_pose = self.move_group.get_current_pose().pose
    target_pose = Pose()
    target_pose.position = current_pose.position

    # 获取当前的四元数
    current_orientation_q = [
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w,
    ]

    # 将要添加的欧拉角转换为四元数
    quaternion_added = tf.transformations.quaternion_from_euler(*euler_added)

    # 将两个四元数相乘
    target_orientation_q = tf.transformations.quaternion_multiply(current_orientation_q, quaternion_added)

    # 将计算得到的新四元数赋值给目标位姿
    target_pose.orientation.x = target_orientation_q[0]
    target_pose.orientation.y = target_orientation_q[1]
    target_pose.orientation.z = target_orientation_q[2]
    target_pose.orientation.w = target_orientation_q[3]

    # 移动到目标位姿
    return self.move_to_pose(target_pose)

```
