---
title: 'My understanding of optimisation based planning & sampling based motion planning'
date: 2024/11/11
permalink: /posts/2024/11/blog-post-1/
tags:
  - optimisation based planning
  - sampling based motion planning
  - RRT; RRTstar
  - ROS
  - Moviet
---

今天在聊天时候 突然发现自己对 sampling based motion planning 和 optimisation based planning 有误解。写一个帖子记录一下。

reference :

[https://slides.com/russtedrake/fall23-lec11#/19/0/2](https://slides.com/russtedrake/fall23-lec11#/19/0/2)

# 首先介绍 optimisation based planning

Firstly, it comes to how to minimize the distance within start position and end position. noting it is within configuration space (关节空间) . 

![image](https://github.com/user-attachments/assets/b17911b1-feee-4986-ba18-adf1aa812f61)


it comes to be a convex optimisation problem. 
因为我对 凸优化甚至不了解，先简单了解一下凸优化。

将路径规划问题表述为一个优化问题。这种方法通常用于找到从起点 $q_{\text{start}}$ 到终点 $ q_{\text{goal}}$ 的一条最优路径，同时避开障碍物。

### 优化目标

优化的目标是通过求解路径中相邻点之间的距离之和来最小化路径的平滑度，即：

$$
\min_{q_0, \ldots, q_N} \sum_{n=0}^{N-1} \| q_{n+1} - q_n \|_2^2
$$

在这里，目标函数 $$| q_{n+1} - q_n \|_2^2 $$ 表示相邻点之间的距离的平方和。这一项通过最小化路径上的位置变化，鼓励生成平滑的路径，使路径不会有过多的急剧转弯或突变。

这就是凸优化了
在表达式 \( \|Ax - b\|_2 \) 中， \( \|Ax - b\|_2 \) 表示的是向量 \( Ax - b \) 的二范数，即这个向量的长度（或称为欧几里得距离）。为了详细解释为什么它可以展开成多个项，我们可以一步步来看这个二范数的定义。

![image](https://github.com/user-attachments/assets/104b0641-44f6-43f4-a648-8770f17e022f)

### 二范数的定义

对于一个向量 \( y = (y_1, y_2, \ldots, y_n) \)，它的二范数 \( \|y\|_2 \) 定义为：

$$
\|y\|_2 = \sqrt{y_1^2 + y_2^2 + \cdots + y_n^2}
$$

换句话说，二范数是向量中各个分量平方的和的平方根。

### 应用到 \( \|Ax - b\|_2 \)

在最小二乘问题中，\( Ax - b \) 是一个向量，代表矩阵 \( A \) 作用于向量 \( x \) 后得到的结果与向量 \( b \) 之间的差异。

假设 \( Ax - b = y \)，其中 \( y \) 是一个 \( n \)-维向量，即 \( y = (y_1, y_2, \ldots, y_n) \)。那么 \( \|Ax - b\|_2 \) 就是 \( \|y\|_2 \)，可以展开为：

$$
\|Ax - b\|_2 = \sqrt{(y_1)^2 + (y_2)^2 + \cdots + (y_n)^2}
$$


# 但是optimisation based motion planning 的缺点是

非凸性导致局部最优解；prerequisite you need to know is: 优化规划（optimization-based planning）方法通常是数值解
如在之前的例子中，优化目标是平滑路径，同时满足碰撞回避的非凸约束。这些非凸约束（例如避开障碍物）使得整个优化问题成为非凸问题，而非凸优化问题的特点之一是可能存在多个**局部最优解。因此，优化算法往往只能找到某一个局部最优解，而非全局最优解。比如下图，有多个局部最优解。类似于我们做梯度下降，优化算法往往只能找到某一个局部最优解，而非全局最优解



因此，优化算法往往只能找到某一个局部最优解，而非全局最优解。比如下图，有多个局部最优解。类似于我们做梯度下降，优化算法往往只能找到某一个局部最优解，而非全局最优解



![image](https://github.com/user-attachments/assets/239cb595-8b90-41aa-b280-0ba15ae16034)

除此之外，在真实环境中使用 optimisation based planning往往需要给他一个初始路径、优化方法往往需要一个初始解，例如通过某种启发式算法（如 RRT 或 RRT*）生成的初始路径。在优化的过程中，算法会在这个初始解的基础上进行改进。由于非凸性，算法可能会收敛到离初始解较近的一个局部最优解。因此，初始路径的质量对最终解的优劣有很大影响。

# 关于RRTstar 和 optimisation based planning

RRTstar rewiring的定义，是 渐进最优，asymptotically optimal，优化是一个渐进过程。随着采样点数量的增加，这些方法可以逐渐靠近最优解，就是逐渐靠近optimisation based 最优解，而不能保证是全局最优解，也不能保证是局部最优解。

# 下面我们讲 Sampling based motion planning
![image](https://github.com/user-attachments/assets/6065a323-f505-47cd-8044-f370c6781a54)

## RRTconnect

关于RRTconnect，其实就是从终点和起点都长出来一颗树，在两棵树接触的时候，就视为成功。RRTconnect的规划完成速度是真的快，而且RRTconnect保证在存在一条路径的时候，就一定可以找到！！所以如果是RRTconnect报错：

```
[ WARN] [1567497398.552827574]: Fail: ABORTED: No motion plan found. No execution attempted.

[ERROR] [1567673055.288638354]: RRTConnect: Unable to sample any valid states for goal tree
```

那就不是更换一个planner就可以解决的问题，说明你期望他规划的路径，真的无论如何都没有，需要检查初始点和终点。

## RRTstar

**RRTstar** planning required time is too long!!!

RRTstar是在RRT找到路径的基础上，对RRT的sampling point rewiring，RRTstar rewiring的定义，是 [渐进最优]，asymptotically optimal，优化是一个渐进过程。随着采样点数量的增加，这些方法可以逐渐靠近最优解，就是逐渐靠近optimisation based 最优解，而不能保证是全局最优解，也不能保证是局部最优解。

## PRM, PRMstar

PRM（概率道路图）
PRM 的基本思想是首先在整个空间中采样一些点，然后将这些点用无碰撞路径连接起来，构建一个连通图，再在图上搜索起点和终点之间的路径。具体步骤如下


## 关于为什么MOVEIT 官方特别喜欢RRTconnect：
RRTconnect是MOVEIT的默认算法，这里引用MOVEIT author Mark Moll 对RRTstart和RRTconnect，以及post processing的理解：
![image](https://github.com/user-attachments/assets/f6b54528-4864-40e7-9c43-b3f76e7c0319)

**[No magical silver bullet](https://github.com/moveit/moveit/issues/197#issuecomment-249446353):**

Optimizing planners RRT* and PRM* are asymptotically optimal but use all time to do planning. The convergence rate tends to be pretty slow。


不过MOVEIT的pose processing就是在RRTconnect找到轨迹后用B spline去做trajectory 优化，这个pose processing应该指的是这里trajectory planning的优化时间？

在 MoveIt 中，Pose Processing 的关键步骤包括：
姿态采样：首先生成粗略的路径，包含一系列的姿态点。
优化和平滑：使用 B样条等方法来优化这些姿态点的轨迹，使路径更平滑并符合物理可行性。
轨迹约束：添加速度、加速度等约束，以确保轨迹在实际执行中稳定且安全。

我记得是如果set了一个 planning time，规划完成后剩余的时间就全部用来做post processing?

当设置了一个 规划时间 (planning time) 时，MoveIt 会在这个时间限制内完成路径规划，并将剩余时间用于 后处理 (post-processing)，以优化轨迹的平滑度和可执行性。

所以我们只需要选择RRTconnect，然后设置planning time 长一点，就会有更好的轨迹，但是这样的优化轨迹确实还和RRTstar plan的轨迹有limit。

