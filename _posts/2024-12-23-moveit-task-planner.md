---
title: 'Try ROS moveit task planner'
date: 2024-12-23
permalink: /posts/2024/12/blog-moveit-task-planner/
tags:
  - moveit task planner
---

<iframe width="560" height="315" src="https://www.youtube.com/embed/CUsAr37loGw" 
        title="YouTube video player" frameborder="0" 
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
        allowfullscreen>
</iframe>



### MoveIt Task Constructor (MTC) 简述

MoveIt Task Constructor (MTC) 是一种模块化运动规划框架，将复杂任务分解为多个阶段（Stages），通过生成（Generator）、传播（Propagator）、连接（Connector）三类阶段完成规划。阶段结果按需传递，最终优化任务的总成本。

容器（Containers）用于组织阶段，包括串行（按顺序执行）、并行（选取最优解）和包装器（过滤结果）。MTC 强调预定义任务序列，无法端到端生成任务，但适合抓取、放置等复杂任务的路径优化与策略选择。


他这个是有 generator 在底层生成不同 cost 的比如说抓取等策略的优化，然后计算总 cost ，得到最好的一套路径. 本质是就是把单个motion planning这个优化上升到 全部task的motion planning 层面上了优化。 
**比如说上一个stage的状态会影响下一个stage planning出来的路径**

