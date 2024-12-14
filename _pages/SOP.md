---
layout: archive
title: "My Statement of Purpose"
permalink: /sop/
author_profile: true
redirect_from:
  - /sop
  - /wenshu
---



![image](https://github.com/user-attachments/assets/3367f37c-b3c6-46f1-919f-9854a97f9e75)

# From the Moment I Conceptualized My First Robot

From the moment I conceptualized my first robot, I have been captivated by a fundamental question: **What does it take to create a truly intelligent, human-like robotic agent?** Robotics, as a multidisciplinary field shown in Fig. 1, requires the seamless integration of diverse domains, including but not limited to dynamics, perception, multimodal interaction, task planning, and artificial intelligence (AI). Yet, among these, what domain holds the key to unlocking a breakthrough in intelligent robotics? This question has not only fueled my academic journey but also drives my research aspirations.

![Figure 1: What it would take to create a truly intelligent robotic agent.](#)

## Academic Journey and Research Aspirations

As the [best overall academic performance](https://tianyi20.github.io/award/) awarded student at Xi’an Jiaotong-Liverpool University, I had the privilege of working as a research assistant under Dr. Quan Zhang throughout my undergraduate studies, focusing on **Motion Planning**. Specifically, we developed a novel motion planning framework within a digital twin system, leading to my first-author publication: [“Development of a Simple and Novel Digital Twin Framework for Industrial Robots in Intelligent Robotics Manufacturing”](https://ieeexplore.ieee.org/abstract/document/10711459) [1] , [Video](https://www.youtube.com/watch?v=f_BEMbMvFso&t=1s) . This work addressed the “delay” issue typically encountered in teleoperation motion planning by achieving a novel 58.82 Hz control & monitoring frequency. Our framework was designed to enable smooth teleoperation and feed relatively asymptotic trajectory for “imitation learning” tasks. However, this work revealed significant limitations in traditional robotic systems, particularly in their capability to handle flexible materials and deformable objects. 

To overcome these challenges, I equipped the robot with a soft pneumatic gripper, modeled as an underactuated robotic finger, and developed a vision-calibrated motion planning framework to enhance its adaptability and precision. This effort resulted in my second first-author publication: [“A Novel Approach to Grasping Control of Soft Robotic Grippers based on Digital Twin”](https://ieeexplore.ieee.org/abstract/document/10718822) [2].

## Exploration in Perception at Yale University

My curiosity about robot perception deepened further during my time at Yale University, where I was inspired by the groundbreaking research of Prof. Brian Scassellati and Prof. Aaron Dollar. I came to realize that robots still struggle with a seemingly fundamental problem: **Perception**. To the best of my knowledge, most contemporary robotics research heavily relies on assumptions about known dimensions, appearances, and materials of target objects, often requiring precisely pre-defined CAD models, as seen in [Boston Dynamics' Atlas robot](https://bostondynamics.com/video/atlas-goes-hands-on/) [3]. 

In contrast, humans can effortlessly perceive unseen objects’ physical properties like pose, appearance, and even mass of inertia. Motivated by this realization, I took Prof. Scassellati’s course and embarked on solving a classical robotic challenge: **liquid manipulation**. The most challenging part is how to generate a feasible pouring trajectory given diverse poses and dimensions of different cups and bottles. To address this, I developed a category-level object pose and dimension estimation detector and formulated the trajectory planning problem as an optimization problem solved by `scipy`. [[Video](https://www.youtube.com/watch?v=oPvfIooH5HU); [detector github](https://github.com/Tianyi20/category-level-estimation-ROS-noetic) , [optimizer and moveit github](https://github.com/Tianyi20/liquid_manipulation_moveit)]. 

Despite these efforts, I found that such perception methods are still limited by pre-trained category-level weights. Motivated to address this limitation, I had the opportunity to collaborate with [Dr. Yifan Zhu](https://yifanzhu95.github.io/) in Prof. Dollar’s group. We proposed an end-to-end framework to jointly optimize shape, appearance, and physical properties of the scene. This framework leverages a novel differentiable point-based object representation coupled with a grid-based appearance field, which allows differentiable object collision detection and rendering. Given 3D cloud data and tactile sensor input, the robot can eventually optimize an object’s appearance, geometry, and physical parameters. While the current optimization process takes 15 minutes, it provides valuable insights into developing a generalizable solution for object estimation. Moreover, the runtime can be further improved with better initial guesses and pre-trained models. This also led to a second-author paper titled: [“Real-to-Sim via End-to-End Differentiable Simulation and Rendering”](https://arxiv.org/pdf/2412.00259) [4], submitted to the IEEE Robotics and Automation Letters (RA-L 2024).

## Future Directions: Integrated Framework for Intelligent Robots

Building on my previous experiences in motion planning and perception, I now believe that the next frontier for truly intelligent robotic agents lies in developing a generalization framework that integrates:

### (1) Task Planning with Perception
A robust task planner is essential for robots operating under uncertainty and long-horizon planning. A perception method is required to work alongside the planner, inferring “predicates” and “states” information and feeding this into the task planner and low-level motion planner.

### (2) Dexterity Manipulation
At the core of a low-level task controller within task planning is human-level dexterity in robotic manipulation. It should be generalized to execute different fine-grained tasks while considering complex physical constraints and contact modes.




[1] T. Xiang, B. Li, X. Pan and Q. Zhang, "Development of a Simple and Novel Digital Twin Framework for Industrial Robots in Intelligent Robotics Manufacturing," 2024 IEEE 20th International Conference on Automation Science and Engineering (CASE). Available: https://ieeexplore.ieee.org/abstract/document/10711459


[2] T. Xiang, B. Li, Q. Zhang, M. Leach and E. Lim, "A Novel Approach to Grasping Control of Soft Robotic Grippers based on Digital Twin," 2024 29th International Conference on Automation and Computing (ICAC). Available: https://ieeexplore.ieee.org/abstract/document/10718822


[3]“Atlas Goes Hands On,” Boston Dynamics. Accessed: Nov. 29, 2024. [Online]. Available: https://bostondynamics.com/video/atlas-goes-hands-on/


[4] Y. Zhu, T. Xiang, A. Dollar, and Z. Pan, “Real-to-Sim via End-to-End Differentiable Simulation and Rendering”. Robotics and Automation Letters (RAL), Manuscript submitted for publication. Available: https://arxiv.org/pdf/2412.00259


