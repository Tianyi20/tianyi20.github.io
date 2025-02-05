---
permalink: /
title: "About me"
author_profile: true
redirect_from: 
  - /about/
  - /about.html
---


I'm a Robotics master student from [Yale University](https://seas.yale.edu/departments/mechanical-engineering-and-materials-science), major in [mechanical engineering & material science](https://seas.yale.edu/faculty-research/research-areas/robotics-mechatronics-and-human-machine-interface). My research interest is **physics-informed machine learning in robotics**, especially **differentiable simulator**, for contact rich manipulation.

I have divided my researches into **four key areas**:

* (A) **Perception** - Infer *uncertain environment*.

* (B) **Reasoning & Task planning** – High-level planner. 

* (C) **Control** - Control for contact rich operation.

* (D) **Mechanics Design** - Co-Design Mechanism with Sensing, Perception, and Control.

Feel free to see **[my research statement]**(../assets/quick_portofolio_research.pdf) for more details, including **future research plans**.


**Contact**: Tianyi.Xiang@yale.edu


# My Resume
You can find my CV here:[Tianyi's CV](../assets/CV_Tianyi_Xiang_Yale.pdf)



<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Publications</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            line-height: 1.4;
            margin: 20px;
        }
        h1, h2, h3, h4, h5, h6 {
            text-align: left; 
            margin-left: 0;
        }
        .publications-section {
            margin: 20px 0;
        }
        .publication-item {
            display: flex;
            align-items: flex-start;
            margin-bottom: 30px;
            gap: 20px;
        }
        .publication-item img {
            width: 350px; /* 更大图片 */
            height: auto;
            border-radius: 6px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); /* 添加阴影 */
        }
        .publication-content {
            flex: 1;
        }
        .publication-title {
            font-weight: bold;
            font-size: 1em; /* 字体变小 */
            margin: 0 0 10px;
        }
        .publication-links {
            margin-top: 10px;
            display: flex;
            gap: 10px;
        }
        .publication-links a {
            text-decoration: none;
            display: inline-block;
            width: 32px;
            height: 32px;
        }
        .publication-links a img {
            width: 100%;
            height: 100%;
        }
        .publication-content p {
            font-size: 0.9em; /* 更小字体 */
            margin: 5px 0;
        }
    </style>
</head>
<body>

<h1>Publications</h1>
<div class="publications-section">
    <div class="publication-item">
        <img src="images/RAL.png" alt="Real-to-Sim">
        <div class="publication-content">
            <p class="publication-title">"Real-to-Sim via End-to-End Differentiable Simulation and Rendering"</p>
            <p>Yifan Zhu, <b>Tianyi Xiang</b>, Aaron Dollar, Zherong Pan</p>
            <p><i>IEEE Robotics and Automation Letters (RA-L 2024)</i>, Manuscript submitted for publication.</p>
            <div class="publication-links">
                <a href="https://arxiv.org/pdf/2412.00259" title="View PDF">
                    <img src="images/pdf_icon.png" alt="PDF Icon">
                </a>
            </div>
        </div>
    </div>

    <div class="publication-item">
        <img src="images/icac.png" alt="Digital Twin Grasping">
        <div class="publication-content">
            <p class="publication-title">"A Novel Approach to Grasping Control of Soft Robotic Grippers based on Digital Twin"</p>
            <p><b>Tianyi Xiang</b>, Borui Li, Quan Zhang, March Leach, Enggee Lim</p>
            <p><i>29th International Conference on Automation and Computing (ICAC 2024)</i>.</p>
            <div class="publication-links">
                <a href="https://arxiv.org/pdf/2410.14928" title="View PDF">
                    <img src="images/pdf_icon.png" alt="PDF Icon">
                </a>
            </div>
        </div>
    </div>

    <div class="publication-item">
        <img src="images/case.png" alt="Digital Twin Framework">
        <div class="publication-content">
            <p class="publication-title">"Development of a Simple and Novel Digital Twin Framework for Industrial Robots in Intelligent Robotics Manufacturing"</p>
            <p><b>Tianyi Xiang</b>, Borui Li, Xiaonan Pan, Quan Zhang</p>
            <p><i>20th International Conference on Automation Science and Engineering (CASE 2024)</i>.</p>
            <div class="publication-links">
                <a href="https://arxiv.org/pdf/2410.14934" title="View PDF">
                    <img src="images/pdf_icon.png" alt="PDF Icon">
                </a>
                <a href="https://www.youtube.com/watch?time_continue=1&v=f_BEMbMvFso" title="Watch Video">
                    <img src="images/youtube_icon.png" alt="YouTube Icon">
                </a>
            </div>
        </div>
    </div>

    <div class="publication-item">
        <img src="images/soft_electronic.png" alt="High Performance Ceramics">
        <div class="publication-content">
            <p class="publication-title">"High performance (Zn₀.₅Mg₀.₅)TiO₃ ceramics based composite films for powering multi-mode translation unit and human motion monitoring"</p>
            <p>B. Xie, Y. Xie, Y. Ma, N. Luo, <b>Tianyi Xiang</b>, et al.</p>
            <p><i>ACS Applied Materials & Interfaces</i>(2024).</p>
            <div class="publication-links">
                <a href="https://pubs.acs.org/doi/abs/10.1021/acsaelm.4c01648" title="View PDF">
                    <img src="images/pdf_icon.png" alt="PDF Icon">
                </a>
            </div>
        </div>
    </div>
</div>

</body>
</html>




# Projects Video & Pictures

### [Real-to-Sim via End-to-End Differentiable Simulation and Rendering](https://arxiv.org/pdf/2412.00259)
[Paper](https://arxiv.org/pdf/2412.00259)  

![screenshot](https://github.com/user-attachments/assets/2dbceb80-58fd-42cd-bf19-18fcf3704acc)


### Liquid Manipulation: Category-level pose & dimension detector with pouring action optimizer

![framework](https://github.com/user-attachments/assets/868e1753-9663-4b4b-a5de-f5a4f0b05e98)

{% include youtube.html id="oPvfIooH5HU" %}  


### Forked PDDLstream Task and Motion planning (TAMP) online replanning.
(simulation forked from Caelan Reed Garrett LTAMP)
{% include youtube.html id="vuVLHsrkUqk" %}  


### Behaviour cloning (BC) learning-based Block Pushing task 
{% include youtube.html id="dakwQ2TanH8" %}  


### [Vision-learning based Soft Actuator Grasping Control within Digital Twin](https://arxiv.org/pdf/2410.14934) 
![OpenCV based Soft Actuator Grasping Control in Unity3D](images/icac.png)



###  [Simple and Novel Motion Planning Digital Twin Framework for Industrial Robot](https://arxiv.org/pdf/2410.14928)
{% include youtube.html id="f_BEMbMvFso" %}  



###  Autonomous tracking DIY Mars Rover Design
{% include youtube.html id="Woah0fB-n9M" %}  

### Tactile Sensing achieving human motion monitoring
![Tactile Sensing](images/Soft_electronics_fit.png)


###  ROS SLAM AGV navigation with Adaptive Monte Carlo Localization (AMCL)
{% include youtube.html id="iu8w5aNAI_8" %}  


###  [AGV with PLC control optimisation simulation](../assets/Dynamic optimisation of AGV.pdf) 

![AGV optimisation](images/Dynamic optimisation of AGV.gif)



