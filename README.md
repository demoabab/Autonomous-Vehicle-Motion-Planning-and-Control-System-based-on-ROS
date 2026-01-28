# 🏎️ F1TENTH Autonomous Navigation System
### Robust Navigation with Kinematic Hybrid A* & Sampling-based MPC

![ROS](https://img.shields.io/badge/ROS-Melodic%2FNoetic-blue?style=flat-square&logo=ros)
![Language](https://img.shields.io/badge/Language-C%2B%2B14-orange?style=flat-square&logo=c%2B%2B)
![Platform](https://img.shields.io/badge/Platform-F1TENTH_Simulator-red?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

> **A full-stack autonomous driving solution featuring kinematic-aware path planning and high-frequency predictive control.**

---

## 📖 简介 (Introduction)

本项目针对 **F1TENTH** 无人赛车平台开发，旨在解决复杂受限空间下的自主导航与高速循迹问题。

系统摒弃了传统的 RRT + PID/Pure Pursuit 方案，采用 **Hybrid A*** 进行符合阿克曼转向几何的全局路径规划，并设计了基于 **MPPI (Model Predictive Path Integral)** 思想的采样型模型预测控制器。

通过引入 **Pure Pursuit 引导采样 (Warm Start)** 和 **动态速度规划 (Velocity Profiling)**，本系统能够在保证避障安全的前提下，实现类似赛车手的“直道加速、弯道重刹”驾驶风格，并将终点停车误差控制在 **10cm** 以内。

---

## ✨ 核心特性 (Key Features)

### 🧠 1. 运动学路径规划 (Kinematic Planning)
* **算法核心**：基于 **Hybrid A*** (混合 A 星) 算法。
* **双向搜索**：支持 **Bi-directional Search**，能够规划出包含倒车（Reversing）动作的路径，完美解决死胡同掉头（3-Point Turn）和侧方停车难题。
* **几何约束**：在 $(x, y, \theta)$ 三维状态空间搜索，确保路径曲率连续且符合车辆最小转弯半径。

### ⚡ 2. 高性能预测控制 (Sampling-based MPC)
* **MPPI Lite**：基于 GPU/CPU 并行的蒙特卡洛采样 MPC，实时推演未来 **1.5s** 的车辆状态。
* **引导采样 (Guided Sampling)**：创新性地利用 Pure Pursuit 的计算结果作为高斯分布的均值进行采样 (Warm Start)，解决了随机采样在急弯处效率低的问题。
* **多重代价函数**：
  * `Collision Cost`: 基于栅格地图膨胀的硬约束避障。
  * `Tracking Cost`: 紧密跟随全局路径。
  * `Steer Cost`: 抑制控制量高频抖动。

### 🚀 3. 动态博弈策略 (Dynamic Maneuvers)
* **智能换挡**：自动识别路径方向，在 **Drive (D档)** 和 **Reverse (R档)** 之间无缝切换。
* **动态变速**：基于实时转向角的 **Velocity Profiling** 策略——“弯道越急，车速越慢”。
* **精准停车**：线性末端速度控制，消除超调震荡。

---

## 🏗️ 系统架构 (System Architecture)

```mermaid
graph LR
    A[User Goal / RViz] -->|Target Pose| B(Hybrid A* Planner)
    C[Occupancy Grid Map] --> B
    C --> D(MPC Controller)
    B -->|Global Path| D
    E[Odometry] --> B
    E --> D
    D -->|Ackermann Drive| F[F1TENTH Car]
    
    style B fill:#f9f,stroke:#333,stroke-width:2px
    style D fill:#bbf,stroke:#333,stroke-width:2px
