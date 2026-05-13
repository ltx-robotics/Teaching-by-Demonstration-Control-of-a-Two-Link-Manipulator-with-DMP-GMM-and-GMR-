# Learning-Based Control for Two-Link Robot Manipulator

## 中文

### 项目简介

本项目基于示教轨迹（Demonstration Trajectory），结合 DMP（Dynamic Movement Primitive）、GMM（Gaussian Mixture Model）与 GMR（Gaussian Mixture Regression）方法，对二连杆机器人进行了运动学习与轨迹生成。

在学习得到的轨迹基础上，分别实现了：

- 位置控制（Position Control）
- 力矩控制（Torque Control）

并对两种控制方式的控制性能与系统响应进行了比较分析。

### 项目内容

- 基于示教数据进行轨迹学习
- 使用 GMM 对轨迹数据进行建模
- 使用 GMR 对轨迹进行回归与重构
- 使用 DMP 实现运动生成与轨迹泛化
- 对二连杆机器人进行位置控制
- 对二连杆机器人进行力矩控制
- 分析不同控制方法下的系统性能与稳定性

### 技术内容

- Dynamic Movement Primitive (DMP)
- Gaussian Mixture Model (GMM)
- Gaussian Mixture Regression (GMR)
- MATLAB / Simulink
- Two-Link Manipulator Control
- Position Control
- Torque Control

---

## English

### Project Description

This project applies demonstration-based learning methods, including Dynamic Movement Primitive (DMP), Gaussian Mixture Model (GMM), and Gaussian Mixture Regression (GMR), to trajectory learning and motion generation for a two-link robot manipulator.

Based on the learned demonstration trajectories, the project implements:

- Position Control
- Torque Control

A comparative analysis was conducted to evaluate the control performance and system response of both methods.

### Project Details

- Learning trajectories from demonstration data
- Modeling trajectory data using GMM
- Reconstructing trajectories using GMR
- Motion generation and trajectory generalization using DMP
- Position control for a two-link manipulator
- Torque control for a two-link manipulator
- Comparative analysis of control performance and stability

### Technical Contents

- Dynamic Movement Primitive (DMP)
- Gaussian Mixture Model (GMM)
- Gaussian Mixture Regression (GMR)
- MATLAB / Simulink
- Two-Link Manipulator Control
- Position Control
- Torque Control
