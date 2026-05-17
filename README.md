# STM32H503_LIS2DW12
STM32H503_LIS2DW12

# Overview
- **Name**: LSM6DSV80X-V1.0
- **MCU**: STM32H503CBT6
- **IDE**: STM32CUBEMX+KEIL


# Buy Link
[https://shop192352884.taobao.com/](https://shop192352884.taobao.com/)




# Image
<img width="934" height="930" alt="image" src="https://github.com/user-attachments/assets/d1287dca-25bb-4771-970b-a8d674c0f7b1" />

<img width="1236" height="1760" alt="image" src="https://github.com/user-attachments/assets/64a6ea3e-20dc-44b1-991f-e9c258301808" />

<img width="1240" height="1759" alt="image" src="https://github.com/user-attachments/assets/35875bcf-3311-4953-bc85-0cb5695ebe23" />

<img width="1236" height="1760" alt="image" src="https://github.com/user-attachments/assets/2987474f-b414-4210-b125-615761cc18f6" />

<img width="1236" height="1760" alt="image" src="https://github.com/user-attachments/assets/a5808ec1-9c3e-4f95-8f50-ebb92b99107c" />


# Contact Information

- **Name**: Billy
- **交流群**: 925643491
- **Email**: a845656974@outlook.com
- **Phone**: +86 15622736378
- **CSDN Blog**: [Blog](https://blog.csdn.net/qq_24312945)
- **Video**: [Video](https://space.bilibili.com/26152390)



# Project Introduction
- **STM32H503_LIS2DW12_Project1**:加速度计LIS2DW12开发(1)----轮询获取加速度数据
- **CSDN Blog**:[https://blog.csdn.net/qq_24312945/article/details/152116334](https://blog.csdn.net/qq_24312945/article/details/152116334)

本文将介绍如何驱动和利用LIS2DW12传感器，实现精确的运动感应功能。
LIS2DW12是一款高性能、超低功耗的三轴线性加速度计，属于“femto”系列，利用了成熟的微机械加速度计制造工艺。这个传感器提供可选择的全量程±2g/±4g/±8g/±16g，能够以1.6 Hz至1600 Hz的数据输出率测量加速度。它包含了一个32级的先进先出（FIFO）缓冲区，用于存储数据，以减少主处理器的干预需求。
此外，LIS2DW12具备自测功能，可在最终应用中验证传感器功能，并集成了一个处理运动和加速度检测的内部引擎。这包括自由落体、唤醒、敲击识别、活动/静止监测、静止/运动检测、纵向/横向检测以及6D/4D定向等功能。

This article will introduce how to drive and use the LIS2DW12 sensor to achieve accurate motion-sensing functions.

The LIS2DW12 is a high-performance, ultra-low-power three-axis linear accelerometer belonging to the “femto” family, based on ST’s mature micromachined accelerometer manufacturing process. This sensor provides selectable full-scale ranges of ±2g, ±4g, ±8g, and ±16g, and can measure acceleration with output data rates from 1.6 Hz to 1600 Hz. It also includes a 32-level first-in, first-out (FIFO) buffer for data storage, helping to reduce the need for intervention from the main processor.

In addition, the LIS2DW12 features a self-test function, which allows sensor functionality to be verified in the final application. It also integrates an internal engine for motion and acceleration detection, including free-fall detection, wake-up detection, tap recognition, activity/inactivity monitoring, stationary/motion detection, portrait/landscape detection, and 6D/4D orientation detection.

- **STM32H503_LIS2DW12_Project2**:加速度计LIS2DW12开发(2)----基于中断信号获取加速度数据
- **CSDN Blog**:[https://blog.csdn.net/qq_24312945/article/details/153848101](https://blog.csdn.net/qq_24312945/article/details/153848101)

本文将介绍实时获取和处理加速度数据。程序的核心流程包括初始化硬件接口、配置加速度计的参数，以及通过轮询检查中断信号来不断读取加速度数据。

This article will introduce how to acquire and process acceleration data in real time. The core workflow of the program includes initializing the hardware interface, configuring the accelerometer parameters, and continuously reading acceleration data by polling and checking the interrupt signal.
- **STM32H503_LIS2DW12_Project3**:加速度计LIS2DW12开发(3)----检测活动和静止状态
- **CSDN Blog**:[https://blog.csdn.net/qq_24312945/article/details/156725404](https://blog.csdn.net/qq_24312945/article/details/156725404)

检测活动和静止状态主要用途是在嵌入式应用中实时监控加速度计的活动状态，例如在可穿戴设备、智能手机或安全系统中检测用户的动作或设备的位置变化。通过设置不同的阈值和时长，可以精确地确定何时设备处于静止状态，何时发生了活动，从而触发相应的操作或警报。
Activity and inactivity detection is mainly used to monitor the activity status of an accelerometer in real time in embedded applications, such as detecting user motion or device position changes in wearable devices, smartphones, or security systems.

By setting different thresholds and durations, it is possible to accurately determine when the device is in a stationary state and when activity occurs, thereby triggering corresponding actions or alarms.

- **STM32H503_LIS2DW12_Project4**:加速度计LIS2DW12开发(4)----测量倾斜度
- **CSDN Blog**:[https://blog.csdn.net/qq_24312945/article/details/158470857](https://blog.csdn.net/qq_24312945/article/details/158470857)

本文将介绍如何驱动和利用LIS2DW12三轴加速度计的倾斜检测理论和倾斜角测量方法。一般来说，这里描述的程序也可以应用于三轴模拟或数字加速度计，这取决于它们各自的规格。

This article will introduce how to drive and use the LIS2DW12 three-axis accelerometer for tilt detection theory and tilt angle measurement methods. In general, the procedure described here can also be applied to three-axis analog or digital accelerometers, depending on their respective specifications.


















