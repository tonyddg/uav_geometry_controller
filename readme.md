# UAV geometry controller
论文 Geometric Tracking Control of a Quadrotor UAV on SE(3) 中几何控制器的实现与仿真复现

## 主要文件说明
* `main.py` 论文中两种示例的复现
* `UAVSimulator.py` 控制器与数值积分仿真相关函数
    * 使用控制器时, 可以去除仿真与结果绘图函数

## 运行环境
* Python 3.12.0
* Numpy 1.26.3
* matplotlib 3.8.2
