import numpy as np
from matplotlib import pyplot as plt

from .UAVSimulator import *

import tyro
from typing import Literal, Optional
from dataclasses import dataclass
from pathlib import Path

@dataclass
class Config:
    # 示例轨迹类型，trajectory 为正弦轨迹跟踪；upside_down 为从颠倒状态恢复
    example: Literal["trajectory", "upside_down"] = "trajectory"
    # 输出图片保存路径
    save_root: Optional[str]= None
    # 仿真间隔
    dt: float = 0.0001

# 轨迹跟踪
if __name__ == "__main__":

    config = tyro.cli(Config)

    if config.example == "trajectory":
        m = UAVModel()
        p = UAVControlParameter()
        t = UAVTrajectory(xd = lambda t: np.array([0.4 * t, 0.4 * np.sin(np.pi * t), 0.6 * np.cos(np.pi * t)]), 
                        b1d = lambda t: np.array([np.cos(np.pi * t), np.sin(np.pi * t), 0]), 
                        vd = lambda t: np.array([0.4, np.pi * 0.4 * np.cos(np.pi * t), -np.pi * 0.6 * np.sin(np.pi * t)]),
                        ad = lambda t: np.array([0, -np.pi ** 2 * 0.4 * np.sin(np.pi * t), -np.pi ** 2 * 0.6 * np.cos(np.pi * t)]),
            )

        s = UAVSimulator(p, m, t)
        
        x0 = np.array([0, 0, 0], dtype = np.float64)
        v0 = np.array([0, 0, 0], dtype = np.float64)
        Omega0 = np.array([0, 0, 0], dtype = np.float64)
        R0 = np.identity(3)

        plot_start = 50
    else:
        m = UAVModel()
        p = UAVControlParameter()
        t = UAVTrajectory(
            xd = lambda t: np.array([0, 0, 0]), 
            b1d = lambda t: np.array([1, 0, 0]),
            vd = lambda t: np.array([0, 0, 0]), 
            ad = lambda t: np.array([0, 0, 0]), 
        )

        s = UAVSimulator(p, m, t)
        
        x0 = np.array([0, 0, 0], dtype = np.float64)
        v0 = np.array([0, 0, 0], dtype = np.float64)
        Omega0 = np.array([0, 0, 0], dtype = np.float64)

        R0 = np.array([
            [1, 0, 0],
            [0, -0.9995, -0.0314],
            [0, 0.0314, -0.9995]
        ])

        t0 = np.array([0, 0])

        plot_start = 0

    s.simulate(x0, v0, Omega0, R0, 4, config.dt)
    s.post_process()

    if config.save_root is None:
        save_rot_error = None
        save_pos = None
        save_omega = None
        save_f = None
    else:
        save_root = Path(config.save_root)
        save_rot_error = save_root.joinpath("rot_error.png")
        save_pos = save_root.joinpath("pos.png")
        save_omega = save_root.joinpath("omega.png")
        save_f = save_root.joinpath("f.png")

    s.draw_rot_error(plt.figure(), save_rot_error)
    s.draw_pos(plt.figure(), save_pos)
    s.draw_omega(plt.figure(), plot_start, save_omega)
    s.draw_f(plt.figure(), plot_start, save_f)

    plt.show()

# # 从翻滚状态恢复
# if __name__ == "__main__":
#     m = UAVModel()
#     p = UAVControlParameter()
#     t = UAVTrajectory(
#         xd = lambda t: np.array([0, 0, 0]), 
#         b1d = lambda t: np.array([1, 0, 0]),
#         vd = lambda t: np.array([0, 0, 0]), 
#         ad = lambda t: np.array([0, 0, 0]), 
#     )

#     s = UAVSimulator(p, m, t)
    
#     x0 = np.array([0, 0, 0], dtype = np.float64)
#     v0 = np.array([0, 0, 0], dtype = np.float64)
#     Omega0 = np.array([0, 0, 0], dtype = np.float64)

#     R0 = np.array([
#         [1, 0, 0],
#         [0, -0.9995, -0.0314],
#         [0, 0.0314, -0.9995]
#     ])

#     t0 = np.array([0, 0])

#     s.simulate(x0, v0, Omega0, R0, 6, 0.01)
#     s.post_process()

#     s.draw_rot_error(plt.figure())
#     s.draw_pos(plt.figure())
#     s.draw_omega(plt.figure(), 0)
#     s.draw_f(plt.figure(), 0)

#     plt.show()
