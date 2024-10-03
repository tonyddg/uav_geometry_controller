import numpy as np
import matplotlib.pyplot as plt

from UAVSimulator import *

# 轨迹跟踪
if __name__ == "__main__":
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

    s.simulate(x0, v0, Omega0, R0, 4, 0.001)
    s.post_process()

    s.draw_rot_error(plt.figure())
    s.draw_pos(plt.figure())
    s.draw_omega(plt.figure(), 50)
    s.draw_f(plt.figure(), 50)

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
