from dataclasses import dataclass
import numpy as np
from typing import Callable

@dataclass
class UAVTrajectory:
    '''
    给定控制轨迹
    * xd: Callable[[float], np.ndarray]  
    位置轨迹
    * b1d: Callable[[float], np.ndarray]  
    朝向轨迹
    * vd: Callable[[float], np.ndarray] | None = None  
    给定速度 (传入 None 时通过差分确定)
    * ad: Callable[[float], np.ndarray] | None = None  
    给定加速度 (传入 None 时通过差分确定)
    '''

    xd: Callable[[float], np.ndarray]
    b1d: Callable[[float], np.ndarray]

    vd: Callable[[float], np.ndarray] | None = None
    ad: Callable[[float], np.ndarray] | None = None

    # todo: IVP 中数值差分没有意义
    def get_vd(self, t: float, t_last: float):
        if self.vd != None:
            return self.vd(t)
        else:
            if t_last <= 0:
                return np.zeros(3)
            else:
                return (self.xd(t) - self.xd(t_last)) / (t - t_last)

    def get_ad(self, t: float, t_last: float, t_ll: float):
        if self.ad != None:
            return self.ad(t)
        else:
            if t_last <= 0 or t_ll <= 0:
                return np.zeros(3)
            else:
                return (self.get_vd(t, t_last) - self.get_vd(t_last, t_ll)) / (t - t_last)
