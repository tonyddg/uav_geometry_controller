from dataclasses import dataclass, field
import numpy as np

@dataclass
class UAVModel:
    '''
    UAV 模型参数, 默认值来自论文
    * 惯性矩阵 (kg m2)  
    J: np.ndarray
    * 质量 (kg)  
    m: float
    * 螺旋桨到质心的距离 (m)  
    d: float
    * 推力转矩系数 (m)  
    ctf: float
    * 总推力, 力矩到螺旋桨推理的转换矩阵 
    FM2f (自动确定)
    '''
    J: np.ndarray = field(default_factory = lambda: np.diag([0.0820, 0.0845, 0.1377]))
    m: float = 4.34
    d: float = 0.315
    ctf: float = 8.004e-4

    FM2f: np.ndarray = field(init = False)
    def __post_init__(self):
        self.FM2f = np.linalg.inv(np.array([
            [1, 1, 1, 1],
            [0, -self.d, 0, self.d],
            [self.d, 0, -self.d, 0],
            [-self.ctf, self.ctf, -self.ctf, self.ctf]
        ]))
