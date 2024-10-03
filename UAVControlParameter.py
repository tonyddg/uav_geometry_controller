from dataclasses import dataclass, field
@dataclass

class UAVControlParameter:
    '''
    UAV 控制器参数, 默认值来自论文 (给定时需要乘上质量参数 m)
    '''
    kx: float = 69.44
    kv: float = 24.304
    kR: float = 8.81
    kOmega: float = 2.54
