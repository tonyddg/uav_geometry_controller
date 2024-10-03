'''
实用数学函数
'''
import numpy as np

# 重力加速度
g = np.array([0, 0, 9.8])

# 实用函数
def So3ToVec(R) -> np.ndarray:
    '''
    so3 (反对称矩阵) 转换为转轴矢量 (行向量)
    '''
    return np.array([R[2, 1], R[0, 2], R[1, 0]])

def VecToSo3(so3) -> np.ndarray:
    '''
    转轴矢量 (行向量) 转换为 so3 (反对称矩阵)
    '''
    return np.array([
        [0, -so3[2], so3[1]],
        [so3[2], 0, -so3[0]],
        [-so3[1], so3[0], 0]
    ])

def NormalizeOrientMatrix(mat) -> np.ndarray:
    '''
    通过奇异值分解标准化旋转矩阵
    '''
    res = np.linalg.svd(mat, compute_uv = True)
    return res.U @ res.Vh

# 误差函数

def ex(x, xd) -> np.ndarray:
    '''
    位置误差
    '''
    return x - xd

def ev(v, vd) -> np.ndarray:
    '''
    速度误差
    '''
    return v - vd

def eR(R, Rd) -> np.ndarray:
    '''
    姿态误差
    '''
    return So3ToVec(Rd.T @ R - R.T @ Rd) * 0.5

def rotErrorFun(R, Rd) -> float:
    '''
    姿态误差衡量 (Psi 函数)
    '''
    return np.trace(np.identity(3) - Rd.T @ R) * 0.5

def eOmega(R, Rd, Omega, Omega_d) -> np.ndarray:
    '''
    角速度误差
    '''
    return Omega - R.T @ Rd @ Omega_d

# 求导
def OmegaDerive(Omega: np.ndarray, Omega_last: np.ndarray, t: float, t_last: float):
    if t_last <= 0:
        return np.zeros(3)
    else:
        return (Omega - Omega_last) / (t - t_last)

def RDerive(R: np.ndarray, R_last: np.ndarray, t: float, t_last: float):
    if t_last <= 0:
        return np.zeros((3, 3))
    else:
        return (R - R_last) / (t - t_last)

def getOmega(R: np.ndarray, RD: np.ndarray):
    return So3ToVec(R.T @ RD)
