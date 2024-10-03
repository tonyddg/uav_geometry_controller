from UAVControlParameter import *
from UAVModel import *
from UAVSimulator import *
from UAVTrajectory import *

from math_utility import *

from matplotlib.figure import Figure

class UAVSimulator:
    def __init__(
        self, 
        param: UAVControlParameter,
        model: UAVModel,
        traj: UAVTrajectory
    ) -> None:
        self.param = param
        self.model = model
        self.traj = traj

        # 辅助仿真信息
        self.Rd_last = None
        self.Omega_d_last = None
        self.t_last = np.zeros(2)

        # 仿真结果记录
        self.r_error_record = []
        self.pos_record = []
        self.omega_record = []
        self.f_record = []
        self.t_record = []

        pass

    def controller(self, t, x, v, Omega, R, is_record = False):
        '''
        几何控制器
        '''

        ### 位置控制器部分
        xd = self.traj.xd(t)
        vd = self.traj.get_vd(t, self.t_last[0])
        ad = self.traj.get_ad(t, self.t_last[0], self.t_last[1])

        ## 速度
        ex_ = ex(x, xd)
        ev_ = ev(v, vd)

        ## 控制力
        # 理论合力矢量
        fd = - (- self.param.kx * ex_ - self.param.kv * ev_ - self.model.m * g + self.model.m * ad)
        # 控制合力大小
        force = fd @ R.T[2]

        ## 姿态控制部分
        b3d = fd / np.linalg.norm(fd)

        b2d = np.cross(b3d, self.traj.b1d(t))
        b2d = b2d / np.linalg.norm(b2d)

        b1d = np.cross(b2d, b3d)
        Rd = np.vstack([b1d, b2d, b3d]).T

        ## 误差计算
        eR_ = eR(R, Rd)

        if type(self.Rd_last) == np.ndarray and type(self.Omega_d_last) == np.ndarray:
            Rd_dot = RDerive(Rd, self.Rd_last, t, self.t_last[0])
            Omega_d = getOmega(Rd, Rd_dot)
            dOmega_d = OmegaDerive(Omega_d, self.Omega_d_last, t, self.t_last[0])
        else:
            Omega_d = np.zeros(3)
            dOmega_d = np.zeros(3)

        eOmega_ = eOmega(R, Rd, Omega, Omega_d)

        ## 控制力矩
        M_part1 = R.T @ Rd
        M_part2 = VecToSo3(Omega) @ M_part1 @ Omega_d
        M_part3 = M_part1 @ dOmega_d
        M_part4 = self.model.J @ (M_part2 - M_part3)

        M = - self.param.kR * eR_ - self.param.kOmega * eOmega_ + np.cross(Omega, self.model.J @ Omega) - M_part4

        ### 记录数据
        if is_record:
            self.r_error_record.append(rotErrorFun(R, Rd)) # type: ignore
            self.pos_record.append(np.hstack((x, xd))) # type: ignore
            self.omega_record.append(np.hstack((Omega, Omega_d))) # type: ignore
            self.f_record.append(self.model.FM2f @ np.hstack((force, M))) # type: ignore
            self.t_record.append(t) # type: ignore

        ### 打包数据
        self.t_last[1] = self.t_last[0]
        self.t_last[0] = t

        self.Rd_last = Rd
        self.Omega_d_last = Omega_d

        return (force, M)

    def simulate(self, x0, v0, Omega0, R0, t_end, dt):
        '''
        基于系统动力模型的仿真
        '''
        x = x0
        v = v0
        Omega = Omega0
        R = R0

        for t in np.arange(dt, t_end, dt):

            force, M = self.controller(t, x, v, Omega, R, True)

            dx = v
            dv = g - force * R.T[2] / self.model.m
            dR = R @ VecToSo3(Omega)
            dOmega = np.linalg.inv(self.model.J) @ (M - np.cross(Omega, self.model.J @ Omega))

            x += dx * dt
            v += dv * dt
            Omega += dOmega * dt
            R += dR * dt

            # 标准化旋转矩阵
            R = NormalizeOrientMatrix(R)

    def post_process(self):
        '''
        记录数据后处理
        '''
        self.r_error_record = np.array(self.r_error_record)
        self.pos_record = np.array(self.pos_record)
        self.omega_record = np.array(self.omega_record)
        self.f_record = np.array(self.f_record)
        self.t_record = np.array(self.t_record)

    def draw_rot_error(self, fig: Figure, save_path = None):
        axe = fig.add_subplot(1, 1, 1)
        axe.plot(self.t_record, self.r_error_record)
        # fig.suptitle(R"姿态误差函数 $\psi$")
        fig.set_layout_engine("constrained")

        if save_path != None:
            fig.savefig(save_path)

    def draw_pos(self, fig: Figure, save_path = None):
        for i in range(3):
            axe = fig.add_subplot(3, 1, i + 1)
            axe.plot(self.t_record, self.pos_record[:, i]) # type: ignore
            axe.plot(self.t_record, self.pos_record[:, i + 3], linestyle = '--') # type: ignore
            axe.legend(["实际位置 $x$", "跟随轨迹 $x_d$"])
        # fig.suptitle(R"无人机坐标 (m)")
        fig.set_layout_engine("constrained")

        if save_path != None:
            fig.savefig(save_path)

    def draw_omega(self, fig: Figure, filter: int = 0, save_path = None):
        for i in range(3):
            axe = fig.add_subplot(3, 1, i + 1)
            axe.plot(self.t_record[filter:], self.omega_record[filter:, i]) # type: ignore
            axe.plot(self.t_record[filter:], self.omega_record[filter:, i + 3], linestyle = '--') # type: ignore
            axe.legend([R"实际角速度 $\Omega$", R"跟随角速度 $\Omega_d$"])
        # fig.suptitle(R"无人机角速度 (rad/s)")
        fig.set_layout_engine("constrained")

        if save_path != None:
            fig.savefig(save_path)

    def draw_f(self, fig: Figure, filter: int = 0, save_path = None):
        for i in range(4):
            axe = fig.add_subplot(4, 1, i + 1)
            axe.plot(self.t_record[filter:], self.f_record[filter:, i]) # type: ignore
        # fig.suptitle(R"无人机螺旋桨推力 (N)")
        fig.set_layout_engine("constrained")

        if save_path != None:
            fig.savefig(save_path)
