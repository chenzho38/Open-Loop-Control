import threading
import time

class MotorController:
    def __init__(self):
        # 电机参数
        self.f_base = 100.0  # 基频 (Hz)
        self.V_base = 36.0  # 基频对应电压 (V)
        self.Ts = 0.00025  # 中断周期 0.25ms (4kHz)
        self.fs = 1 / self.Ts  # 载波频率

        # 运行变量
        self.theta = 0.0  # 当前角度
        self.f_set = 0.0  # 设定频率
        self.mi = 0.0  # 调制比
        self.lock = threading.Lock()

        # PWM输出缓存
        self.Ta = 0.0
        self.Tb = 0.0
        self.Tc = 0.0

    def update_frequency(self, new_f):
        """更新频率设定值（每100ms调用）"""
        with self.lock:
            self.f_set = max(0.0, min(new_f, self.f_base))  # 限制频率范围

    def calculate_parameters(self):
        """计算控制参数"""
        with self.lock:
            f = self.f_set

        if f < 0.1:  # 死区处理
            return 0.0, 0.0

        # V/F控制计算
        V = (self.V_base / self.f_base) * f
        mi = V / (self.V_base / 0.612)  # 0.612为SVPWM的电压利用率

        return f, mi

    def interrupt_handler(self):
        """定时中断服务程序"""
        # 计算控制参数
        f, self.mi = self.calculate_parameters()

        # 角度更新
        if f > 0.1:
            delta_theta = 360 * f * self.Ts
            self.theta = (self.theta + delta_theta) % 360

        # 生成SVPWM
        self.Ta, self.Tb, self.Tc = svpwm(self.theta, self.mi)

        def get_sector(Valpha, Vbeta):

         self.Valpha = [V_base /np.sqrt(3) ] * np.sin(theta -theta_down)
         self.Vbeta = [V_base / np.sqrt(3)] * np.cos(theta - theta_down)

         def get_sector(Valpha, Vbeta):
             angle = np.arctan2(Vbeta, Valpha)
             if angle < 0:
                 angle += 2 * np.pi
                 sector = int(angle // (np.pi / 3)) + 1
                 return sector


        if sector == 1:
           thheta_down = np.pi *0
           theta_up = np.pi/3


        elif sector == 2:
           theta_down =np.pi/3
           theta_up =np.pi *2/3

        elif sector == 3:
           thheta_down =np.pi*2/3
           theta_up= np.pi

        elif sector == 4:
           thheta_down =np.pi
           theta_up =np.pi *4/3

        elif sector == 5:
           theta_down =np.pi* 4/3
           theta_up=np.pi * 5/3

        elif sector == 6:
           thheta_down = np.pi* 5/3
           theay_up_=np.pi*2
           return theta_down , theta_up


def calculate_times(Valpha, Vbeta, sector):
            T1 = (np.sqrt(3) * Ts / Vdc) * (Valpha * np.sin(theta_up) - Vbeta *
                                            np.cos(theta_up))
            T2 = (np.sqrt(3) * Ts / Vdc) * (-Valpha * np.sin(theta_down) + Vbeta *
                                            np.cos(theta_down))
            T0 = Ts - T1 - T2
            return T1, T2, T0

def generate_pwm(T1, T2, T0, sector) :
            if sector == 1:
                Ta = T1 + T2 + T0 / 2
                Tb = T2 + T0 / 2
                Tc = T0 / 2
            elif sector == 2:
                Tb = T2 + T0 / 2
                Ta = T1 + T0 / 2
                Tb = T1 + T2 + T0 / 2
                Tc = T0 / 2
            elif sector == 3:
                Ta = T0 / 2
                Tb = T1 + T2 + T0 / 2
                Tc = T2 + T0 / 2
            elif sector == 4:
                Ta = T0 / 2
                Tb = T1 + T0 / 2
                Tc = T1 + T2 + T0 / 2
            elif sector == 5:
                Ta = T2 + T0 / 2git
                Tb = T0 / 2
                Tc = T1 + T2 + T0 / 2
            elif sector == 6:
                Ta = T1 + T2 + T0 / 2
                Tb = T0 / 2
                Tc = T1 + T0 / 2
                return Ta, Tb, Tc

        # 这里添加实际硬件输出代码
        # self.update_hardware_pwm(self.Ta, self.Tb, self.Tc)

    def run_control_loop(self):
        """主控制循环"""
        try:
            while True:
                start_time = time.time()
                self.interrupt_handler()

                # 精确计时（考虑实际执行时间）
                elapsed = time.time() - start_time
                sleep_time = self.Ts - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            pass


# 使用示例
if __name__ == "__main__":
    controller = MotorController()

    # 启动控制线程
    control_thread = threading.Thread(target=controller.run_control_loop)
    control_thread.daemon = True
    control_thread.start()

    # 模拟电动车手柄输入（实际应替换为硬件输入读取）
    try:
        while True:
            # 这里添加实际频率输入读取代码（例如通过ADC）
            # 示例：每100ms读取一次频率设定值
            input_f = float(input("请输入设定频率 (0-100Hz): "))
            controller.update_frequency(input_f)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass