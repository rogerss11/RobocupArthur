import time


class ULog:
    def setup(self):
        self.f = open("logfile.csv", "wt")
        self.f.write(
            "time,state,x,y,h,frame_id,line_pos,eKp,turn_rate,valid_cnt,tripA,tripAh,tripB,tripBh,ir_L,ir_R,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n"
        )
        self.f.flush()

    def write(self, state=-1):
        from spose import pose
        from scam import cam
        from srobot import robot
        from spose import pose as spose
        from sir import ir
        from simu import imu
        from sedge import edge

        self.f.write(
            f"{time.time():.2f},{state},{pose.pose[0]:.3f},{pose.pose[1]:.3f},{pose.pose[2]:.3f},"
        )
        self.f.write(
            f"{cam.cnt},{edge.position:.3f},{edge.u:.3f},{spose.turnrate():.3f},{edge.lineValidCnt},"
        )
        self.f.write(
            f"{spose.tripA:.3f},{spose.tripAh:.3f},{spose.tripB:.3f},{spose.tripBh:.3f},"
        )
        self.f.write(f"{ir.ir[0]:.2f},{ir.ir[1]:.2f},")
        self.f.write(f"{imu.acc[0]:.3f},{imu.acc[1]:.3f},{imu.acc[2]:.3f},")
        self.f.write(f"{imu.gyro[0]:.3f},{imu.gyro[1]:.3f},{imu.gyro[2]:.3f}\n")
        self.f.flush()

    def writeRemark(self, txt):
        self.f.write(f"# remark: {txt}\n")
        self.f.flush()

    def writeDataString(self, txt):
        self.f.write(f"# data: {txt}\n")
        self.f.flush()

    def terminate(self):
        self.f.close()


# create a global instance
flog = ULog()
