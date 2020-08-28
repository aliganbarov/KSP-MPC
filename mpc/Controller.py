import krpc
from mpc.Vessel import Vessel
from mpc.Panel import Panel
from mpc.controllers.PID import PID
from mpc.controllers.MPC import MPC
from datetime import datetime
import time
import pandas as pd


class Controller:

    def __init__(self):
        self.conn = krpc.connect()
        self.conn.space_center.load('500m')
        self.vessel = Vessel(self.conn)
        self.vessel.stage = 2
        self.panel = Panel(self.conn)

    def run(self):
        status = self.vessel.get_status()
        self.panel.init_panel(status)

        # Targets
        target_direction_y = 0
        target_direction_x = 0
        target_roll = -90

        # Controllers settings
        # horizon, dT, dt = (10, 0.5, 0.1)
        horizon, dT, dt = (10, 0.5, 0.5)

        # PID gains for 20K, 200m/s velocity
        roll_p, roll_i, roll_d = (-0.14, -0.011, -0.56)
        pitch_p, pitch_i, pitch_d = (10, 1, 100)
        yaw_p, yaw_i, yaw_d = (-10, -1, -100)

        # Init controllers
        pid_roll = PID(0, roll_p, roll_i, roll_d, status['Roll'], target_roll)
        pid_pitch = PID(0, pitch_p, pitch_i, pitch_d, status['Direction Y'], target_direction_y)
        pid_yaw = PID(0, yaw_p, yaw_i, yaw_d, status['Direction X'], target_direction_x)
        throttle_mpc = MPC(self.vessel, horizon=horizon, dT=dT, dt=dt)

        times = []
        while True:
            t3 = datetime.now()
            status = self.vessel.get_status()

            target_alt, target_vel = self.sliding_target(status['Altitude'])
            if abs(target_alt - status['Altitude']) < 7 and self.vessel.get_stage() == 2:
                print("Reached Target. Altitude: " + str(status['Altitude']))
                self.vessel.set_throttle(0)
                break

            # get controls
            t1 = datetime.now()
            # new_throttle = throttle_mpc.get_optimal_throttle([status['Altitude'], status['Vertical Velocity']],[target_alt, target_vel])
            t2 = datetime.now()
            # times.append((t2-t1).total_seconds())
            # print("Avg time: ", sum(times) / len(times))
            new_pitch = pid_pitch.get_val(status['Direction Y'])
            new_yaw = pid_yaw.get_val(status['Direction X'])
            new_roll = pid_roll.get_val(status['Roll'])

            # set controls
            # self.vessel.set_throttle(new_throttle)
            self.vessel.set_pitch(new_pitch)
            self.vessel.set_yaw(new_yaw)
            self.vessel.set_roll(new_roll)

            # update panel
            # self.panel.update_panel(status)

            t4 = datetime.now()
            times.append((t4-t3).total_seconds())
            print("Avg total time: ", sum(times)/len(times))

    def run_model_validation(self):
        log_filename = 'logs/model_validation/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        df = pd.DataFrame(columns=['horizon', 'dt', 'Input Altitude', 'Input Velocity', 'Model Altitude', 'Model Velocity',
                                   'Model Thrust', 'Model Drag', 'Model Mass', 'Model Weight', 'Model Acceleration',
                                   'Actual Altitude', 'Actual Velocity', 'Actual Thrust', 'Actual Drag',
                                   'Actual Mass', 'Input Throttle'])
        horizons = [0, 5, 10, 15, 20, 1000]
        dts = [0.1, 0.2, 0.5, 1]
        inputs = [(0, 0, 10), (0, 1, 10), (0, 0, 10), (0, 1, 10), (0, 0, 10), (0, 1, 10), (0, 0, 15), (0.3, 0.3, 20)]
        for horizon in horizons:
            for dt in dts:
                self.conn.space_center.load('launch_stage')
                self.vessel = Vessel(self.conn)
                if self.vessel.get_stage() == 0:
                    self.vessel.next_stage()
                    self.vessel.set_throttle(0.5)
                    time.sleep(4)
                throttle_mpc = MPC(self.vessel, horizon=horizon, dt=dt)
                print('h: ', horizon, 't: ', dt)
                data = throttle_mpc.model_validation(inputs)
                for item in data:
                    df.loc[len(df)] = [horizon, dt] + item
        df.to_csv(log_filename, index=False)

    def run_data_gathering(self, lower_alt, upper_alt, velocity):
        if lower_alt > upper_alt:
            return None
        elif lower_alt == upper_alt:
            velocity = 0
        print("Running low: ", lower_alt, ", up: ", upper_alt, ", vel: ", velocity)
        status = self.vessel.get_status()
        logs = pd.DataFrame(columns=['Timestamp',
                                     'Input Throttle', 'horizon', 'dT', 'dt',
                                     'Input Roll', 'Roll P', 'Roll I', 'Roll D',
                                     'Input Pitch', 'Pitch P', 'Pitch I', 'Pitch D',
                                     'Input Yaw', 'Yaw P', 'Yaw I', 'Yaw D'] +
                                    [x for x in status.keys()])
        log_filename = 'logs/data_collection/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'

        # Targets
        target_direction_y = 0
        target_direction_x = 0
        target_roll = -90
        target_alt = upper_alt
        target_vel = velocity
        ascending = True

        # Controllers settings
        horizon, dT, dt = (5, 0.5, 0.5)
        roll_p, roll_i, roll_d = (-0.14, -0.011, -0.56)
        pitch_p, pitch_i, pitch_d = (10, 1, 100)
        yaw_p, yaw_i, yaw_d = (-10, -1, -100)

        # Init controllers
        pid_roll = PID(0, roll_p, roll_i, roll_d, status['Roll'], target_roll)
        pid_pitch = PID(0, pitch_p, pitch_i, pitch_d, status['Direction Y'], target_direction_y)
        pid_yaw = PID(0, yaw_p, yaw_i, yaw_d, status['Direction X'], target_direction_x)
        throttle_mpc = MPC(self.vessel, horizon=horizon, dT=dT, dt=dt)
        while True:
            status = self.vessel.get_status()
            if status['Altitude'] < upper_alt and ascending:
                target_alt = status['Altitude'] + 200
                target_vel = velocity
            else:
                ascending = False
            if status['Altitude'] > lower_alt and not ascending:
                target_alt = status['Altitude'] - 200
                target_vel = -1 * velocity
            else:
                ascending = True
            if not status['Has Fuel']:
                break
            # get controls
            new_throttle = throttle_mpc.get_optimal_throttle([status['Altitude'], status['Vertical Velocity']],[target_alt, target_vel])
            new_pitch = pid_pitch.get_val(status['Direction Y'])
            new_yaw = pid_yaw.get_val(status['Direction X'])
            new_roll = pid_roll.get_val(status['Roll'])

            # set controls
            self.vessel.set_throttle(new_throttle)
            self.vessel.set_pitch(new_pitch)
            self.vessel.set_yaw(new_yaw)
            self.vessel.set_roll(new_roll)

            # save logs
            logs.loc[len(logs)] = [time.time(),
                                   0, horizon, dT, dt,
                                   new_roll, roll_p, roll_i, roll_d,
                                   new_pitch, pitch_p, pitch_i, pitch_d,
                                   new_yaw, yaw_p, yaw_i, yaw_d] + \
                                  [status[x] for x in status.keys()]

        logs.to_csv(log_filename, index=False)

    @staticmethod
    def sliding_target(alt):
        if alt > 5000:
            target_alt = alt - 500
            target_vel = -200
        elif alt > 1000:
            target_alt = alt - 800
            target_vel = 0
        else:
            target_alt = -2
            target_vel = 0
        return target_alt, target_vel

