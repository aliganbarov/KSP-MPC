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
        self.vessel = Vessel(self.conn)
        self.panel = Panel(self.conn)
        self.error_status = {
            'Altitude Error': 0,
            'Direction X Error': 0,
            'Direction Y Error': 0,
        }
        self.panel.init_panel(self.error_status)
        self.log_filename = 'logs/general/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        init_status = self.vessel.get_status()
        self.logs = pd.DataFrame(columns=['Input Throttle'] + [x for x in init_status.keys()])

    def run(self):
        status = self.vessel.get_status()
        target_alt = 500
        target_vel = 50
        target_direction_y = 0
        target_direction_x = 0
        # pid_hover = PID(.25, -.03, .01, -0.55, status['Altitude'], target_alt)
        # pid_yaw = PID(0, -10, 5, -5, status['Direction X'], target_direction_x)
        # pid_pitch = PID(0, 10, -5, -5, status['Direction Y'], target_direction_y)
        pid_yaw = PID(0, -1, .5, -5, status['Direction X'], target_direction_x)
        pid_pitch = PID(0, 1, -.5, -5, status['Direction Y'], target_direction_y)
        if self.vessel.get_stage() == 0:
            self.vessel.next_stage()
            self.vessel.set_throttle(1)
            time.sleep(1)
        throttle_mpc = MPC(self.vessel, horizon=5, dt=1)
        times = []
        while True:
            if self.vessel.altitude() > 500:
                target_alt = 5
                self.vessel.next_stage()
            status = self.vessel.get_status()
            if status['Altitude'] < 3 and self.vessel.get_stage() == 2:
                return 0
            current_state = [status['Altitude'], status['Vertical Velocity']]
            print("Current state")
            print(current_state)
            t1 = datetime.now()
            new_throttle = throttle_mpc.get_optimal_throttle(current_state, [target_alt, target_vel])
            t2 = datetime.now()
            # new_pitch = pid_pitch.get_val(status['Direction Y'])
            # new_yaw = pid_yaw.get_val(status['Direction X'])
            print({
                'New Throttle': new_throttle,
                # 'New Yaw': new_yaw,
                # 'New Pitch': new_pitch,
            })
            times.append((t2-t1).total_seconds())
            print("Avg time: ", sum(times) / len(times))
            self.vessel.set_throttle(new_throttle)
            # self.vessel.set_pitch(new_pitch)
            # self.vessel.set_yaw(new_yaw)
            # self.write_log(new_throttle, status)
            self.display_errors(status, target_alt, target_direction_x, target_direction_y)

    def write_log(self, inp_throttle, status):
        self.logs.loc[len(self.logs)] = [inp_throttle] + [status[x] for x in status.keys()]
        self.logs.to_csv(self.log_filename, index=False)

    def display_errors(self, status, target_alt, target_direction_x, target_direction_y):
        self.error_status['Altitude Error'] = status['Altitude'] - target_alt
        self.error_status['Direction X Error'] = status['Direction X'] - target_direction_x
        self.error_status['Direction Y Error'] = status['Direction Y'] - target_direction_y
        self.panel.update_panel(self.error_status)

    def run_model_validation(self):
        log_filename = 'logs/model_validation/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        if self.vessel.get_stage() == 0:
            self.vessel.next_stage()
            self.vessel.set_throttle(1)
            time.sleep(1)
        df = pd.DataFrame(columns=['dt', 'Input Altitude', 'Input Velocity', 'Model Altitude', 'Model Velocity',
                                   'Model Thrust', 'Model Drag', 'Model Mass', 'Model Weight', 'Model Acceleration',
                                   'Actual Altitude', 'Actual Velocity', 'Actual Thrust', 'Actual Drag',
                                   'Actual Mass'])
        throttle_mpc = MPC(self.vessel, horizon=2, dt=0.1)
        dts = [0.1, 0.5, 1]
        inputs = [(0, 1, 10), (0, 0, 10), (0, 1, 10), (0, 0, 5)]
        for dt in dts:
            throttle_mpc = MPC(self.vessel, horizon=2, dt=dt)
            for inp in inputs:
                print(dt)
                print(inp)
                data = throttle_mpc.model_validation(inp[0], inp[1], inp[2])
                for item in data:
                    df.loc[len(df)] = [dt] + item
        df.to_csv(log_filename, index=False)

