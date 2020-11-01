import krpc
from mpc.Vessel import Vessel
from mpc.Panel import Panel
from mpc.controllers.PID import PID
from mpc.controllers.MPC import MPC
from mpc.handlers.LogHandler import LogHandler
from mpc.handlers.TerminationHandler import TerminationHandler
from mpc.handlers.TargetHandler import TargetHandler
from datetime import datetime
import time
import pandas as pd
import numpy as np


class Controller:

    def __init__(self, saved_game_filename=None):
        self.conn = krpc.connect()
        if saved_game_filename:
            self.saved_game_filename = saved_game_filename
            self.conn.space_center.load(saved_game_filename)
        self.vessel = Vessel(self.conn)
        self.vessel.stage = 2
        # self.panel = Panel(self.conn)
        self.params = {
            # before PID dt tuning
            # 'Roll P': -0.001, 'Roll I': -0.02275, 'Roll D': -0.5,
            # 'Pitch P': 10, 'Pitch I': 1, 'Pitch D': 100,
            # 'Yaw P': -10, 'Yaw I': -1, 'Yaw D': -100,
            # after
            # 'Roll P': -0.01, 'Roll I': -0.0275, 'Roll D': -0.01,
            'Roll P': -0.0105, 'Roll I': -0.025, 'Roll D': -0.0125,
            # 'Roll P': -0.035, 'Roll I': -0.055, 'Roll D': -0.05,
            'Pitch P': 10, 'Pitch I': 0.01, 'Pitch D': 10,
            # 'Pitch P': 10, 'Pitch I': 5, 'Pitch D': 10, # works for separate
            'Yaw P': -10, 'Yaw I': -0.01, 'Yaw D': -10,
            # 'Yaw P': -10, 'Yaw I': -5, 'Yaw D': -10, # works for separate
            'horizon': 10, 'dT': 0.5, 'dt': 0.5,
            'Target Direction X': 0, 'Target Direction Y': 0, 'Target Roll': -90
        }

    def run(self, params, target_handler, termination_handler, mode='ALL', log_handler=None):
        times = []
        # get initial status
        status = self.vessel.get_status()
        # initialize controllers depending on the selected mode
        controllers = self.init_controllers(params, status, mode)
        while True:
            t1 = datetime.now()
            status = self.vessel.get_status()
            # get targets
            target_alt, target_vel = target_handler(status)
            # check for termination condition
            if termination_handler(target_alt, status):
                self.vessel.set_throttle(0)
                print("Avg total time: ", sum(times) / len(times))
                break
            # update controller parameters based on current status
            controllers = self.update_controllers(controllers, status)
            # get the new input values
            new_inputs = self.get_new_inputs(controllers, status, target_alt, target_vel,
                                             self.conn.space_center.ut)
            # set new values
            for control, value in new_inputs.items():
                self.vessel.set_control_value(control, value)
            # update panel
            # self.panel.update_panel(status)
            if log_handler:
                inputs = {}
                # prepend New to each key to differentiate from actual values
                for control, value in new_inputs.items():
                    inputs['New ' + control] = value
                log_handler.write(params, inputs, status)
            # calculate average time for cycle
            t2 = datetime.now()
            times.append((t2 - t1).total_seconds())
            # print("Avg total time: ", sum(times) / len(times))
        if log_handler:
            log_handler.save()

    def init_controllers(self, params, status, mode):
        pid_roll = PID(0, params['Roll P'], params['Roll I'], params['Roll D'],
                       status['Roll'], params['Target Roll'], self.conn.space_center.ut)
        pid_pitch = PID(0, params['Pitch P'], params['Pitch I'], params['Pitch D'],
                        status['Direction Y'], params['Target Direction Y'], self.conn.space_center.ut)
        pid_yaw = PID(0, params['Yaw P'], params['Yaw I'], params['Yaw D'],
                      status['Direction X'], params['Target Direction X'], self.conn.space_center.ut)
        throttle_mpc = MPC(self.vessel, horizon=params['horizon'], dT=params['dT'], dt=params['dt'])
        pid_controllers = {
            'Roll': pid_roll,
            'Pitch': pid_pitch,
            'Yaw': pid_yaw
        }
        mpc_controller = {
            'Throttle': throttle_mpc
        }
        if mode == 'PID':
            return pid_controllers
        elif mode == 'MPC':
            return mpc_controller
        else:
            return {**pid_controllers, **mpc_controller}

    def run_landing(self, mode):
        # get init status
        status = self.vessel.get_status()
        # init log handler
        log_filename = 'logs/results/landing_somewhere.csv'
        log_handler = LogHandler(log_filename, status)
        # set controller parameters
        self.run(self.params, TargetHandler.landing_sliding_target, TerminationHandler.landing_termination, mode,
                 log_handler)

    def run_data_gathering(self, lower_alt, upper_alt, velocity):
        # get init status
        status = self.vessel.get_status()
        # init log handler
        log_filename = 'logs/data_collection/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        log_handler = LogHandler(log_filename, status)
        # init target handler
        target_handler = TargetHandler(upper_alt, lower_alt, velocity)
        # set controller parameters
        self.params['horizon'] = 5
        self.run(self.params, target_handler.data_gather_target, TerminationHandler.fuel_termination, 'ALL', log_handler)

    def run_pid_tuning(self, param):
        p_values = np.linspace(0.01, 0.03, 5)
        i_values = np.linspace(0.01, 0.03, 5)
        d_values = np.linspace(0.01, 0.03, 5)
        for upper_alt, lower_alt, velocity, saved_game_filename in \
                [(8000, 3000, -150, '8K_Roll'),
                 (3000, 50, 0, '3K_Roll')]:
            for p in p_values:
                for i in i_values:
                    for d in d_values:
                        print("lower_alt: ", lower_alt, " | upper_alt: ", upper_alt, " | velocity: ", velocity,
                              " | p: ", p, " | i: ", i, " | d: ", d)
                        self.params[str(param) + ' P'] = -p
                        self.params[str(param) + ' I'] = -i
                        self.params[str(param) + ' D'] = -d
                        self.conn.space_center.load(saved_game_filename)
                        self.vessel = Vessel(self.conn)
                        self.vessel.stage = 2
                        # set target handler
                        target_handler = TargetHandler(upper_alt, lower_alt, velocity)
                        # get init status
                        status = self.vessel.get_status()
                        # init log handler
                        log_filename = 'logs/pid_tuning_separate/' + param + '_' + saved_game_filename + '_' + \
                                       time.strftime("%Y%m%d-%H%M%S") + '.csv'
                        log_handler = LogHandler(log_filename, status)
                        self.run(self.params, target_handler.pid_tuning_target, TerminationHandler.hard_stop, 'ALL',
                                 log_handler)

    def run_model_validation(self):
        # log_filename = 'logs/model_validation_drag_with_alt_vel/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        df = pd.DataFrame(columns=['horizon', 'dt', 'Input Altitude', 'Input Velocity', 'Model Altitude', 'Model Velocity',
                                   'Model Thrust', 'Model Drag', 'Model Mass', 'Model Weight', 'Model Acceleration',
                                   'Actual Altitude', 'Actual Velocity', 'Actual Thrust', 'Actual Drag',
                                   'Actual Mass', 'Input Throttle'])
        # horizons = [0, 5, 10, 15, 20, 1000]
        horizons = [10, 20, 50, 100, 100000]
        dts = [1]
        inputs = [(1, 1, 5), (0, 0, 10), (0, 1, 10), (0, 1, 10), (0, 0, 15), (1, 1, 5),
                  (0.5, 0.5, 20), (0, 0.5, 45), (0.1, 0.1, 5), (0, 0, 50), (0.5, 0.3, 30)]
        inputs = [(1, 1, 5), (0, 0, 10), (1, 1, 5), (0, 0, 5), (1, 1, 5), (0, 0, 5), (1, 1, 5),
                  (0, 0, 5), (1, 1, 5), (0, 0, 5), (1, 1, 4), (0, 0, 60), (0.1, 0.1, 30), (0.2, 0.2, 20)]
        for horizon in horizons:
            for dt in dts:
                self.conn.space_center.load('launch_stage')
                self.vessel = Vessel(self.conn)
                self.vessel.set_autopilot()
                if self.vessel.get_stage() == 0:
                    self.vessel.next_stage()
                throttle_mpc = MPC(self.vessel, horizon=horizon, dT=dt, dt=dt)
                print('h: ', horizon, 't: ', dt)
                data = throttle_mpc.model_validation(inputs)
                for item in data:
                    df.loc[len(df)] = [horizon, dt] + item
                df.to_csv('logs/model_validation/horizon_' + str(horizon) + "_dt_ " + str(dt) + '.csv', index=False)
                # df.to_csv('logs/model_validation_drag_with_alt_vel/horizon_' + str(horizon) + "_dt_" + str(dt) + '.csv', index=False)
                df = pd.DataFrame(
                    columns=['horizon', 'dt', 'Input Altitude', 'Input Velocity', 'Model Altitude', 'Model Velocity',
                             'Model Thrust', 'Model Drag', 'Model Mass', 'Model Weight', 'Model Acceleration',
                             'Actual Altitude', 'Actual Velocity', 'Actual Thrust', 'Actual Drag',
                             'Actual Mass', 'Input Throttle'])

    @staticmethod
    def update_controllers(controllers, status):
        if 'Roll' in controllers.keys():
            if status['Altitude'] < 15000:
                controllers['Roll'].set_k_p(-0.03)
                controllers['Roll'].set_k_i(-0.045)
                controllers['Roll'].set_k_d(-0.02)
            elif status['Altitude'] < 8000:
                controllers['Roll'].set_k_p(-0.045)
                controllers['Roll'].set_k_i(-0.03)
                controllers['Roll'].set_k_d(-0.055)
            elif status['Altitude'] < 3000:
                controllers['Roll'].set_k_p(-0.035)
                controllers['Roll'].set_k_i(-0.055)
                controllers['Roll'].set_k_d(-0.05)
        if 'Pitch' in controllers.keys():
            if abs(status['Vertical Velocity']) > 3:
                controllers['Pitch'].set_target_val(status['Retrograde Y'])
            else:
                controllers['Pitch'].set_target_val(0)
        if 'Yaw' in controllers.keys():
            if abs(status['Vertical Velocity']) > 3:
                controllers['Yaw'].set_target_val(status['Retrograde X'])
            else:
                controllers['Yaw'].set_target_val(0)
        # print(f"Setting new target pitch:  {controllers['Pitch'].get_target_val()}")
        # print(f"Setting new target yaw: {controllers['Yaw'].get_target_val()}")
        return controllers

    @staticmethod
    def get_new_inputs(controllers, status, target_alt, target_vel, curr_time):
        controller_input = {
            'Pitch': (status['Direction Y'], curr_time),
            'Yaw': (status['Direction X'], curr_time),
            'Roll': (status['Roll'], curr_time),
            'Throttle': ([status['Altitude'], status['Vertical Velocity']], [target_alt, target_vel])
        }
        new_inputs = {}
        for key, val in controllers.items():
            new_inputs[key] = val.get_val(*controller_input[key])
        return new_inputs
