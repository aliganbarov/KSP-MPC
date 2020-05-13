import numpy as np
from mpc.Settings import Settings
from scipy.optimize import minimize

class MPC:
    def __init__(self, vessel, horizon=5):
        self.vessel = vessel
        self.horizon = horizon
        self.m = self.vessel.get_mass()
        self.available_thrust = self.vessel.get_available_thrust()
        print("available thrust")
        print(self.available_thrust)
        self.dt = 1

    def model(self, prev_state, throttle):
        y_t = prev_state[0]
        v_t = prev_state[1]
        # TODO: compute thrust
        T = self.vessel.get_available_thrust() * throttle
        # TODO: compute drag
        D = 0.35 * T
        # compute weight
        W = Settings.G * self.m * Settings.M / (Settings.R + y_t) ** 2
        # compute acceleration ma = (T - D - W)
        a_t = 1/self.m * (T - D - W)
        # compute the vertical speed
        v_t_1 = v_t + a_t * self.dt
        # compute new altitude
        y_t_1 = y_t + v_t * self.dt

        # return new state
        return [y_t_1, v_t_1]

    def cost(self, u, init_state, target_state):
        state = init_state
        cost = 0
        for i in range(self.horizon):
            state = self.model(state, u[i])
            cost += np.square(target_state[0] - state[0])
            cost += np.abs(target_state[1] - state[1])
        return cost

    def get_optimal_throttle(self, current_state, target_state):
        u = np.zeros(self.horizon)
        bounds = [[0, 1] for i in range(self.horizon)]
        optimal_throttle = minimize(self.cost, u, (current_state, target_state),
                                    method='SLSQP',
                                    bounds=bounds,
                                    tol=1e-5)
        return optimal_throttle.x[0]
