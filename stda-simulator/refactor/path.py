import numpy as np
import casadi

class Path:
    def __init__(self, simulator):
        self.sim = simulator

        self.yaw_desireds = None
        self.last_plan_time = None
        self.replan_interval = 50 # Replan every replan_interval seconds. Must be less than or equal to planning horizon.
        self.planning_horizon = 50 # Plan planning_horizon seconds into the future

        self.plan = None

    
    def yaw(self, time):
        if self.yaw_desireds is None:
            self.plan, u = self.plan_path()
            self.yaw_desireds = self.plan[2, :]
            self.last_plan_time = time
        
        
        if self.last_plan_time + self.replan_interval < time:
            print('replanning at time: ', time)
            self.plan, u = self.plan_path()
            print('replanned')
            self.yaw_desireds = self.plan[2, :]
            self.last_plan_time = time

        index = int((time - self.last_plan_time) / self.sim.stepsize)
        return self.yaw_desireds[index]

    def pos(self, time):
        index = int((time - self.last_plan_time) / self.sim.stepsize)
        return self.plan[:2, index]
        
        # if time < 25:
        #     return 0
        # elif time < 50:
        #     return np.pi/4
        # elif time < 75:
        #     return np.pi/2
        # else:
        #     return 7.25*np.pi/8

        # elif time < 90:
        #     return np.pi/4
        # # elif time < 100:
        # #     return np.pi/4
        # else:
        #     return 0
        # return 0
    
    def plan_path(self):

        dt = self.sim.stepsize
        n_steps = int(self.planning_horizon/dt)
        q_start = np.array([self.sim.boat.pos_x, self.sim.boat.pos_y, self.sim.boat.yaw])
        q_goal = np.array([100, 40, np.pi/4])

        # Assuming a constant speed over the entire trajectory for now.
        speed_x = self.sim.boat.vel_x 
        speed_y = self.sim.boat.vel_y

        opti = casadi.Opti()

        # State is (x, y, yaw) where
        # (x,y) is position of boat and
        # yaw is yaw of boat related to pseudo-input u such that d(yaw)/dt = u
        # to ensure continuity of yaw.
        q = opti.variable(3, n_steps+1)
        u = opti.variable(1, n_steps)

        Q = np.diag([10, 10, 0.0])
        R = np.diag([0.1])
        P = (n_steps / 5) * Q

        # Create warm start
        q0 = np.zeros((3, n_steps + 1))
        u0 = np.zeros((1, n_steps))
        for i in range(n_steps + 1):
            q0[:, i] = (i/n_steps)*q_goal + (1-i/n_steps)*q_start

        opti.set_initial(q, q0)
        opti.set_initial(u, u0)

        # Setup objective function
        obj = 0
        for i in range(n_steps):
            qi = q[:, i]
            ui = u[:, i]
            obj += (qi - q_goal).T @ Q @ (qi-q_goal) + ui.T @ R @ ui
        q_last = q[:, n_steps]
        obj += (q_last - q_goal).T @ Q @ (q_last - q_goal)

        opti.minimize(obj)

        # Setup constraints
        constraints = []
        # (x, y) bounds
        # constraints.extend([-50 <= q[0, :], q[0, :] <= 50,
        #                     -50 <= q[1, :], q[1, :] <= 50])
        # # Input constraints (restrict rate of change of yaw)
        constraints.extend([-0.1 <= u[0, :], u[0, :] <= 0.1])
        # Dynamic constraints
        
        speed_angs = [0, np.pi/4, np.pi/2, 5*np.pi/8, 3*np.pi/4, 7*np.pi/8, np.pi]
        speed_vals = [0.729, 0.7373, 0.822, 0.8, 0.67, 0.35, 0.01]
        speed_lut = casadi.interpolant('LUT', 'bspline', [speed_angs], speed_vals)
        def dynamics_model(q, u):
            def qdot(q, u):
                x = q[0]
                y = q[1]
                yaw = q[2]

                ang_diff = yaw - self.sim.env.true_wind.direction
                
                # ang_diff = casadi.fabs(ang_diff) # * casadi.sign(ang_diff)
                # ang_diff = casadi.fmod(yaw - self.sim.env.true_wind.direction, 2*np.pi)
                # assert(casadi.lt(1e-3, ang_diff) and casadi.lt(ang_diff <= np.pi + 1e-3))

                # speed = (1/2)*(1+casadi.cos(ang_diff))
                # speed = .35\left(0.1+1+\sin^{2}\left(\theta\right)+\cos\left(\theta\right)\right)
                # speed = 0.35*(0.1+1+casadi.sin(ang_diff)**2 + casadi.cos(ang_diff))
                
                # speed = speed_lut(ang_diff)
                # speed = casadi.if_else(ang_diff < np.pi/8, 0.72,
                #     casadi.if_else(ang_diff < 3*np.pi/8, 0.737, 
                #     casadi.if_else(ang_diff < 4.5*np.pi/8, 0.823, 
                #     casadi.if_else(ang_diff < 5.5 * np.pi/8, 0.8,
                #     casadi.if_else(ang_diff < 6.5*np.pi/8, 0.67,
                #     casadi.if_else(ang_diff < 7.1*np.pi/8, 0.35, 0.01))))))

                # speed = (1/2)*(1 - casadi.cos(yaw - self.sim.env.true_wind.direction))
                # speed_x = speed*casadi.cos(yaw)
                # speed_y = speed*casadi.sin(yaw)
                
                xdot = speed_x*casadi.cos(yaw) - speed_y*casadi.sin(yaw)
                ydot = speed_x*casadi.sin(yaw) + speed_y*casadi.cos(yaw)
                yawdot = u
                
                return casadi.vertcat(xdot, ydot, yawdot)
            return q + qdot(q, u)*dt
        
        for i in range(n_steps):
            q_i = q[:, i]
            q_ip1 = q[:, i+1]
            u_i = u[:, i]
            constraints.append(q_ip1 == dynamics_model(q_i, u_i))
        
        # Initial and final state constraints
        constraints.append(q[:, 0] == q_start)
        # constraints.append(q[:, -1] == q_goal)

        opti.subject_to(constraints)

        # Solve
        p_opts = {"expand": False, "print_time": 0}
        s_opts = {"max_iter": 1e4, "print_level": 0}
        opti.solver('ipopt', p_opts, s_opts)

        sol = opti.solve()

        return sol.value(q), sol.value(u)


