import numpy as np
import casadi

class Path:
    def __init__(self, simulator):
        self.sim = simulator

        self.yaw_desireds = None
        self.last_plan_time = None
        self.replan_interval = 50 # Replan every replan_interval seconds. Must be less than or equal to planning horizon.
        self.planning_horizon = 50 # Plan planning_horizon seconds into the future

    
    def yaw(self, time):
        if self.yaw_desireds is None:
            plan, u = self.plan_path()
            self.yaw_desireds = plan[2, :]
            self.last_plan_time = time
        
        
        if self.last_plan_time + self.replan_interval < time:
            print('replanning at time: ', time)
            plan, u = self.plan_path()
            print('replanned')
            self.yaw_desireds = plan[2, :]
            self.last_plan_time = time

        index = int((time - self.last_plan_time) / self.sim.stepsize)
        return self.yaw_desireds[index]
        
        # if time < 40:
        #     return 0.0
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
        q_goal = np.array([40, 40, np.pi/4])

        # Assuming a constant speed over the entire trajectory for now.
        # 0.8 number was experimentally determined from the stable speed the boat tends to go at
        speed_x = self.sim.boat.vel_x 
        speed_y = self.sim.boat.vel_y

        opti = casadi.Opti()

        # State is (x, y, yaw) where
        # (x,y) is position of boat and
        # yaw is yaw of boat related to pseudo-input u such that d(yaw)/dt = u
        # to ensure continuity of yaw.
        q = opti.variable(3, n_steps+1)
        u = opti.variable(1, n_steps)

        Q = np.diag([10, 10, 0.1])
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
        def dynamics_model(q, u):
            def qdot(q, u):
                x = q[0]
                y = q[1]
                yaw = q[2]

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
        p_opts = {"expand": True, "print_time": 0}
        s_opts = {"max_iter": 1e4, "print_level": 0}
        opti.solver('ipopt', p_opts, s_opts)

        sol = opti.solve()

        return sol.value(q), sol.value(u)


