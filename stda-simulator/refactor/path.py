import numpy as np
import casadi

class Path:
    def __init__(self, simulator):
        self.sim = simulator
    
    def yaw(self, time):
        if time < 40:
            return 0.0
        elif time < 90:
            return np.pi/4
        # elif time < 100:
        #     return np.pi/4
        else:
            return np.pi/2
        # return 0
    
    def plan_path(self):

        n_steps = 1000
        dt = self.sim.stepsize
        q_start = np.array([self.sim.boat.pos_x, self.sim.boat.pos_y, self.sim.boat.yaw])
        q_goal = np.array([40, 40, np.pi/4])

        # Assuming a constant speed over the entire trajectory for now.
        # 0.8 number was experimentally determined from the stable speed the boat tends to go at
        # speed = np.sqrt(self.sim.boat.vel_x**2 + self.sim.boat.vel_y**2)
        # speed = (speed + 0.8)/2
        speed_x = self.sim.boat.vel_x
        speed_y = self.sim.boat.vel_y

        opti = casadi.Opti()

        # State is (x, y, yaw) where
        # (x,y) is position of boat and
        # yaw is yaw of boat related to pseudo-input u such that d(yaw)/dt = u
        # to ensure continuity of yaw.
        q = opti.variable(3, n_steps+1)
        u = opti.variable(1, n_steps)

        Q = np.diag([1, 1, 0.1])
        R = 2*np.diag([1])
        P = n_steps * Q

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
        obj += (q_last - q_goal).T @ P @ (q_last - q_goal)

        opti.minimize(obj)

        # Setup constraints
        constraints = []
        # (x, y) bounds
        constraints.extend([-50 <= q[0, :], q[0, :] <= 50,
                            -50 <= q[1, :], q[1, :] <= 50])
        # Input constraints (restrict rate of change of yaw)
        constraints.extend([-100 <= u[0, :], u[0, :] <= 100])
        # Dynamic constraints
        def dynamics_model(q, u):
            def qdot(q, u):
                x = q[0]
                y = q[1]
                yaw = q[2]
                
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
        constraints.append(q[:, -1] == q_goal)

        opti.subject_to(constraints)

        # Solve
        p_opts = {"expand": False}
        s_opts = {"max_iter": 1e4}
        opti.solver('ipopt', p_opts, s_opts)

        sol = opti.solve()


