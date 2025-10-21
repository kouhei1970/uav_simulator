"""
Visualization tools for simulation results

3D trajectory, state plots, animations, etc.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class SimulationVisualizer:
    """Visualization of simulation results"""

    def __init__(self):
        """Initialize visualizer"""
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.command_history = []  # Command values (phi_c, theta_c, etc.)

    def add_data(self, time, state, control, command=None):
        """
        Add data point

        Parameters:
            time: Time [s]
            state: State vector
            control: Control input vector
            command: Command vector [phi_c, theta_c, psi_c, p_c, q_c, r_c] (optional)
        """
        self.time_history.append(time)
        self.state_history.append(state.copy())
        self.control_history.append(control.copy())
        if command is not None:
            self.command_history.append(command.copy())
        else:
            self.command_history.append(None)

    def plot_3d_trajectory(self, waypoints=None):
        """
        Plot 3D trajectory

        Parameters:
            waypoints: List of waypoints (optional)
        """
        if len(self.state_history) == 0:
            print("No data available")
            return

        states = np.array(self.state_history)
        x = states[:, 0]
        y = states[:, 1]
        z = -states[:, 2]  # Display altitude as positive

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot trajectory
        ax.plot(x, y, z, 'b-', linewidth=2, label='Trajectory')
        ax.plot([x[0]], [y[0]], [z[0]], 'go', markersize=10, label='Start')
        ax.plot([x[-1]], [y[-1]], [z[-1]], 'ro', markersize=10, label='End')

        # Plot waypoints
        if waypoints is not None:
            wp = np.array(waypoints)
            ax.plot(wp[:, 0], wp[:, 1], -wp[:, 2], 'r*', markersize=15, label='Waypoints')

        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_zlabel('Altitude [m]')
        ax.set_title('3D Flight Trajectory')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.show()

    def plot_states(self):
        """Plot state time series"""
        if len(self.state_history) == 0:
            print("No data available")
            return

        times = np.array(self.time_history)
        states = np.array(self.state_history)

        fig, axes = plt.subplots(4, 3, figsize=(15, 12))

        # Position
        axes[0, 0].plot(times, states[:, 0])
        axes[0, 0].set_ylabel('x [m]')
        axes[0, 0].set_title('North Position')
        axes[0, 0].grid(True)

        axes[0, 1].plot(times, states[:, 1])
        axes[0, 1].set_ylabel('y [m]')
        axes[0, 1].set_title('East Position')
        axes[0, 1].grid(True)

        axes[0, 2].plot(times, -states[:, 2])
        axes[0, 2].set_ylabel('Altitude [m]')
        axes[0, 2].set_title('Altitude')
        axes[0, 2].grid(True)

        # Velocity
        axes[1, 0].plot(times, states[:, 3])
        axes[1, 0].set_ylabel('u [m/s]')
        axes[1, 0].set_title('Forward Velocity')
        axes[1, 0].grid(True)

        axes[1, 1].plot(times, states[:, 4])
        axes[1, 1].set_ylabel('v [m/s]')
        axes[1, 1].set_title('Lateral Velocity')
        axes[1, 1].grid(True)

        axes[1, 2].plot(times, states[:, 5])
        axes[1, 2].set_ylabel('w [m/s]')
        axes[1, 2].set_title('Vertical Velocity')
        axes[1, 2].grid(True)

        # Check if command data is available
        has_commands = any(cmd is not None for cmd in self.command_history)

        # Attitude
        axes[2, 0].plot(times, np.rad2deg(states[:, 6]), 'b-', label='Actual')
        if has_commands:
            # Extract roll command (phi_c) if available
            phi_c_list = [np.rad2deg(cmd[0]) if cmd is not None and len(cmd) > 0 else np.nan
                         for cmd in self.command_history]
            if not all(np.isnan(phi_c_list)):
                axes[2, 0].plot(times, phi_c_list, 'r--', label='Command', alpha=0.7)
                axes[2, 0].legend()
        axes[2, 0].set_ylabel('φ [deg]')
        axes[2, 0].set_title('Roll Angle')
        axes[2, 0].grid(True)

        axes[2, 1].plot(times, np.rad2deg(states[:, 7]), 'b-', label='Actual')
        if has_commands:
            # Extract pitch command (theta_c) if available
            theta_c_list = [np.rad2deg(cmd[1]) if cmd is not None and len(cmd) > 1 else np.nan
                           for cmd in self.command_history]
            if not all(np.isnan(theta_c_list)):
                axes[2, 1].plot(times, theta_c_list, 'r--', label='Command', alpha=0.7)
                axes[2, 1].legend()
        axes[2, 1].set_ylabel('θ [deg]')
        axes[2, 1].set_title('Pitch Angle')
        axes[2, 1].grid(True)

        axes[2, 2].plot(times, np.rad2deg(states[:, 8]), 'b-', label='Actual')
        if has_commands:
            # Extract yaw command (psi_c) if available
            psi_c_list = [np.rad2deg(cmd[2]) if cmd is not None and len(cmd) > 2 else np.nan
                         for cmd in self.command_history]
            if not all(np.isnan(psi_c_list)):
                axes[2, 2].plot(times, psi_c_list, 'r--', label='Command', alpha=0.7)
                axes[2, 2].legend()
        axes[2, 2].set_ylabel('ψ [deg]')
        axes[2, 2].set_title('Yaw Angle')
        axes[2, 2].grid(True)

        # Angular velocity
        axes[3, 0].plot(times, np.rad2deg(states[:, 9]), 'b-', label='Actual')
        if has_commands:
            # Extract roll rate command (p_c) if available
            p_c_list = [np.rad2deg(cmd[3]) if cmd is not None and len(cmd) > 3 else np.nan
                       for cmd in self.command_history]
            if not all(np.isnan(p_c_list)):
                axes[3, 0].plot(times, p_c_list, 'r--', label='Command', alpha=0.7)
                axes[3, 0].legend()
        axes[3, 0].set_ylabel('p [deg/s]')
        axes[3, 0].set_xlabel('Time [s]')
        axes[3, 0].set_title('Roll Rate')
        axes[3, 0].grid(True)

        axes[3, 1].plot(times, np.rad2deg(states[:, 10]), 'b-', label='Actual')
        if has_commands:
            # Extract pitch rate command (q_c) if available
            q_c_list = [np.rad2deg(cmd[4]) if cmd is not None and len(cmd) > 4 else np.nan
                       for cmd in self.command_history]
            if not all(np.isnan(q_c_list)):
                axes[3, 1].plot(times, q_c_list, 'r--', label='Command', alpha=0.7)
                axes[3, 1].legend()
        axes[3, 1].set_ylabel('q [deg/s]')
        axes[3, 1].set_xlabel('Time [s]')
        axes[3, 1].set_title('Pitch Rate')
        axes[3, 1].grid(True)

        axes[3, 2].plot(times, np.rad2deg(states[:, 11]), 'b-', label='Actual')
        if has_commands:
            # Extract yaw rate command (r_c) if available
            r_c_list = [np.rad2deg(cmd[5]) if cmd is not None and len(cmd) > 5 else np.nan
                       for cmd in self.command_history]
            if not all(np.isnan(r_c_list)):
                axes[3, 2].plot(times, r_c_list, 'r--', label='Command', alpha=0.7)
                axes[3, 2].legend()
        axes[3, 2].set_ylabel('r [deg/s]')
        axes[3, 2].set_xlabel('Time [s]')
        axes[3, 2].set_title('Yaw Rate')
        axes[3, 2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_controls(self):
        """Plot control input time series"""
        if len(self.control_history) == 0:
            print("No data available")
            return

        times = np.array(self.time_history)
        controls = np.array(self.control_history)

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        # Elevator
        axes[0, 0].plot(times, np.rad2deg(controls[:, 0]))
        axes[0, 0].set_ylabel('δe [deg]')
        axes[0, 0].set_title('Elevator')
        axes[0, 0].grid(True)

        # Aileron
        axes[0, 1].plot(times, np.rad2deg(controls[:, 1]))
        axes[0, 1].set_ylabel('δa [deg]')
        axes[0, 1].set_title('Aileron')
        axes[0, 1].grid(True)

        # Rudder
        axes[1, 0].plot(times, np.rad2deg(controls[:, 2]))
        axes[1, 0].set_ylabel('δr [deg]')
        axes[1, 0].set_xlabel('Time [s]')
        axes[1, 0].set_title('Rudder')
        axes[1, 0].grid(True)

        # Throttle
        axes[1, 1].plot(times, controls[:, 3])
        axes[1, 1].set_ylabel('δt [-]')
        axes[1, 1].set_xlabel('Time [s]')
        axes[1, 1].set_title('Throttle')
        axes[1, 1].set_ylim([-0.1, 1.1])
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_airdata(self):
        """Plot airdata time series"""
        if len(self.state_history) == 0:
            print("No data available")
            return

        times = np.array(self.time_history)
        states = np.array(self.state_history)

        # Calculate airspeed, angle of attack, sideslip angle
        Va_list = []
        alpha_list = []
        beta_list = []

        for state in states:
            u, v, w = state[3:6]
            Va = np.sqrt(u**2 + v**2 + w**2)
            alpha = np.arctan2(w, u)
            if Va > 0.1:
                beta = np.arcsin(v / Va)
            else:
                beta = 0.0

            Va_list.append(Va)
            alpha_list.append(alpha)
            beta_list.append(beta)

        Va_arr = np.array(Va_list)
        alpha_arr = np.rad2deg(np.array(alpha_list))
        beta_arr = np.rad2deg(np.array(beta_list))

        fig, axes = plt.subplots(3, 1, figsize=(10, 9))

        # Airspeed
        axes[0].plot(times, Va_arr)
        axes[0].set_ylabel('Va [m/s]')
        axes[0].set_title('Airspeed')
        axes[0].grid(True)

        # Angle of attack
        axes[1].plot(times, alpha_arr)
        axes[1].set_ylabel('α [deg]')
        axes[1].set_title('Angle of Attack')
        axes[1].grid(True)

        # Sideslip angle
        axes[2].plot(times, beta_arr)
        axes[2].set_ylabel('β [deg]')
        axes[2].set_xlabel('Time [s]')
        axes[2].set_title('Sideslip Angle')
        axes[2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_2d_trajectory(self, waypoints=None, orbit_center=None, orbit_radius=None):
        """
        Plot 2D trajectory (top view)

        Parameters:
            waypoints: List of waypoints (optional)
            orbit_center: Orbit center (optional)
            orbit_radius: Orbit radius (optional)
        """
        if len(self.state_history) == 0:
            print("No data available")
            return

        states = np.array(self.state_history)
        x = states[:, 0]
        y = states[:, 1]

        fig, ax = plt.subplots(figsize=(10, 10))

        # Plot trajectory
        ax.plot(x, y, 'b-', linewidth=2, label='Trajectory')
        ax.plot(x[0], y[0], 'go', markersize=10, label='Start')
        ax.plot(x[-1], y[-1], 'ro', markersize=10, label='End')

        # Plot waypoints
        if waypoints is not None:
            wp = np.array(waypoints)
            ax.plot(wp[:, 0], wp[:, 1], 'r*', markersize=15, label='Waypoints')
            # Connect waypoints with lines
            ax.plot(wp[:, 0], wp[:, 1], 'r--', alpha=0.5, linewidth=1)

        # Plot orbit circle
        if orbit_center is not None and orbit_radius is not None:
            circle = plt.Circle((orbit_center[0], orbit_center[1]), orbit_radius,
                              color='r', fill=False, linestyle='--', linewidth=2, label='Target Orbit')
            ax.add_patch(circle)

        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_title('2D Flight Trajectory (Top View)')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')

        plt.tight_layout()
        plt.show()

    def save_data(self, filename):
        """
        Save data to file

        Parameters:
            filename: Filename to save
        """
        data = {
            'time': np.array(self.time_history),
            'states': np.array(self.state_history),
            'controls': np.array(self.control_history)
        }
        # Save commands if available
        if self.command_history and any(cmd is not None for cmd in self.command_history):
            # Convert None to NaN array for saving
            commands_array = []
            for cmd in self.command_history:
                if cmd is not None:
                    commands_array.append(cmd)
                else:
                    commands_array.append(np.full(6, np.nan))
            data['commands'] = np.array(commands_array)

        np.savez(filename, **data)
        print(f"Data saved to {filename}")

    def load_data(self, filename):
        """
        Load data from file

        Parameters:
            filename: Filename to load
        """
        data = np.load(filename)
        self.time_history = data['time'].tolist()
        self.state_history = data['states'].tolist()
        self.control_history = data['controls'].tolist()

        # Load commands if available
        if 'commands' in data:
            self.command_history = data['commands'].tolist()
        else:
            self.command_history = []

        print(f"Data loaded from {filename}")

    def reset(self):
        """Reset history"""
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.command_history = []
