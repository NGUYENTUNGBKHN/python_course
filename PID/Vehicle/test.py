import numpy as np
from pid import PIDLongitudinalController
from vehicle import Vehicle

# simulation settings
sim_step = 100 # [step]
delta_t = 0.1 # [s]

# initialize vehicle simulator
ref_path_x = np.linspace(-3.0, 50.0, 10)
ref_path_y = np.zeros(10)
vehicle = Vehicle(ref_path = np.array([ref_path_x, ref_path_y]).T, delta_t=delta_t)
vehicle.reset(init_state=np.array([0.0, 0.0, 0.0, 0.0])) # set initial state of the vehicle, [x, y, yaw, v]
vehicle_trajectory = np.array([vehicle.get_state()[:2]])

# initialize speed controller
speed_controller = PIDLongitudinalController(
    p_gain = 1.2,
    i_gain = 0.4,
    d_gain = 0.1,
    target_velocity = 3.0, # [m/s]
)

# simulation loop
for i in range(sim_step):

    # get current state of vehicle
    current_state = vehicle.get_state()
    current_velocity = current_state[3]

    # calculate control inputs
    steer_input = 0.0  # steering command [rad] # set zero because this is just a test run of speed controller
    accel_input = speed_controller.calc_control_input(observed_vel=current_velocity, delta_t=delta_t) # acceleration command [m/s^2]
    vehicle.update(u=[steer_input, accel_input], delta_t=delta_t, vehicle_traj=vehicle_trajectory) # update vehicle state
    vehicle_trajectory = np.vstack((vehicle_trajectory, vehicle.get_state()[:2])) # record vehicle trajectory

# save animation to a file
print("Saving animation to simulation_output.gif...")
# Thay movie_writer="ffmpeg" bằng writer="PillowWriter"
vehicle.save_animation(filename="simulation_output.gif", interval=delta_t*1000, movie_writer="PillowWriter")
print("Done!")