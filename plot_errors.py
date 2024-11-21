import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append((val[-1] - first_stamp)/10**9)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,10))

    # code to create the 'ideal' spiral
    x, y = create_spiral()

    # plotting code, added code to plot the odometry values from the robot
    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values], label='EKF Estimated')
    axes[0].plot([lin[len(headers) - 9] for lin in values], [lin[len(headers) - 8] for lin in values], label='Odom')
    #axes[0].plot(x, y, label='Ideal')
    axes[0].set_title("State Space Robot Pose")
    axes[0].set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].grid()
    axes[0].legend()
    
    axes[1].set_title("Individual State Values")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('varies')

    # Adjust spacing
    plt.subplots_adjust(hspace=0.25)  # Increase vertical space between plots

    # Overall Title
    plt.suptitle("EKF Filter Implementation with Q = 0.5, R = 5", fontsize=16)

    plt.show()
    
    # calculating the error of the EKF - comparing odom to the ekf values that are outputted
    error_x = np.array([lin[len(headers) - 3] - lin[len(headers) - 9] for lin in values])
    error_y = np.array([lin[len(headers) - 2]  - lin[len(headers) - 8]for lin in values])

    total_error = np.sqrt(error_x**2 + error_y**2)
    # calculate the error of the EKF over the first 200 time steps, for performance evaluation
    total_error_first200 = np.sum(total_error[:200])


    # plot the error values
    plt.title("EKF vs Odom Error, Q = 0.5, R = 0.05")
    plt.plot(error_x, label='x error')
    plt.plot(error_y, label='y error')
    plt.plot(total_error, label='total error')
    plt.legend()
    plt.grid(True)
    plt.xlabel("Sample Number")
    plt.ylabel("Error")
    plt.show()
    
    print(f'Integral of error over first 200 samples: {total_error_first200}')

# function to create the 'ideal' spiral, using the spiral trajectory planner as the basis
def create_spiral():
    dt = 0.1
    curr_linearVelocity = 0
    x_coords = []
    y_coords = []
    theta = 0
    x = 0
    y = 0

    # Generate x and y coordinates
    linearVelocity = 0

    time_steps = np.arange(0, 6, 0.1)
    for t in time_steps:
        # Linear velocity increases by 0.01 per unit time, capped at max_linear_velocity
        linearVelocity += 0.01 if linearVelocity < 1.0 else 0.0
        # Calculate radius from linear velocity and time
        radius = linearVelocity * t
        # Calculate x and y based on polar coordinates
        angular_velocity = 1
        x = radius * np.cos(angular_velocity * t)
        y = radius * np.sin(angular_velocity * t)
        # Append coordinates to lists
        x_coords.append(x)
        y_coords.append(y)

    # Convert lists to arrays
    x_coords = np.array(x_coords)
    y_coords = np.array(y_coords)
    
    return x_coords, y_coords




import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


