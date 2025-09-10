import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

# 1. Setup the figure and axes for the animation
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect('equal')
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.axis('off') # Hide axes for a cleaner visual

# 2. Define static components (magnets)
magnet_width = 0.5
magnet_height = 1.0
magnet_offset = 1.0

# North pole (red)
ax.add_patch(Rectangle((-magnet_width/2, magnet_offset), magnet_width, magnet_height, color='red'))
# South pole (blue)
ax.add_patch(Rectangle((-magnet_width/2, -magnet_offset - magnet_height), magnet_width, magnet_height, color='blue'))

# 3. Define the rotor (coil) and its initial state
coil_length = 1.0
coil_pos = np.array([[-coil_length/2, 0], [coil_length/2, 0]])
coil_line = Line2D([], [], color='black', linewidth=5)
ax.add_line(coil_line)

# 4. Animation functions
def init():
    coil_line.set_data([], [])
    return coil_line,

def update(frame):
    # Simulate rotation. Replace this with your actual LQR-controlled angle.
    # For a simple demo, we'll just increment the angle.
    angle_rad = np.deg2rad(frame * 5) # Rotate 5 degrees per frame

    # Apply rotation transformation
    rot_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])

    rotated_coil_pos = (rot_matrix @ coil_pos.T).T

    # Update the coil's position
    coil_line.set_data(rotated_coil_pos[:, 0], rotated_coil_pos[:, 1])
    return coil_line,

# 5. Create and run the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 360, 1), init_func=init, blit=True)

plt.show()

# To save the animation as a GIF, you might need to install 'ffmpeg' or 'imagemagick'
# ani.save('motor_animation.gif', writer='imagemagick', fps=60)