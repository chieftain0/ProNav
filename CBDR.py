import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

dt = 1

target_coordinates_xy = [0.0, 0.0]
target_velocity_magnitude = 1.0
target_velocity_direction = random.uniform(0, math.pi)
target_velocity_xy = [target_velocity_magnitude*math.cos(
    target_velocity_direction), target_velocity_magnitude*math.sin(target_velocity_direction)]

interceptor_coordinates_xy = [40.0, -40.0]
interceptor_velocity_magnitude = 2.0


# Calculate the slope and the initial y-intercept
m = (target_coordinates_xy[1] - interceptor_coordinates_xy[1]) / \
    (target_coordinates_xy[0] - interceptor_coordinates_xy[0])
d_0 = target_coordinates_xy[1] - m * target_coordinates_xy[0]


target_px, target_py = [], []
interceptor_px, interceptor_py = [], []
los_frames = []


while math.dist(target_coordinates_xy, interceptor_coordinates_xy) >= interceptor_velocity_magnitude*dt:

    # Target is maneuvring
    target_coordinates_xy[0] = target_coordinates_xy[0] + \
        dt * target_velocity_xy[0]
    target_coordinates_xy[1] = target_coordinates_xy[1] + \
        dt * target_velocity_xy[1]

    # Calculate the y-intercept of the shifted line
    d_1 = d_0 + target_velocity_xy[1]*dt-m*target_velocity_xy[0]*dt
    d_0 = d_1

    # ax^2+bx+c=0
    a = m*m + 1
    b = 2*m*d_1-2*m * \
        interceptor_coordinates_xy[1] - 2*interceptor_coordinates_xy[0]
    c = (d_1 - interceptor_coordinates_xy[1]) * (d_1 - interceptor_coordinates_xy[1]) + interceptor_coordinates_xy[0] * \
        interceptor_coordinates_xy[0] - \
        (interceptor_velocity_magnitude*dt)*(interceptor_velocity_magnitude*dt)

    # Solve for x using the quadratic
    x_1 = (-b + math.sqrt(b*b - 4*a*c)) / (2*a)
    x_2 = (-b - math.sqrt(b*b - 4*a*c)) / (2*a)

    # Solve for y using the equation of the line
    y_1 = m*x_1 + d_1
    y_2 = m*x_2 + d_1

    # Choose the solution that is closest to the target and move the interceptor
    if math.dist(target_coordinates_xy, [x_1, y_1]) <= math.dist(target_coordinates_xy, [x_2, y_2]):
        interceptor_coordinates_xy[0] = x_1
        interceptor_coordinates_xy[1] = y_1
    else:
        interceptor_coordinates_xy[0] = x_2
        interceptor_coordinates_xy[1] = y_2

    target_px.append(target_coordinates_xy[0])
    target_py.append(target_coordinates_xy[1])
    interceptor_px.append(interceptor_coordinates_xy[0])
    interceptor_py.append(interceptor_coordinates_xy[1])
    los_frames.append(
        (
            [interceptor_coordinates_xy[0], target_coordinates_xy[0]],
            [interceptor_coordinates_xy[1], target_coordinates_xy[1]],
        )
    )

fig, ax = plt.subplots()


def update(i):
    ax.cla()
    ax.plot(target_px[:i+1], target_py[:i+1],
            "b-o", markersize=4, label="target")
    ax.plot(interceptor_px[:i+1], interceptor_py[:i+1],
            "r-o", markersize=4, label="interceptor")
    ax.plot(los_frames[i][0], los_frames[i][1], "k--", lw=0.5)
    ax.set_aspect("equal")
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.legend(loc="upper right")


ani = animation.FuncAnimation(fig, update, frames=len(target_px), interval=50)
ani.save(r"CBDR.gif", writer="pillow", fps=20)

plt.show()
