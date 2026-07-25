import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

timer = 0
dt = 1

N = 5
a_n = 0
V_c = 0

target_coordinates_xy = [0.0, 4000.0]
target_velocity_magnitude = 100.0
target_velocity_norm_xy = [1.0, 0.0]
target_omega = 0.1

interceptor_coordinates_xy = [20000.0, 0.0]
interceptor_velocity_magnitude = 200.0
interceptor_velocity_xy = [0.0, interceptor_velocity_magnitude]


target_px, target_py = [], []
interceptor_px, interceptor_py = [], []
los_frames = []

prev_distance = math.dist(target_coordinates_xy, interceptor_coordinates_xy)

prev_lambda_los = math.atan2(
    target_coordinates_xy[1] - interceptor_coordinates_xy[1],
    target_coordinates_xy[0] - interceptor_coordinates_xy[0],
)

while (
    math.dist(target_coordinates_xy, interceptor_coordinates_xy)
    >= interceptor_velocity_magnitude * dt
):
    timer = timer + dt

    # Target is maneuvring
    target_velocity_norm_xy[1] = math.cos(target_omega * timer)
    target_velocity_norm_xy[0] = math.sqrt(
        1 - math.pow(target_velocity_norm_xy[1], 2))
    target_coordinates_xy[1] = target_coordinates_xy[1] + \
        (target_velocity_magnitude * target_velocity_norm_xy[1])*dt
    target_coordinates_xy[0] = target_coordinates_xy[0] + \
        (target_velocity_magnitude * target_velocity_norm_xy[0])*dt

    # Calculate the distance and the closing velocity
    distance = math.dist(target_coordinates_xy, interceptor_coordinates_xy)
    V_c = -(distance - prev_distance) / dt
    prev_distance = distance

    # Calculate the LOS angle and the rate of LOS angle change
    lambda_los = math.atan2(target_coordinates_xy[1] - interceptor_coordinates_xy[1],
                            target_coordinates_xy[0] - interceptor_coordinates_xy[0],)
    angle_diff = (lambda_los - prev_lambda_los +
                  math.pi) % (2 * math.pi) - math.pi
    lambda_dot = angle_diff / dt
    prev_lambda_los = lambda_los

    # Finally calculate the required lateral acceleration a_n
    a_n = N * V_c * lambda_dot

    # Calculate the heading of the interceptor and the angle normal to the heading to apply a_n
    theta_V = math.atan2(
        interceptor_velocity_xy[1], interceptor_velocity_xy[0])
    theta_a_n = theta_V + math.pi / 2

    # Apple the calculated lateral acceleration
    interceptor_velocity_xy[0] = interceptor_velocity_xy[0] + \
        a_n * math.cos(theta_a_n)
    interceptor_velocity_xy[1] = interceptor_velocity_xy[1] + \
        a_n * math.sin(theta_a_n)

    # Fix the velocity of interceptor to its fixed speed
    velocity_with_a_n = math.hypot(
        interceptor_velocity_xy[0], interceptor_velocity_xy[1])
    interceptor_velocity_xy[1] = (
        interceptor_velocity_magnitude * interceptor_velocity_xy[1] / velocity_with_a_n)
    interceptor_velocity_xy[0] = (
        interceptor_velocity_magnitude * interceptor_velocity_xy[0] / velocity_with_a_n)

    # Maneuver the interceptor
    interceptor_coordinates_xy[1] = (
        interceptor_coordinates_xy[1] + interceptor_velocity_xy[1] * dt)
    interceptor_coordinates_xy[0] = (
        interceptor_coordinates_xy[0] + interceptor_velocity_xy[0] * dt)

    target_px.append(target_coordinates_xy[0])
    target_py.append(target_coordinates_xy[1])
    interceptor_px.append(interceptor_coordinates_xy[0])
    interceptor_py.append(interceptor_coordinates_xy[1])
    los_frames.append(([interceptor_coordinates_xy[0], target_coordinates_xy[0]], [
                      interceptor_coordinates_xy[1], target_coordinates_xy[1]],))

fig, ax = plt.subplots()


def update(i):
    ax.cla()
    ax.plot(target_px[: i + 1], target_py[: i + 1],
            "b-o", markersize=4, label="target")
    ax.plot(interceptor_px[: i + 1], interceptor_py[: i + 1],
            "r-o", markersize=4, label="interceptor",)
    ax.plot(los_frames[i][0], los_frames[i][1], "k--", lw=0.5)
    ax.set_aspect("equal")
    ax.set_xlim(0, 25000)
    ax.set_ylim(0, 10000)
    ax.legend(loc="upper right")


ani = animation.FuncAnimation(fig, update, frames=len(target_px), interval=50)
ani.save(r"ProNav2D.gif", writer="pillow", fps=20)

plt.show()
