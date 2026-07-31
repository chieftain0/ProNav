import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def cross3D(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        -1 * (a[0] * b[2] - b[0] * a[2]),
        a[0] * b[1] - a[1] * b[0],
    ]


def dot3D(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


timer = 0
dt = 1
N = 3
a_xyz = [0.0, 0.0, 0.0]
Vr_xyz = [0.0, 0.0, 0.0]
R_xyz = [0.0, 0.0, 0.0]
omega_xyz = [0.0, 0.0, 0.0]

target_coordinates_xyz = [0.0, 5000.0, 8000.0]
target_velocity_magnitude = 150.0
target_velocity_norm_xyz = [1.0, 0.0, 0.0]
target_velocity_xyz = [0.0, 0.0, 0.0]
w_target = 0.1

interceptor_coordinates_xyz = [25000.0, 10000.0, 0.0]
interceptor_velocity_magnitude = 200.0
interceptor_velocity_xyz = [0.0, 0.0, interceptor_velocity_magnitude]
interceptor_velocity_norm_xyz = [0.0, 0.0, 0.0]

target_px, target_py, target_pz = [], [], []
interceptor_px, interceptor_py, interceptor_pz = [], [], []
los_frames = []

while (
    math.dist(target_coordinates_xyz, interceptor_coordinates_xyz)
    >= interceptor_velocity_magnitude
):
    timer += dt

    # Target is maneuvring
    target_velocity_norm_xyz[1] = 0.75 * math.cos(w_target * timer)
    target_velocity_norm_xyz[2] = -0.75 * math.sin(w_target * timer)
    target_velocity_norm_xyz[0] = math.sqrt(
        1
        - target_velocity_norm_xyz[1] ** 2
        - target_velocity_norm_xyz[2] ** 2
    )

    target_velocity_xyz[0] = (
        target_velocity_norm_xyz[0] * target_velocity_magnitude
    )
    target_velocity_xyz[1] = (
        target_velocity_norm_xyz[1] * target_velocity_magnitude
    )
    target_velocity_xyz[2] = (
        target_velocity_norm_xyz[2] * target_velocity_magnitude
    )

    # Calculate closing velocity
    Vr_xyz[0] = target_velocity_xyz[0] - \
        interceptor_velocity_xyz[0]
    Vr_xyz[1] = target_velocity_xyz[1] - \
        interceptor_velocity_xyz[1]
    Vr_xyz[2] = target_velocity_xyz[2] - \
        interceptor_velocity_xyz[2]

    # Calculate the relative position
    R_xyz[0] = target_coordinates_xyz[0] - interceptor_coordinates_xyz[0]
    R_xyz[1] = target_coordinates_xyz[1] - interceptor_coordinates_xyz[1]
    R_xyz[2] = target_coordinates_xyz[2] - interceptor_coordinates_xyz[2]

    # Finally calculate the required lateral acceleration
    omega_xyz_times_RdotR = cross3D(R_xyz, Vr_xyz)
    # or RdotR = math.hypot(R_xyz[0], R_xyz[1], R_xyz[2])**2
    RdotR = dot3D(R_xyz, R_xyz)
    omega_xyz[0] = omega_xyz_times_RdotR[0] / RdotR
    omega_xyz[1] = omega_xyz_times_RdotR[1] / RdotR
    omega_xyz[2] = omega_xyz_times_RdotR[2] / RdotR
    a_xyz_divide_N = cross3D(Vr_xyz, omega_xyz)
    a_xyz = [a_xyz_divide_N[0] * N,
             a_xyz_divide_N[1] * N, a_xyz_divide_N[2] * N]

    interceptor_velocity_xyz[0] += a_xyz[0] * dt
    interceptor_velocity_xyz[1] += a_xyz[1] * dt
    interceptor_velocity_xyz[2] += a_xyz[2] * dt

    spd = math.dist(interceptor_velocity_xyz, [0.0, 0.0, 0.0])
    interceptor_velocity_norm_xyz[0] = interceptor_velocity_xyz[0] / spd
    interceptor_velocity_norm_xyz[1] = interceptor_velocity_xyz[1] / spd
    interceptor_velocity_norm_xyz[2] = interceptor_velocity_xyz[2] / spd

    interceptor_velocity_xyz[0] = (
        interceptor_velocity_norm_xyz[0] *
        interceptor_velocity_magnitude
    )
    interceptor_velocity_xyz[1] = (
        interceptor_velocity_norm_xyz[1] *
        interceptor_velocity_magnitude
    )
    interceptor_velocity_xyz[2] = (
        interceptor_velocity_norm_xyz[2] *
        interceptor_velocity_magnitude
    )

    interceptor_coordinates_xyz[0] += interceptor_velocity_xyz[0] * dt
    interceptor_coordinates_xyz[1] += interceptor_velocity_xyz[1] * dt
    interceptor_coordinates_xyz[2] += interceptor_velocity_xyz[2] * dt

    target_coordinates_xyz[0] += target_velocity_xyz[0] * dt
    target_coordinates_xyz[1] += target_velocity_xyz[1] * dt
    target_coordinates_xyz[2] += target_velocity_xyz[2] * dt

    target_px.append(target_coordinates_xyz[0])
    target_py.append(target_coordinates_xyz[1])
    target_pz.append(target_coordinates_xyz[2])
    interceptor_px.append(interceptor_coordinates_xyz[0])
    interceptor_py.append(interceptor_coordinates_xyz[1])
    interceptor_pz.append(interceptor_coordinates_xyz[2])
    los_frames.append(
        (
            [interceptor_coordinates_xyz[0], target_coordinates_xyz[0]],
            [interceptor_coordinates_xyz[1], target_coordinates_xyz[1]],
            [interceptor_coordinates_xyz[2], target_coordinates_xyz[2]],
        )
    )


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


def update(i):
    ax.cla()
    ax.plot(
        target_px[: i + 1],
        target_py[: i + 1],
        target_pz[: i + 1],
        "b-o",
        markersize=3,
        label="target",
    )
    ax.plot(
        interceptor_px[: i + 1],
        interceptor_py[: i + 1],
        interceptor_pz[: i + 1],
        "r-o",
        markersize=3,
        label="interceptor",
    )
    ax.plot(los_frames[i][0], los_frames[i][1],
            los_frames[i][2], "k--", lw=0.5)
    ax.set_xlim(0, 26000)
    ax.set_ylim(0, 10000)
    ax.set_zlim(0, 10000)
    ax.legend(loc="upper right")


ani = animation.FuncAnimation(fig, update, frames=len(target_px), interval=50)

ani.save(
    r"ProNav3D.gif",
    writer="pillow",
    fps=20,
)


plt.show()
