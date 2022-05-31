import magpylib as magpy
import matplotlib.pyplot as plt
import numpy as np

# create grid
ts = np.linspace(-200, 200, 200)
grid = np.array([[(x, 0, z) for x in ts] for z in ts])


# coil 1
ts = np.linspace(-60, 60, 1000)
vertices_1 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]

coil_1 = magpy.current.Line(current=7500, vertices=vertices_1)  # AAA battery
coil_1.rotate_from_angax(45, "y")

# coil 2
ts = np.linspace(-60, 60, 1000)
vertices_2 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]
coil_2 = magpy.current.Line(current=7500, vertices=vertices_2)  # AAA battery
coil_2.rotate_from_angax(-45, "y")


# compute and plot field of coil 1
B_1 = magpy.getB(coil_1, grid)
Bamp_1 = np.linalg.norm(B_1, axis=2)
Bamp_1 /= np.amax(Bamp_1)

# compute and plot field of coil 2
B_2 = magpy.getB(coil_2, grid)
Bamp_2 = np.linalg.norm(B_2, axis=2)
Bamp_2 /= np.amax(Bamp_2)


# Adding magnetic fields of coil 1 and coil 2
B = np.add(B_1, B_2)
B_amp = np.linalg.norm(B, axis=2)
B_amp /= np.amax(B_amp)


def plot_coil_em_field():
    pass

def plot_transceiver_em_field():
    # Creating plot
    plt.figure(figsize=(12, 7))
    plt.streamplot(
        grid[:, :, 0],
        grid[:, :, 2],
        B[:, :, 0],
        B[:, :, 2],
        density=8,
        color=B_amp,
        linewidth=np.sqrt(B_amp) * 3,
        cmap="coolwarm",
    )

    # show plot
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_transceiver_em_field()