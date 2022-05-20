import magpylib as magpy
import matplotlib.pyplot as plt
import numpy as np

# reference: https://magpylib.readthedocs.io/en/latest/examples/examples_30_coil_field_lines.html

# create grid
ts = np.linspace(-100, 100, 100)
grid = np.array([[(x, 0, z) for x in ts] for z in ts])

# coil 1
ts = np.linspace(-8, 8, 1000)
vertices_1 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]

coil_1 = magpy.current.Line(current=20, vertices=vertices_1)
coil_1.rotate_from_angax(45, "y")

# compute and plot field of coil 1
B_1 = magpy.getB(coil_1, grid)
Bamp_1 = np.linalg.norm(B_1, axis=2)
Bamp_1 /= np.amax(Bamp_1)
# coil_1.show()

# coil 2
ts = np.linspace(-8, 8, 1000)
vertices_2 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]
coil_2 = magpy.current.Line(current=20, vertices=vertices_2)
coil_2.rotate_from_angax(-45, "y")

# # compute and plot field of coil 2
B_2 = magpy.getB(coil_2, grid)
Bamp_2 = np.linalg.norm(B_2, axis=2)
Bamp_2 /= np.amax(Bamp_2)
# coil_2.show()

# Plot 
fig, [ax1, ax2] = plt.subplots(1, 2, figsize=(9, 4))

sp_1 = ax1.streamplot(
    grid[:, :, 0],
    grid[:, :, 2],
    B_1[:, :, 0],
    B_1[:, :, 2],
    density=3,
    color=Bamp_1,
    linewidth=np.sqrt(Bamp_1) * 2,
    cmap="coolwarm",
)

sp_2 = ax2.streamplot(
    grid[:, :, 0],
    grid[:, :, 2],
    B_2[:, :, 0],
    B_2[:, :, 2],
    density=3,
    color=Bamp_2,
    linewidth=np.sqrt(Bamp_2) * 2,
    cmap="coolwarm",
)

# figure styling
ax1.set(
    title="Magnetic field of coil 1",
    xlabel="x-position [mm]",
    ylabel="z-position [mm]",
    aspect=1,
)
ax2.set(
    title="Magnetic field of coil 2",
    xlabel="x-position [mm]",
    ylabel="z-position [mm]",
    aspect=1,
)

plt.colorbar(sp_1.lines, ax=ax1, label="[mT]")
plt.colorbar(sp_2.lines, ax=ax2, label="[mT]")

plt.tight_layout()
plt.show()
