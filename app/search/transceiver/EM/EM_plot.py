import EmField as beacon
import magpylib as magpy
import matplotlib.pyplot as plt
import numpy as np

beacon_EM_field = beacon.EmField()


def plot_coil_em_field():
    # Plot
    fig, [ax1, ax2] = plt.subplots(1, 2, figsize=(9, 4))

    sp_1 = ax1.streamplot(
        beacon_EM_field.grid[:, :, 0],
        beacon_EM_field.grid[:, :, 2],
        beacon_EM_field.B_1[:, :, 0],
        beacon_EM_field.B_1[:, :, 2],
        density=8,
        color=beacon_EM_field.Bamp_1,
        linewidth=np.sqrt(beacon_EM_field.Bamp_1) * 2,
        cmap="coolwarm",
    )

    # coil 2 plot
    sp_2 = ax2.streamplot(
        beacon_EM_field.grid[:, :, 0],
        beacon_EM_field.grid[:, :, 2],
        beacon_EM_field.B_2[:, :, 0],
        beacon_EM_field.B_2[:, :, 2],
        density=8,
        color=beacon_EM_field.Bamp_2,
        linewidth=np.sqrt(beacon_EM_field.Bamp_2) * 2,
        cmap="coolwarm",
    )

    # # figure styling
    ax1.set(
        title="Magnetic field of front coil",
        xlabel="x-position [mm]",
        ylabel="z-position [mm]",
        aspect=1,
    )

    ax2.set(
        title="Magnetic field of back coil",
        xlabel="x-position [mm]",
        ylabel="z-position [mm]",
        aspect=1,
    )

    plt.colorbar(sp_1.lines, ax=ax1, label="[mT]")
    plt.colorbar(sp_2.lines, ax=ax2, label="[mT]")

    # show plot
    plt.tight_layout()
    plt.show()


def plot_transceiver_em_field():
    # Creating plot
    plt.figure(figsize=(12, 7))
    plt.streamplot(
        beacon_EM_field.grid[:, :, 0],
        beacon_EM_field.grid[:, :, 2],
        beacon_EM_field.B[:, :, 0],
        beacon_EM_field.B[:, :, 2],
        density=10,
        color=beacon_EM_field.B_amp,
        linewidth=np.sqrt(beacon_EM_field.B_amp) * 3,
        cmap="coolwarm",
    )

    # show plot
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("\nUncomment which plot you'd like to see in transceiver/EM_plot.py.")
    print("-----------------------------------------------------------------")
    print(" - DISPLAY COILS SEPARATELY")
    print(" - DISPLAY COIL TOGETHER")

    # DISPLAY COIL SEPARATELY
    # plot_coil_em_field()

    # DISPLAY COILS TOGETHER
    # plot_transceiver_em_field()
