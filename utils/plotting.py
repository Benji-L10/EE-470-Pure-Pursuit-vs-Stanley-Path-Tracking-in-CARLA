import os
import pandas as pd
import matplotlib.pyplot as plt


def plot_trajectory(csv_files, labels, output_path):
    plt.figure()

    for csv_file, label in zip(csv_files, labels):
        df = pd.read_csv(csv_file)
        plt.plot(df["x"], df["y"], label=label)

    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.title("Trajectory Comparison")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_time_series(csv_files, labels, column, ylabel, title, output_path):
    plt.figure()

    for csv_file, label in zip(csv_files, labels):
        df = pd.read_csv(csv_file)
        plt.plot(df["time"], df[column], label=label)

    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=300)
    plt.close()