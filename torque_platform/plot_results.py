"""实验数据离线绘图工具。"""

import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np


def find_latest_npz(data_dir):
    candidates = glob.glob(os.path.join(data_dir, "*.npz"))
    if not candidates:
        candidates = glob.glob("*.npz")
    if not candidates:
        return None
    candidates.sort(key=os.path.getmtime)
    return candidates[-1]


def load_data(path):
    data = np.load(path, allow_pickle=True)
    required = ["t", "q", "xr", "u_raw", "u"]
    missing = [key for key in required if key not in data.files]
    if missing:
        raise KeyError(f"Missing required fields in {path}: {missing}")
    return data


def joint_labels(data, joint_count):
    if "p_robot_torque_joints" not in data.files:
        return [f"Joint {i + 1}" for i in range(joint_count)]
    indexes = np.asarray(data["p_robot_torque_joints"]).reshape(-1)
    if indexes.size != joint_count:
        return [f"Joint {i + 1}" for i in range(joint_count)]
    return [f"J{int(index) + 1}" for index in indexes]


def make_subplots(title, t, values, ylabel, labels, plotter):
    joint_count = values.shape[1]
    fig, axes = plt.subplots(
        joint_count,
        1,
        figsize=(8.0, max(2.4 * joint_count, 3.0)),
        sharex=True,
    )
    if joint_count == 1:
        axes = [axes]

    fig.suptitle(title)
    for i, ax in enumerate(axes):
        plotter(ax, i)
        ax.set_ylabel(ylabel)
        ax.set_title(labels[i])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
    axes[-1].set_xlabel("Time (s)")
    for ax in axes:
        ax.set_xlim(t[0], t[-1])
    fig.tight_layout()
    return fig


def save_or_show(figures, source_path, save, outdir, formats, show):
    if save:
        os.makedirs(outdir, exist_ok=True)
        stem = os.path.splitext(os.path.basename(source_path))[0]
        for name, fig in figures:
            for fmt in formats:
                path = os.path.join(outdir, f"{stem}_{name}.{fmt}")
                fig.savefig(path, dpi=300)
                print(f"Saved -> {path}")
    if show:
        plt.show()
    else:
        for _, fig in figures:
            plt.close(fig)


def plot_results(path, save=False, outdir="figures", formats=("png", "pdf"), show=True):
    data = load_data(path)
    t = np.asarray(data["t"], dtype=float)
    q = np.asarray(data["q"], dtype=float)
    xr = np.asarray(data["xr"], dtype=float)
    u_raw = np.asarray(data["u_raw"], dtype=float)
    u = np.asarray(data["u"], dtype=float)

    if q.ndim != 2:
        raise ValueError("q must be a 2D array with shape (samples, joints)")
    joint_count = q.shape[1]
    labels = joint_labels(data, joint_count)

    figures = []

    figures.append(
        (
            "tracking",
            make_subplots(
                "Tracking Performance",
                t,
                q,
                "Position (rad)",
                labels,
                lambda ax, i: (
                    ax.plot(t, xr[:, i], "--", label="Reference"),
                    ax.plot(t, q[:, i], label="Actual"),
                ),
            ),
        )
    )

    error = xr - q
    figures.append(
        (
            "error",
            make_subplots(
                "Tracking Error",
                t,
                error,
                "Error (rad)",
                labels,
                lambda ax, i: ax.plot(t, error[:, i], label="Reference - Actual"),
            ),
        )
    )

    figures.append(
        (
            "torque",
            make_subplots(
                "Control Torque",
                t,
                u,
                "Torque (N*m)",
                labels,
                lambda ax, i: (
                    ax.plot(t, u_raw[:, i], "--", label="Raw"),
                    ax.plot(t, u[:, i], label="Limited"),
                ),
            ),
        )
    )

    save_or_show(figures, path, save, outdir, formats, show)


def main():
    parser = argparse.ArgumentParser(description="Plot Kinova TorqueLab .npz data.")
    parser.add_argument("file", nargs="?", default=None, help=".npz data file")
    parser.add_argument(
        "--data-dir",
        default=os.path.join(os.path.dirname(__file__), "data"),
        help="Directory used when no file is given.",
    )
    parser.add_argument("--save", action="store_true", help="Save figures.")
    parser.add_argument(
        "--fmt",
        nargs="+",
        default=("png", "pdf"),
        help="Figure formats used with --save, e.g. --fmt png pdf.",
    )
    parser.add_argument("--outdir", default="figures", help="Figure output directory.")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display figures. Useful when only saving files.",
    )
    args = parser.parse_args()

    path = args.file or find_latest_npz(args.data_dir)
    if path is None:
        raise FileNotFoundError("No .npz file found.")
    plot_results(
        path,
        save=args.save,
        outdir=args.outdir,
        formats=args.fmt,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
