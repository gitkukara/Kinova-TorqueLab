"""实验数据离线绘图工具。

本文件定义默认图表的内容和样式。如需改变曲线、标题、配色或导出方式，
可直接修改 ``plot_results()``、``make_subplots()`` 和 ``save_or_show()``。
"""

import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


def configure_chinese_font():
    """优先选择系统中已有的中文字体，避免中文标题显示为方框。"""

    installed = {font.name for font in font_manager.fontManager.ttflist}
    candidates = (
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "Arial Unicode MS",
    )
    for name in candidates:
        if name in installed:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


configure_chinese_font()


def find_latest_npz(data_dir):
    """查找指定数据目录中的最新 ``.npz`` 文件。"""

    candidates = glob.glob(os.path.join(data_dir, "*.npz"))
    if not candidates:
        candidates = glob.glob("*.npz")
    if not candidates:
        return None
    candidates.sort(key=os.path.getmtime)
    return candidates[-1]


def load_data(path):
    """加载实验数据，并确认绘图所需的基本字段存在。"""

    data = np.load(path, allow_pickle=True)
    required = ["t", "q", "xr", "u_raw", "u"]
    missing = [key for key in required if key not in data.files]
    if missing:
        raise KeyError(f"{path} 缺少绘图所需字段：{missing}")
    return data


def joint_labels(data, joint_count):
    """优先根据保存的执行器索引生成关节标签。"""

    if "p_robot_torque_joints" not in data.files:
        return [f"关节 {i + 1}" for i in range(joint_count)]
    indexes = np.asarray(data["p_robot_torque_joints"]).reshape(-1)
    if indexes.size != joint_count:
        return [f"关节 {i + 1}" for i in range(joint_count)]
    return [f"J{int(index) + 1}" for index in indexes]


def make_subplots(title, t, values, ylabel, labels, plotter):
    """按受控关节数量创建纵向排列、共享时间轴的子图。"""

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
    axes[-1].set_xlabel("时间 (s)")
    for ax in axes:
        ax.set_xlim(t[0], t[-1])
    fig.tight_layout()
    return fig


def save_or_show(figures, source_path, save, outdir, formats, show):
    """根据参数保存、显示或关闭已经生成的图窗。"""

    if save:
        os.makedirs(outdir, exist_ok=True)
        stem = os.path.splitext(os.path.basename(source_path))[0]
        for name, fig in figures:
            for fmt in formats:
                path = os.path.join(outdir, f"{stem}_{name}.{fmt}")
                fig.savefig(path, dpi=300)
                print(f"已保存 -> {path}")
    if show:
        plt.show()
    else:
        for _, fig in figures:
            plt.close(fig)


def plot_results(path, save=False, outdir="figures", formats=("png", "pdf"), show=True):
    """读取一次实验数据并生成跟踪、误差和控制力矩图。"""

    data = load_data(path)
    t = np.asarray(data["t"], dtype=float)
    q = np.asarray(data["q"], dtype=float)
    xr = np.asarray(data["xr"], dtype=float)
    u_raw = np.asarray(data["u_raw"], dtype=float)
    u = np.asarray(data["u"], dtype=float)

    if q.ndim != 2:
        raise ValueError("q 必须是形状为 (采样点数, 关节数) 的二维数组")
    joint_count = q.shape[1]
    labels = joint_labels(data, joint_count)

    figures = []

    figures.append(
        (
            "tracking",
            make_subplots(
                "轨迹跟踪",
                t,
                q,
                "位置 (rad)",
                labels,
                lambda ax, i: (
                    ax.plot(t, xr[:, i], "--", label="参考轨迹"),
                    ax.plot(t, q[:, i], label="实际轨迹"),
                ),
            ),
        )
    )

    error = xr - q
    figures.append(
        (
            "error",
            make_subplots(
                "跟踪误差",
                t,
                error,
                "误差 (rad)",
                labels,
                lambda ax, i: ax.plot(t, error[:, i], label="参考值 - 实际值"),
            ),
        )
    )

    figures.append(
        (
            "torque",
            make_subplots(
                "控制力矩",
                t,
                u,
                "力矩 (N*m)",
                labels,
                lambda ax, i: (
                    ax.plot(t, u_raw[:, i], "--", label="控制器原始输出"),
                    ax.plot(t, u[:, i], label="安全层处理后"),
                ),
            ),
        )
    )

    save_or_show(figures, path, save, outdir, formats, show)


def main():
    parser = argparse.ArgumentParser(description="绘制 Kinova TorqueLab 的 .npz 数据。")
    parser.add_argument("file", nargs="?", default=None, help="要绘制的 .npz 数据文件")
    parser.add_argument(
        "--data-dir",
        default=os.path.join(os.path.dirname(__file__), "data"),
        help="未指定数据文件时用于查找最新数据的目录。",
    )
    parser.add_argument("--save", action="store_true", help="保存图片。")
    parser.add_argument(
        "--fmt",
        nargs="+",
        default=("png", "pdf"),
        help="与 --save 配合使用的图片格式，例如 --fmt png pdf。",
    )
    parser.add_argument("--outdir", default="figures", help="图片输出目录。")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="不显示图窗，适合只保存图片的场景。",
    )
    args = parser.parse_args()

    path = args.file or find_latest_npz(args.data_dir)
    if path is None:
        raise FileNotFoundError("未找到 .npz 数据文件。")
    plot_results(
        path,
        save=args.save,
        outdir=args.outdir,
        formats=args.fmt,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
