import pathlib
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import itertools
from scipy.spatial.transform import Rotation as R
try:
    from IPython.display import display
except ImportError:
    def display(x): print(x)
sns.set_style("darkgrid")


# def umeyama_alignment(P, Q, with_scale=False):
#     """
#     Return (s, R, t) that best aligns     P (Nx3, est)
#     to                                      Q (Nx3, gt)
#     s = scale (1 if with_scale=False)
#     R = 3×3 rotation matrix
#     t = 3-vector translation
#     """
#     mu_P, mu_Q   = P.mean(0), Q.mean(0)
#     P0,   Q0     = P - mu_P, Q - mu_Q   
#     C            = P0.T @ Q0
#     U, sigma, Vt     = np.linalg.svd(C)
#     D            = np.eye(3)
#     D[-1, -1] = np.sign(np.linalg.det(Vt.T @ U.T))
#     R_opt        = Vt.T @ D @ U.T
#     if with_scale:
#         var_P    = (P0**2).sum()/len(P)
#         s_opt    = (sigma @ D).sum() / var_P
#     else:
#         s_opt    = 1.0
#     t_opt        = mu_Q - s_opt*R_opt@mu_P
#     return s_opt, R_opt, t_opt


def quat_error_deg(q_est, q_gt):
    """
    Smallest angular distance between two quaternions, in degrees.
    Input shape (..., 4) with scalar-last convention (x,y,z,w).
    """
    q_est = q_est/np.linalg.norm(q_est, axis=-1, keepdims=True)
    q_gt  = q_gt /np.linalg.norm(q_gt , axis=-1, keepdims=True)
    # relative rotation  q_err = q_est * conj(q_gt)
    # scipy expects (x,y,z,w):
    R_err = R.from_quat(q_est)*R.from_quat(q_gt).inv()
    return np.degrees(R_err.magnitude())          

log_dir   = pathlib.Path("comparison_results") 
log_paths = sorted(log_dir.glob("*.csv")) 




#static
#turn back and frouth
#pitch
runs = []
for p in log_paths:
    run_name = p.stem
    df = pd.read_csv(p)
    df.columns = df.columns.str.strip()

    df["error_x"] = df["est_pos_x"] - df["gt_pos_x"]
    df["error_y"] = df["est_pos_y"] - df["gt_pos_y"]
    df["error_z"] = df["est_pos_z"] - df["gt_pos_z"]
    df["error_total"] = np.sqrt(df["error_x"]**2 +
                                df["error_y"]**2 +
                                df["error_z"]**2)

    P_est = df[["est_pos_x", "est_pos_y", "est_pos_z"]].values
    P_gt  = df[["gt_pos_x",  "gt_pos_y",  "gt_pos_z"] ].values
    # _, R_opt, t_opt = umeyama_alignment(P_est, P_gt, with_scale=False)
    # P_est_aligned = (R_opt @ P_est.T).T + t_opt
    # df[["est_al_x", "est_al_y", "est_al_z"]] = P_est_aligned
    df["trans_err"] = df["error_total"]

    q_est = df[["est_quat_x","est_quat_y","est_quat_z","est_quat_w"]].values
    q_gt  = df[["gt_quat_x", "gt_quat_y", "gt_quat_z", "gt_quat_w"] ].values
    df["rot_err_deg"] = quat_error_deg(q_est, q_gt)

    df["run"] = run_name
    runs.append(df)          

# big = pd.concat(runs, ignore_index=True)
min_len = min(len(df) for df in runs)
print(f"Truncating all runs to {min_len} samples (the shortest run)")

# 2) truncate every DataFrame in-place (and recombine)
runs_trunc = [df.iloc[:min_len].copy() for df in runs]
big = pd.concat(runs_trunc, ignore_index=True)
display(big.groupby("run")[["error_x", "error_y", "error_z", "error_total"]].describe().round(3))
distances = []
for run, g in big.groupby("run"):
    # extract arrays of shape (N,3)
    gt_pts  = g[["gt_pos_x",  "gt_pos_y",  "gt_pos_z"]].values
    est_pts = g[["est_pos_x", "est_pos_y", "est_pos_z"]].values

    # differences between consecutive frames
    gt_deltas  = np.diff(gt_pts,  axis=0)
    est_deltas = np.diff(est_pts, axis=0)

    # Euclidean lengths of those deltas
    gt_step_lengths  = np.linalg.norm(gt_deltas,  axis=1)
    est_step_lengths = np.linalg.norm(est_deltas, axis=1)

    # total distance = sum of step lengths
    total_gt  = gt_step_lengths.sum()
    total_est = est_step_lengths.sum()
    distances.append({
        "run": run,
        "total_gt_distance_m":  round(total_gt, 3),
        "total_est_distance_m": round(total_est, 3)
    })

# turn into a DataFrame and display
dist_df = pd.DataFrame(distances)
display(dist_df)
runs_new =  sorted(big["run"].unique())
run_colours = dict(zip(runs_new, sns.color_palette("tab10", len(runs_new))))
def rmse(err):
    return np.sqrt((err**2).mean())
style_list = [
    dict(ls="-",  lw=2.5),
    dict(ls="-", lw=2.0),
    dict(ls="-", lw=1.8),
    dict(ls="-",  lw=1.8),
]

for axis in ["error_x", "error_y", "error_z"]:
    run_styles = itertools.cycle(style_list)

    plt.figure(figsize=(12, 4))
    for run in runs_new:
        g = big[big["run"] == run]
        style = next(run_styles)
        plt.plot(
            g["time"],
            g[axis],
            label=f"{run} – {axis.split('_')[1].upper()}",
            color=run_colours[run],  
            **style,
            alpha=0.9,
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.title(f"{axis.split('_')[1].upper()} Error over Time")
    plt.legend(fontsize=8, ncol=len(runs_new))
    plt.tight_layout()
    plt.show()  



summary = (big.groupby("run")[["trans_err","rot_err_deg"]]
           .agg(["mean","median","max",rmse]).round(3))
summary.rename_axis("run / metric", inplace=True)
display(summary)

# for axis, label in [("trans_err","ATE [m]"), ("rot_err_deg","ARE [°]")]:
#     plt.figure(figsize=(12,4))
#     run_styles = itertools.cycle(style_list)
#     for run in runs_new:
#         g = big[big["run"] == run]
#         plt.plot(g["time"], g[axis], label=f"{run}", 
#                  color=run_colours[run], **next(run_styles))
#     plt.xlabel("Time [s]"); plt.ylabel(label)
#     plt.title(f"{label} over time"); plt.legend(ncol=len(runs_new))
#     plt.tight_layout(); plt.show()




baseline_run = runs_trunc[0]["run"].iloc[0]

# 2) compute the common end‐time (smallest finish time across all runs)
end_times  = big.groupby("run")["time"].max()
common_end = end_times.min()

# 3) extract GT segment once
gt = big[big["run"] == baseline_run]
gt_seg = gt[gt["time"] <= common_end]

# helper to plot GT
def plot_ground_truth(ax):
    ax.plot(
        gt_seg["gt_pos_x"], gt_seg["gt_pos_y"],
        c="black", lw=2, ls="-.", label="Ground truth"
    )
    ax.scatter(
        gt_seg["gt_pos_x"].iloc[0], gt_seg["gt_pos_y"].iloc[0],
        c="green", marker="o", s=80, label="Start"
    )
    ax.scatter(
        gt_seg["gt_pos_x"].iloc[-1], gt_seg["gt_pos_y"].iloc[-1],
        c="red", marker="X", s=80, label="End"
    )

# figure 1: only runs ending with "_mm"
plt.figure(figsize=(8, 8))
ax1 = plt.gca()
plot_ground_truth(ax1)

for run, g in big.groupby("run"):
    if run.endswith("_mm"):
        seg = g[g["time"] <= common_end]
        ax1.plot(seg["est_pos_x"], seg["est_pos_y"], lw=1.5, label=run)
        ax1.scatter(seg["est_pos_x"].iloc[0], seg["est_pos_y"].iloc[0], c="green", marker="o", s=60)
        ax1.scatter(seg["est_pos_x"].iloc[-1], seg["est_pos_y"].iloc[-1], c="red", marker="X", s=60)

ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_title("XY‑plane trajectories (_mm runs)")
ax1.legend(fontsize=8, ncol=2)
ax1.axis("equal")
plt.tight_layout()
plt.show()


# figure 2: only runs NOT ending with "_mm"
plt.figure(figsize=(8, 8))
ax2 = plt.gca()
plot_ground_truth(ax2)

for run, g in big.groupby("run"):
    if not run.endswith("_mm"):
        seg = g[g["time"] <= common_end]
        ax2.plot(seg["est_pos_x"], seg["est_pos_y"], lw=1.5, label=run)
        ax2.scatter(seg["est_pos_x"].iloc[0], seg["est_pos_y"].iloc[0], c="green", marker="o", s=60)
        ax2.scatter(seg["est_pos_x"].iloc[-1], seg["est_pos_y"].iloc[-1], c="red", marker="X", s=60)

ax2.set_xlabel("X [m]")
ax2.set_ylabel("Y [m]")
ax2.set_title("XY‑plane trajectories (non _mm runs)")
ax2.legend(fontsize=8, ncol=2)
ax2.axis("equal")
plt.tight_layout()
plt.show()


# plt.figure(figsize=(8, 8))

# plt.plot(runs_trunc[0]["gt_pos_x"], runs_trunc[0]["gt_pos_y"],
#          c="black", lw=2, ls="-.", label="Ground truth")
# for run, g in big.groupby("run"):
#     plt.plot(g["est_al_x"], g["est_al_y"],
#              lw=1.5, label=f"{run} ")

# plt.xlabel("X [m]"); plt.ylabel("Y [m]")
# plt.title("XY‑plane trajectory comparison")
# plt.legend(fontsize=8); plt.axis("equal"); plt.tight_layout(); plt.show()


plt.figure(figsize=(8, 6))
sns.boxplot(data=big, x="run", y="trans_err", fliersize=1)
plt.xlabel("Run")
plt.ylabel("Absolut trajectory error [m]")
plt.title("Absolute trajectory‑error distribution per run")
plt.tight_layout()
plt.show()

# plt.figure(figsize=(8,8))
# plt.plot(runs_trunc[0]["gt_pos_x"], runs_trunc[0]["gt_pos_y"],
#          c="black", lw=2, ls="--", label="GT")
# sc = plt.scatter(df["est_al_x"], df["est_al_y"], c=df["trans_err"],
#                  s=8, cmap="viridis_r")
# plt.colorbar(sc, label="ATE [m]")
# plt.axis("equal"); plt.legend(); plt.title("Error-coloured trajectory")
# plt.tight_layout(); plt.show()


quat_est = big[["est_quat_x","est_quat_y","est_quat_z","est_quat_w"]].values
quat_gt  = big[["gt_quat_x","gt_quat_y","gt_quat_z","gt_quat_w"]].values

rots_est = R.from_quat(quat_est)  # scipy expects (x,y,z,w)
euler_est = rots_est.as_euler('xyz', degrees=True)

rots_gt = R.from_quat(quat_gt)
euler_gt = rots_gt.as_euler('xyz', degrees=True)

# Compute the difference in roll, pitch, yaw, wrapped to [-180, 180]
diff = euler_est - euler_gt
diff = (diff + 180) % 360 - 180

# Add to DataFrame
big["roll_err"]  = diff[:, 0]
big["pitch_err"] = diff[:, 1]
big["yaw_err"]   = diff[:, 2]

# Plot roll, pitch, yaw errors over time
for axis, label in [("roll_err",  "Roll Error [°]"),
                    ("pitch_err", "Pitch Error [°]"),
                    ("yaw_err",   "Yaw Error [°]")]:
    plt.figure(figsize=(12, 4))
    run_styles = itertools.cycle(style_list)
    for run in runs_new:
        g = big[big["run"] == run]
        style = next(run_styles)
        plt.plot(
            g["time"],
            g[axis],
            label=f"{run}",
            color=run_colours[run],
            **style,
            alpha=0.9,
        )
    plt.xlabel("Time [s]")
    plt.ylabel(label)
    plt.title(f"{label} Over Time")
    plt.legend(fontsize=8, ncol=len(runs_new))
    plt.tight_layout()
    plt.show()