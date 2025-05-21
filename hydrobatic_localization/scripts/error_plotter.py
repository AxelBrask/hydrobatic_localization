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


def umeyama_alignment(P, Q, with_scale=False):
    """
    Return (s, R, t) that best aligns     P (Nx3, est)
    to                                      Q (Nx3, gt)
    s = scale (1 if with_scale=False)
    R = 3×3 rotation matrix
    t = 3-vector translation
    """
    mu_P, mu_Q   = P.mean(0), Q.mean(0)
    P0,   Q0     = P - mu_P, Q - mu_Q
    C            = P0.T @ Q0 / len(P)
    U, sigma, Vt     = np.linalg.svd(C)
    D            = np.eye(3);   D[-1, -1] = np.sign(np.linalg.det(U @ Vt))
    R_opt        = U @ D @ Vt
    if with_scale:
        var_P    = (P0**2).sum()/len(P)
        s_opt    = (sigma @ D).sum() / var_P
    else:
        s_opt    = 1.0
    t_opt        = mu_Q - s_opt*R_opt@mu_P
    return s_opt, R_opt, t_opt


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

log_dir   = pathlib.Path("open_water_logs") 
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
    _, R_opt, t_opt = umeyama_alignment(P_est, P_gt, with_scale=False)
    P_est_aligned = (R_opt @ P_est.T).T + t_opt
    df[["est_al_x", "est_al_y", "est_al_z"]] = P_est_aligned
    df["trans_err"]  = np.linalg.norm(P_est_aligned - P_gt, axis=1)

    q_est = df[["est_quat_x","est_quat_y","est_quat_z","est_quat_w"]].values
    q_gt  = df[["gt_quat_x", "gt_quat_y", "gt_quat_z", "gt_quat_w"] ].values
    df["rot_err_deg"] = quat_error_deg(q_est, q_gt)

    df["run"] = run_name
    runs.append(df)          

big = pd.concat(runs, ignore_index=True)

display(big.groupby("run")[["error_x", "error_y", "error_z", "error_total"]].describe().round(3))

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

for axis, label in [("trans_err","ATE [m]"), ("rot_err_deg","ARE [°]")]:
    plt.figure(figsize=(12,4))
    run_styles = itertools.cycle(style_list)
    for run in runs_new:
        g = big[big["run"] == run]
        plt.plot(g["time"], g[axis], label=f"{run}", 
                 color=run_colours[run], **next(run_styles))
    plt.xlabel("Time [s]"); plt.ylabel(label)
    plt.title(f"{label} over time"); plt.legend(ncol=len(runs_new))
    plt.tight_layout(); plt.show()




plt.figure(figsize=(8, 8))

plt.plot(runs[0]["gt_pos_x"], runs[0]["gt_pos_y"],
         c="black", lw=2, ls="-.", label="Ground truth")
for run, g in big.groupby("run"):
    plt.plot(g["est_pos_x"], g["est_pos_y"],
             lw=1.5, label=f"{run} – est")

plt.xlabel("X [m]"); plt.ylabel("Y [m]")
plt.title("XY‑plane trajectory comparison")
plt.legend(fontsize=8); plt.axis("equal"); plt.tight_layout(); plt.show()


plt.figure(figsize=(8, 6))
sns.boxplot(data=big, x="run", y="error_total", fliersize=1)
plt.xlabel("Run")
plt.ylabel("Total |position error| [m]")
plt.title("Absolute position‑error distribution per run")
plt.tight_layout()
plt.show()

plt.figure(figsize=(8,8))
plt.plot(runs[0]["gt_pos_x"], runs[0]["gt_pos_y"],
         c="black", lw=2, ls="--", label="GT")
sc = plt.scatter(df["est_al_x"], df["est_al_y"], c=df["trans_err"],
                 s=8, cmap="viridis_r")
plt.colorbar(sc, label="ATE [m]")
plt.axis("equal"); plt.legend(); plt.title("Error-coloured trajectory")
plt.tight_layout(); plt.show()
