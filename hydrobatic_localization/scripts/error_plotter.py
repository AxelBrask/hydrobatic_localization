import pathlib
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import itertools
try:
    from IPython.display import display
except ImportError:
    def display(x): print(x)
sns.set_style("darkgrid")

log_dir   = pathlib.Path("localizaton_logs") 
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
    df["error_total"] = np.sqrt(df["error_x"]**2 + df["error_y"]**2 + df["error_z"]**2)

    df["run"] = run_name                  
    runs.append(df)

big = pd.concat(runs, ignore_index=True)

display(big.groupby("run")[["error_x", "error_y", "error_z", "error_total"]].describe().round(3))


axis_colours = dict(zip(["error_x", "error_y", "error_z"],
                        sns.color_palette("tab10", 3)))

run_styles = itertools.cycle([
    dict(ls="-",  lw=2.5),    
    dict(ls="--", lw=2.0),    
    dict(ls="-.", lw=1.8),   
    dict(ls=":",  lw=1.8),   
])

plt.figure(figsize=(12, 6))

for run, g in big.groupby("run"):
    style = next(run_styles)           # grab the next visual style
    for axis in ["error_x", "error_y", "error_z"]:
        plt.plot(
            g["time"],
            g[axis],
            label=f"{run} – {axis.split('_')[1]}",   # → “ISAM – x”, “EKF – y”, …
            color=axis_colours[axis],                # colour encodes x/y/z
            **style,                                 # linestyle & linewidth encode run
            alpha=0.9,
        )

plt.xlabel("Time [s]")
plt.ylabel("Error [m]")
plt.title("State‑estimator error per axis")
plt.legend(ncol=3, fontsize=8)
plt.tight_layout()
plt.show()


# 5.  Plot the XY trajectories --------------------------------------------------
plt.figure(figsize=(8, 8))

# plot *one* ground‑truth trajectory (all files share it, so pick the first)
plt.plot(runs[0]["gt_pos_x"], runs[0]["gt_pos_y"],
         c="black", lw=2, ls="-.", label="Ground truth")

# overlay every estimator trajectory
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