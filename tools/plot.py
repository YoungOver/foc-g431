"""Plots the simulation trace written by `build/test --csv`: speed, current and observer error."""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

src, out = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(src)))
col = lambda k: [float(r[k]) for r in rows]
t = col("t")

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})
fig, ax = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True, constrained_layout=True)

ax[0].plot(t, col("rpm_true"), label="реальная", lw=1.6)
ax[0].plot(t, col("rpm_est"), label="оценка наблюдателя", lw=1, ls="--")
ax[0].set_ylabel("скорость, об/мин")
ax[0].legend(loc="lower right")

ax[1].plot(t, col("iq_ref"), label="задание iq", lw=1)
ax[1].plot(t, col("iq"), label="iq", lw=0.8, alpha=0.8)
ax[1].set_ylabel("ток, А")
ax[1].legend(loc="upper right")

ax[2].plot(t, col("theta_err_deg"), lw=0.8, color="tab:red")
ax[2].set_ylim(-20, 20)
ax[2].set_ylabel("ошибка угла, °")
ax[2].set_xlabel("время, с")

for a in ax:
    a.axvline(2.0, color="gray", lw=0.8, ls=":")
ax[0].annotate("наброс нагрузки", (2.0, 0), xytext=(2.05, 400), fontsize=9, color="gray")

fig.savefig(f"{out}/response.png", dpi=150)
print(f"{out}/response.png")
