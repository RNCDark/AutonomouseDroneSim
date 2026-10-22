
# generate_charts.py

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Use the completed reachable-map experiment.
DATA_FILE = Path("results/reachable_experiment_results.csv")
OUTPUT_DIR = Path("results")

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Load the experiment data exported by experiments.py.
df = pd.read_csv(DATA_FILE)

# Separate the two scenarios and align them by seed.
static = df[df["scenario"] == "static"].set_index("seed")
dynamic = df[df["scenario"] == "dynamic"].set_index("seed")

seeds = sorted(set(static.index) & set(dynamic.index))

# ---------------------------------------------------------
# Figure 1: Static versus dynamic travel distance
# ---------------------------------------------------------

x = np.arange(len(seeds))
bar_width = 0.38

fig, ax = plt.subplots(figsize=(9, 4.5), dpi=150)

ax.bar(
    x - bar_width / 2,
    static.loc[seeds, "distance_traveled"],
    bar_width,
    label="Static",
    color="#4C78A8"
)

ax.bar(
    x + bar_width / 2,
    dynamic.loc[seeds, "distance_traveled"],
    bar_width,
    label="Dynamic",
    color="#F58518"
)

ax.set_title("Static vs. Dynamic Navigation Distance")
ax.set_xlabel("Environment Seed")
ax.set_ylabel("Distance Traveled (Grid Movements)")
ax.set_xticks(x)
ax.set_xticklabels(seeds)
ax.set_ylim(0, 45)
ax.legend()
ax.grid(axis="y", alpha=0.25)

fig.tight_layout()
fig.savefig(OUTPUT_DIR / "distance_comparison.png")
plt.close(fig)

# ---------------------------------------------------------
# Figure 2: Dynamic obstacle rerouting frequency
# ---------------------------------------------------------

fig, ax = plt.subplots(figsize=(9, 4.5), dpi=150)

reroutes = dynamic.loc[seeds, "reroutes"]

bars = ax.bar(
    [str(seed) for seed in seeds],
    reroutes,
    color="#E45756"
)

# Label each bar with its recorded reroute count.
ax.bar_label(bars, padding=3)

ax.set_title("Dynamic Obstacle Rerouting Frequency")
ax.set_xlabel("Environment Seed")
ax.set_ylabel("Number of Reroutes")
ax.set_ylim(0, max(5, reroutes.max() + 1))
ax.grid(axis="y", alpha=0.25)

fig.tight_layout()
fig.savefig(OUTPUT_DIR / "rerouting_frequency.png")
plt.close(fig)

# ---------------------------------------------------------
# Print summary statistics for the report.
# ---------------------------------------------------------

print("\nExperimental Summary")
print("-" * 40)

for scenario_name, subset in (
    ("Static", static),
    ("Dynamic", dynamic)
):
    print(f"\n{scenario_name}:")
    print(f"Success rate: {subset['success'].mean() * 100:.1f}%")
    print(f"Average distance: {subset['distance_traveled'].mean():.2f}")
    print(f"Average reroutes: {subset['reroutes'].mean():.2f}")
    print(
        "Average path efficiency: "
        f"{subset['path_efficiency_percent'].mean():.2f}%"
    )
    print(
        "Average completion time: "
        f"{subset['completion_time_seconds'].mean():.2f}s"
    )

print("\nCharts saved to results/")
