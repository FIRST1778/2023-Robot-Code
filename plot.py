#!/usr/bin/env python3
# SPDX-License-Identifier: CC0
import sys
import csv
from matplotlib import pyplot as plt

margin_points = 65

def plot_csv(ax, header, rows):
    ax.margins(0.1, 0.1)
    ax.set(xlabel=header[0])
    ax.spines[["right", "top"]].set_visible(False)
    ax.yaxis.set_visible(False)

    twins = [ax.twinx() for _ in header[1:]]
    for i, twin in enumerate(twins):
        twin.spines[["left", "bottom", "top"]].set_visible(False)
        twin.spines.right.set_position(("outward", margin_points*i))

        p, = twin.plot([row[0] for row in rows],
                       [row[i+1] for row in rows],
                       f"C{i}",
                       label=header[i+1])

        ymin, ymax = twin.get_ylim()
        yh = max(abs(ymin), abs(ymax))
        twin.set_ylim(-yh, yh)

        twin.set(ylabel=header[i+1])
        twin.yaxis.label.set_color(p.get_color())
        twin.tick_params(axis="y", colors=p.get_color())

filenames = sys.argv[1:]
if len(filenames) == 0:
    print("usage: ./plot.py file...")
    sys.exit(1)
fig, axs = plt.subplots(nrows=len(filenames), ncols=1, squeeze=False)

max_header_len = 0
for i, filename in enumerate(filenames):
    with open(filename) as f:
        r = csv.reader(f)
        header = next(r)
        if len(header) > max_header_len:
            max_header_len = len(header)
        rows = [[float(x) for x in row] for row in r]
        del rows[-1]
    plot_csv(axs[i, 0], header, rows)

width = fig.get_figwidth()
labels_width = max_header_len * margin_points / 72
fig.set_figwidth(width + labels_width)
fig.subplots_adjust(right=width / (width + labels_width))

plt.show()
