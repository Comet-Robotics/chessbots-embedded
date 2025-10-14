from pathlib import Path
import csv
from collections import defaultdict
from matplotlib import pyplot as plt
import numpy as np

with Path('pid_test.csv').open('r') as f:
    reader = csv.DictReader(filter(lambda line: not line.startswith('#'), f))

    lists = defaultdict(list)
    # header = "EncoderLeft,EncoderRight,DesiredVelocityLeft,DesiredVelocityRight,CurrentVelocityLeft,CurrentVelocityRight,LeftPower,RightPower,EncoderTargetLeft,EncoderTargetRight"
    header = "LeftSetpoint,LeftVelocity,LeftError,RightSetpoint,RightVelocity,RightError,HeadingVelocityCorrection,HeadingSetpoint,Heading"
    keys = header.split(",")
    for row in reader:
        for k in keys:
            val = int(row[k]) if "." not in row[k] else float(row[k])
            # if "Left" in k:
            #     val *= -1 # Correct for rotation test
            lists[k].append(val)
    lists = {k: np.array(v) for k, v in lists.items()}

# power_deadzone = [0.15] * len(lists["EncoderLeft"])
# neg_power_deadzone = [-0.15] * len(lists["EncoderLeft"])

# ax_enc.set_title("Encoder Values")
# ax_enc.plot(
#     lists["EncoderLeft"], "-b",
#     lists["EncoderRight"], "-r",
#     lists["EncoderTargetLeft"], "--c",
#     lists["EncoderTargetRight"], "--m",
# )
# ax_vel.set_title("Velocity Values")
# ax_vel.plot(
#     lists["CurrentVelocityLeft"], "-b",
#     lists["CurrentVelocityRight"], "-r",
#     lists["DesiredVelocityLeft"], "--c",
#     lists["DesiredVelocityRight"], "--m",
# )
# ax_pow.set_title("Motor Powers")
# ax_pow.plot(power_deadzone, "--k", neg_power_deadzone, "--k", lists["LeftPower"], "-g", lists["RightPower"], "-y")
# plt.show()

fig = plt.figure(figsize=(15, 8))

# Velocity plot (top left)
ax_vel = plt.subplot2grid((2, 2), (0, 0))
ax_vel.set_title("Velocity Values")
ax_vel.plot(lists["LeftSetpoint"]-lists["HeadingVelocityCorrection"], "--m", lists["LeftVelocity"], "-r", lists["LeftError"], "--g",
            lists["RightSetpoint"]+lists["HeadingVelocityCorrection"], "--c", lists["RightVelocity"], "-b", lists["RightError"], "--y",
            lists["HeadingVelocityCorrection"], "-.k")
ax_vel.legend(["Left Setpoint", "Left Velocity", "Left Error", "Right Setpoint", "Right Velocity", "Right Error", "Heading Velocity Correction"])

# Heading plot (bottom left)
ax_heading = plt.subplot2grid((2, 2), (1, 0), sharex=ax_vel)
ax_heading.set_title("Heading")
ax_heading.plot(lists["Heading"], "-k")
ax_heading.plot(lists["HeadingSetpoint"], "--c")
ax_heading.legend(["Heading", "Heading Setpoint"])
ax_heading.set_xlabel("Time (index)")

# Polar plot (right, spans both rows)
ax_polar = plt.subplot2grid((2, 2), (0, 1), rowspan=2, projection='polar')
ax_polar.set_title("Heading (Polar)")
theta = np.deg2rad(lists["Heading"])
r = np.arange(len(theta))
ax_polar.plot(theta, r, color="purple")
ax_polar.plot(np.deg2rad(lists["HeadingSetpoint"]), r, "--", color="cyan")
ax_polar.set_rticks([])  # Hide radial ticks for clarity

plt.tight_layout()
plt.show()
