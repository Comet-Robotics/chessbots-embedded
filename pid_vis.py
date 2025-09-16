from pathlib import Path
import csv
from collections import defaultdict
from matplotlib import pyplot as plt

with Path('pid_test.csv').open('r') as f:
    reader = csv.DictReader(filter(lambda line: not line.startswith('#'), f))
    lists = defaultdict(list)
    # header = "EncoderLeft,EncoderRight,DesiredVelocityLeft,DesiredVelocityRight,CurrentVelocityLeft,CurrentVelocityRight,LeftPower,RightPower,EncoderTargetLeft,EncoderTargetRight"
    header = "LeftSetpoint,LeftVelocity,LeftError,RightSetpoint,RightVelocity,RightError"
    keys = header.split(",")
    for row in reader:
        for k in keys:
            val = int(row[k]) if "." not in row[k] else float(row[k])
            if "Left" in k:
                val *= -1 # Correct for rotation test
            lists[k].append(val)

    # power_deadzone = [0.15] * len(lists["EncoderLeft"])
    # neg_power_deadzone = [-0.15] * len(lists["EncoderLeft"])

    # fig, (ax_enc, ax_vel, ax_pow) = plt.subplots(3, 1)
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
    
    plt.plot(lists["LeftSetpoint"], "-b", lists["LeftVelocity"], "-r", lists["LeftError"], "-g", lists["RightSetpoint"], "--c", lists["RightVelocity"], "--m", lists["RightError"], "--y")
    plt.show()
