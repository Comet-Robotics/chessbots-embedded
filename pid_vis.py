import matplotlib.pyplot as plt
import sys

xpos = []
ypos = []
lpow = []
rpow = []
xvel = []
yvel = []
avel = []

with open(sys.argv[1]) as f:
    for line in f:
        if "vel" in line:
            lines = line.split(" ")
            for x in range(0,len(lines)-1,2):
                match lines[x]:
                    case "vel":
                        xvel.append(float(lines[x+1]))
                    case "yvel":
                        yvel.append(float(lines[x+1]))
                    case "avel":
                        avel.append(float(lines[x+1]))
                    case "lpower":
                        lpow.append(float(lines[x+1]))
                    case "rpower":
                        rpow.append(float(lines[x+1]))
                    case "xpos":
                        xpos.append(float(lines[x+1]))
                    case "ypos":
                        ypos.append(float(lines[x+1]))

t = range(0,len(avel))

fig, axs = plt.subplots(3, 2)
axs[0, 0].plot(xpos, ypos)
axs[0, 0].set_title('Position')

axs[1, 0].plot(t,lpow, 'tab:green')
axs[1, 0].set_title('Left Power')

axs[2, 0].plot(t,rpow, 'tab:red')
axs[2, 0].set_title('Right Power')

axs[0, 1].plot(t,xvel, 'tab:orange')
axs[0, 1].set_title('Forward Velocity')

axs[1, 1].plot(t,yvel, 'tab:blue')
axs[1, 1].set_title('Horizontal Velocity')

axs[2, 1].plot(t,avel, 'tab:red')
axs[2, 1].set_title('Rotational Velocity')

fig.tight_layout()
plt.show()