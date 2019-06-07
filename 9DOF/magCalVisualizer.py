# Import packages
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define the given offsets
magXbias = 145
magYbias = 145
magZbias = -155

magXscale = 1.10
magYscale = 1.05
magZscale = 1.05

# Initialize empty lists
mx = []
my = []
mz = []

def plotter(mx, my, mz, title):
    # Plot the results
    fig, ax = plt.subplots(1)
    ax.scatter(mx, my, c='#FFAA41', marker='o', alpha=0.7, label='XY')
    ax.scatter(mx, mz, c='#2EC6FF', marker='o', alpha=0.7, label='XZ')
    ax.scatter(my, mz, c='#79C850', marker='o', alpha=0.7, label='YZ')

    # Set title and labels
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Magnetic Field [mG]', fontweight='bold')
    ax.set_ylabel('Magnetic Field [mG]', fontweight='bold')

    # Space equally and add a grid
    ax.axis('equal')
    ax.grid()

    # Add a legend
    plt.legend(loc='upper left');

def plotter3D(mx, my, mz, title):
    # Plot the results
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(mx, my, mz, c='#5862C2')

    # Plot projection
    ax.plot(mx, my, color='#FFAA41', alpha=0.7, marker='o', zdir='z', zs=-500)
    ax.plot(mx, mz, color='#2EC6FF', alpha=0.7, marker='o', zdir='y', zs=500)
    ax.plot(my, mz, color='#79C850', alpha=0.7, marker='o', zdir='x', zs=-500)

    # Set title and labels
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Magnetic Field X [mG]', fontweight='bold')
    ax.set_ylabel('Magnetic Field Y [mG]', fontweight='bold')
    ax.set_zlabel('Magnetic Field Z [mG]', fontweight='bold')

    # Set limits on axis
    ax.set_xlim([-500, 500])
    ax.set_ylim([-500, 500])
    ax.set_zlim([-500, 500])

# Open the txt file
f = open('data.txt', 'r')
f1 = f.readlines()

# Read line by line and store the data
for x in f1:
    tempStr = x.split(",")
    mx.append(float(tempStr[0]))
    my.append(float(tempStr[1]))
    mz.append(float(tempStr[2]))

# Close the txt file
f.close()

# Plot the raw results
plotter(mx, my, mz, 'Raw Values')
plotter3D(mx, my, mz, 'Raw Values')
plt.show()

# Remove any bias and replot the results (hard iron correction)
mx2 = [ii - magXbias for ii in mx]
my2 = [ii - magYbias for ii in my]
mz2 = [ii - magZbias for ii in mz]

# Scale and replot the results (soft iron correction)
mx3 = [ii * magXscale for ii in mx2]
my3 = [ii * magYscale for ii in my2]
mz3 = [ii * magZscale for ii in mz2]

plotter(mx3, my3, mz3, 'Bias Removal and Scaled Values\n Hard and Soft Iron Correction')
plotter3D(mx3, my3, mz3, 'Bias Removal and Scaled Values\n Hard and Soft Iron Correction')
plt.show()
