# Import packages
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define the given offsets
magXbias = 296.949
magYbias = 330.334
magZbias = -105.309

magXscale = 1.083
magYscale = 0.949
magZscale = 0.977

# Initialize empty lists
mx = []
my = []
mz = []

def plotter(mx, my, mz, title):
    # Plot the results
    fig, ax = plt.subplots()
    ax.scatter(mx, my, label='XY')
    ax.scatter(mx, mz, label='XZ')
    ax.scatter(my, mz, label='YZ')

    # Set title and labels
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Magnetic Field [mG]', fontweight='bold')
    ax.set_ylabel('Magnetic Field [mG]', fontweight='bold')

    # Space equally and add a grid
    ax.axis('equal')
    ax.grid()

    # Show the legend and plot
    plt.legend(loc='upper left');
    plt.show()

def plotter3D(mx, my, mz, title):
    # Plot the results
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(mx, my, mz, c='#606060')

    # Plot projection
    ax.plot(mx, my, 'r.', zdir='z', zs=-500)
    ax.plot(mx, mz, 'g.', zdir='y', zs=500)
    ax.plot(my, mz, 'b.', zdir='x', zs=-500)

    # Set title and labels
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Magnetic Field X [mG]', fontweight='bold')
    ax.set_ylabel('Magnetic Field Y [mG]', fontweight='bold')
    ax.set_zlabel('Magnetic Field Z [mG]', fontweight='bold')

    # Set limits on axis
    ax.set_xlim([-500, 500])
    ax.set_ylim([-500, 500])
    ax.set_zlim([-500, 500])

    # Show the plot
    plt.show()

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

# Remove any bias and replot the results (hard iron correction)
mx2 = [ii - magXbias for ii in mx]
my2 = [ii - magYbias for ii in my]
mz2 = [ii - magZbias for ii in mz]

plotter(mx2, my2, mz2, 'Bias Removed\nHard Iron Correction')

# Scale and replot the results (soft iron correction)
mx3 = [ii * magXscale for ii in mx2]
my3 = [ii * magYscale for ii in my2]
mz3 = [ii * magZscale for ii in mz2]

plotter(mx3, my3, mz3, 'Bias Removal and Scaled Values\n Hard and Soft Iron Correction')
plotter3D(mx3, my3, mz3, 'Bias Removal and Scaled Values\n Hard and Soft Iron Correction')
