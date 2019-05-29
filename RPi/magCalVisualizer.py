# Import packages
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Define the given offsets
xBias = 333.8484432234432
yBias = 290.79956501831504
zBias = -76.43372252747253

xScale = 1.2572463768115942
yScale = 0.9719887955182074
zScale = 0.8504901960784315

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
    ax.scatter(mx, my, mz)

    # Set title and labels
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Magnetic Field [mG]', fontweight='bold')
    ax.set_ylabel('Magnetic Field [mG]', fontweight='bold')
    ax.set_zlabel('Magnetic Field [mG]', fontweight='bold')

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
plotter3D(mx, my, mz, 'Raw Values')

# Remove any bias and replot the results (hard iron correction)
mx2 = [ii - xBias for ii in mx]
my2 = [ii - yBias for ii in my]
mz2 = [ii - zBias for ii in mz]

plotter(mx2, my2, mz2, 'Bias Removed\nHard Iron Correction')

# Scale and replot the results (soft iron correction)
mx3 = [ii * xScale for ii in mx2]
my3 = [ii * yScale for ii in my2]
mz3 = [ii * zScale for ii in mz2]

plotter(mx3, my3, mz3, 'Bias Removal and Scaled Values\n Hard and Soft Iron Correction')
