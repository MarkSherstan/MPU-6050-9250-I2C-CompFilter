# Import packages
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# Define the given offsets and scale
magXbias = 100
magYbias = 150
magZbias = -250

magXscale = 1.101
magYscale = 1.029
magZscale = 0.893

# Initialize empty lists
mx = []
my = []
mz = []

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

# Perform initial offset (hard iron correction)
mx2 = [ii - magXbias for ii in mx]
my2 = [ii - magYbias for ii in my]
mz2 = [ii - magZbias for ii in mz]

# Scale the offset results (soft iron correction)
mx3 = [ii * magXscale for ii in mx2]
my3 = [ii * magYscale for ii in my2]
mz3 = [ii * magZscale for ii in mz2]

# Create the initial plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.5)

a = ax.scatter(mx3, my3, label='XY')
b = ax.scatter(mx3, mz3, label='XZ')
c = ax.scatter(my3, mz3, label='YZ')

# Set title and labels
ax.set_title('Hard and Soft Iron Correction Slider', fontsize=14, fontweight='bold')
ax.set_xlabel('Magnetic Field [mG]', fontweight='bold')
ax.set_ylabel('Magnetic Field [mG]', fontweight='bold')

# Place grid, space equally, and show the legend
ax.grid()
ax.axis('equal')
plt.legend(loc='upper left');

# Create the sliders (hard iron)
X = plt.axes([0.2, 0.35, 0.65, 0.03], facecolor='#606060')
Y = plt.axes([0.2, 0.30, 0.65, 0.03], facecolor='#606060')
Z = plt.axes([0.2, 0.25, 0.65, 0.03], facecolor='#606060')

sliderX = Slider(X, 'X Bias', -400, 400, valinit=magXbias, valstep=5)
sliderY = Slider(Y, 'Y Bias', -400, 400, valinit=magYbias, valstep=5)
sliderZ = Slider(Z, 'Z Bias', -400, 400, valinit=magZbias, valstep=5)

# Create the sliders (soft iron)
XX = plt.axes([0.2, 0.20, 0.65, 0.03], facecolor='#606060')
YY = plt.axes([0.2, 0.15, 0.65, 0.03], facecolor='#606060')
ZZ = plt.axes([0.2, 0.10, 0.65, 0.03], facecolor='#606060')

sliderXX = Slider(XX, 'X Scale', 0.8, 1.4, valinit=magXscale, valstep=0.05)
sliderYY = Slider(YY, 'Y Scale', 0.8, 1.4, valinit=magYscale, valstep=0.05)
sliderZZ = Slider(ZZ, 'Z Scale', 0.8, 1.4, valinit=magZscale, valstep=0.05)

# Update function
def update(val):
    # Perform the calculation
    mx2 = [ii - sliderX.val for ii in mx]
    my2 = [ii - sliderY.val for ii in my]
    mz2 = [ii - sliderZ.val for ii in mz]

    mx3 = [ii * sliderXX.val for ii in mx2]
    my3 = [ii * sliderYY.val for ii in my2]
    mz3 = [ii * sliderZZ.val for ii in mz2]

    # Update the plots data
    xyData = np.vstack ((mx3, my3))
    a.set_offsets (xyData.T)

    xzData = np.vstack ((mx3, mz3))
    b.set_offsets (xzData.T)

    yzData = np.vstack ((my3, mz3))
    c.set_offsets (yzData.T)

    # Refresh the plot
    fig.canvas.draw_idle()

# Call the update function for any of the sliders
sliderX.on_changed(update)
sliderY.on_changed(update)
sliderZ.on_changed(update)

sliderXX.on_changed(update)
sliderYY.on_changed(update)
sliderZZ.on_changed(update)

# Create a reset button
resetLoc = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetLoc, 'Reset', color='#c2c4c6', hovercolor='0.9')

# Set up the reset button event
def reset(event):
    sliderX.reset()
    sliderY.reset()
    sliderZ.reset()
    sliderXX.reset()
    sliderYY.reset()
    sliderZZ.reset()

# Call the reset button
button.on_clicked(reset)

# Display the plot
plt.show()
