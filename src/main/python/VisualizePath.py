import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.animation as animation
from matplotlib import style

fieldcorners = [[217,40], [1372, 615]]
imRes = [656, 1592]
fieldsize = [54,27]
multiplier = 21.388888888888888888888888888889

def toFeet(pixels):
  return pixels / multiplier

def toPixels(feet):
  return feet * multiplier


def getData(filepath):
  data = np.genfromtxt(open(filepath, 'r'), delimiter=",")
  data = np.array(data)
  xdata = data[:,0]
  ydata = data[:,1]
  img=mpimg.imread('src/main/python/2019-field.png')
  return data, xdata, ydata, img

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
data, x, y, img = getData("src/main/python/twoHatchLLtest.csv")
# i=0

def main():
    extent = [ [toFeet(-217), toFeet(1592-216)], [toFeet(-40), toFeet(656-40)] ]

    # while True:

    img=mpimg.imread('src/main/python/2019-field.png')
    implot = plt.imshow(img, extent=[ toFeet(-217), toFeet(1592-216), toFeet(-40), toFeet(656-40) ])

    ani = animation.FuncAnimation(fig, animate, interval=10)

    # plt.figure(1)
    # plt.imshow(mpimg.imread('src/main/python/2019-field.png'))

    # ax1.imshow(mpimg.imread('src/main/python/2019-field.png'))

    plt.show()

    ax1.clear()
    ax1.imshow(img, extent=[ toFeet(-217), toFeet(1592-216), toFeet(-40), toFeet(656-40) ])
    ax1.scatter(data[:,0], data[:,1], color='m', s=5)
    plt.show()



def animate(i):
#   i = i + 1
  ax1.clear()
  ax1.imshow(img, extent=[ toFeet(-217), toFeet(1592-216), toFeet(-40), toFeet(656-40) ])
  multi = 8
  if i > len(x)/multi - 1:
    return
  # print(data[0:i,0])
  ax1.scatter(data[0:i*multi,0], data[0:i*multi,1], color='m', s=5)




main()