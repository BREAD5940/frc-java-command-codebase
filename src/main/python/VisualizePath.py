import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.animation as animation

fieldcorners = [[217,40], [1372, 615]]
imRes = [656, 1592]
fieldsize = [54,27]
multiplier = 21.388888888888888888888888888889
# fig2 = plt.figure()
# ims = []


def toFeet(pixels):
  return pixels / multiplier

def toPixels(feet):
  return feet * multiplier



# def animate2(i, data):
#   plt.clf()
#   plt.plot(data[0:i,0], data[0:i,1])
#   plt.title('Scatter plot of robot pos X vs Y')
#   plt.xlabel('robot X')
#   plt.ylabel('robot Y')
#   # plt.show()

# def init():
  # plt.clf()
  # plt.plot(data[0:i,0], data[0:i,1])
  # plt.title('Scatter plot of robot pos X vs Y')
  # plt.xlabel('robot X')
  # plt.ylabel('robot Y')

# def animate(i, data):
#   xdata = data[0:i,0]
#   ydata = data[0:i,1]
#   # for index in xrange(len(xdata)):
#   ims.append((plt.scatter(xdata, ydata)))

def getData(filepath):
  data = np.genfromtxt(open(filepath, 'r'), delimiter=",")
  data = np.array(data)
  xdata = data[:,0]
  ydata = data[:,1]

  img=mpimg.imread('src/main/python/2019-field.png')
  # plt.imshow(img, extent=[ toFeet(-217), toFeet(1592-216), toFeet(-40), toFeet(656-40) ])

  # anim = animation.FuncAnimation(fig, animate, init_func=init,
  #                              frames=len(xdata), interval=20, blit=True)

  # animate(300, data)

  # plt.show()
  return xdata, ydata, img

def main():

    # x, y, c = np.random.random((3, numpoints))

    x, y, img = getData("src/main/python/twoHatchLLtest.csv")

    numframes = len(x)/10
    numpoints = len(x)

    color_data = np.random.random((len(x)))

    fig = plt.figure()

  # plt.plot(data[0:i,0], data[0:i,1])
    implot = plt.imshow(img, extent=[ toFeet(-217), toFeet(1592-216), toFeet(-40), toFeet(656-40) ])
    fig.title('Scatter plot of robot pos X vs Y')
    fig.xlabel('robot X')
    fig.ylabel('robot Y')

    scat = plt.scatter(x, y, c= 'm', s=100)


    ani = animation.FuncAnimation(fig, update_plot, frames=range(len(x)),
                                  fargs=(color_data, scat))
    plt.show()

def update_plot(i, data, scat):
    scat.set_array(data[i])
    return scat,

main()






# # ?  plt.scatter(x, y, color = 'm')
#   plt.title('Scatter plot of robot pos X vs Y')
#   plt.xlabel('robot X')
#   plt.ylabel('robot Y')
#   plt.show()


# plotAgraph("'src/main/python/twoHatchRRtest.csv'")