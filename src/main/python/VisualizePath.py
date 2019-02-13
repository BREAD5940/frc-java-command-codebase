import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

fieldcorners = [[217,40], [1372, 615]]
imRes = [656, 1592]
fieldsize = [54,27]
multiplier = 21.388888888888888888888888888889

def toFeet(pixels):
  return pixels / multiplier

def toPixels(feet):
  return feet * multiplier

# x = x * multiplier
# y = y * multiplier
# y = 615 - y

def plotAgraph(filepath):
  data = np.genfromtxt(open(filepath, 'r'), delimiter=",")
  data = np.array(data)
  x = data[:,0]
  y = data[:,1]

  img=mpimg.imread('src/main/python/2019-field.png')
  plt.imshow(img, extent=[ toFeet(-217), toFeet(1592-216), toFeet(-40), toFeet(656-40) ])

  plt.scatter(x, y, color = 'm')
  plt.title('Scatter plot of robot pos X vs Y')
  plt.xlabel('robot X')
  plt.ylabel('robot Y')
  plt.show()

plotAgraph("src/main/python/twoHatchLLtest.csv")

# plotAgraph("'src/main/python/twoHatchRRtest.csv'")