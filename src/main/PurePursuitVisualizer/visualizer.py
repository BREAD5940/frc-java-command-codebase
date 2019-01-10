import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

# {
# "game" : "Destination: Deep Space",
# "field-image" : "2019-field.jpg",
# "field-corners": {
# 	"top-left" : [217, 40],
# 	"bottom-right" : [1372, 615]
# },
# "field-size" : [54, 27],
# "field-unit" : "foot"
# }
topleft = [217, 40]
bottomright = [1372, 615]
fieldsize = [54, 27] # in feet
fieldSizeInPixels = [ (1372 - 217), (615 - 40) ]
imagesize = [1592, 676]

def plotFromFieldCoordinates ( fieldCoordinates, size, color='b' ):
  imageCoordintes = [ fieldCoordinates[0] / fieldsize[0] * fieldSizeInPixels[0] + topleft[0], \
    fieldCoordinates[1] / fieldsize[1] * fieldSizeInPixels[1] + topleft[1] ]
  plt.scatter(imageCoordintes[0], imageCoordintes[1], s=size, c=color)

field = mpimg.imread('2019-field.png')
imgplot = plt.imshow(field)
plotFromFieldCoordinates([0,0], 100)
plotFromFieldCoordinates([54,27], 100)

plt.show()

