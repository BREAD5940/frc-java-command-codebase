import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math

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

#waypoints = np.array([ [5.0,10.0], [15.0,10.0], [30.0, 20.0]])
waypoints = np.array([ [5.0,5.0], [15.0,15.0], [30.0,20.0]])


#waypoints = np.array([ [5.0,10.0],[45.0,10.0], [95.0,10.0], [135.0, 0.0]])
maxAllowableDistance = 3
linepointsy = []
linepointsx = []



def plotFromFieldCoordinates ( fieldCoordinates, size = 75, color='b' ):
  imageCoordintes = [ (fieldCoordinates[0] / fieldsize[0] * fieldSizeInPixels[0]) + topleft[0], fieldCoordinates[1] / fieldsize[1] * fieldSizeInPixels[1] + topleft[1] ]
  # print imageCoordintes
  # print fieldCoordinates
  # print (fieldCoordinates[0] / 54.00)
  plt.scatter(imageCoordintes[0], imageCoordintes[1], s=size, c=color)

field = mpimg.imread('2019-field.png')
imgplot = plt.imshow(field)


for point in range(len(waypoints) - 1):
  # Do stuff
  tx = (waypoints[point][0] - waypoints[point+1][0]) #gonna have to change to use for multiple points.. oops
  ty = (waypoints[point][1] - waypoints[point+1][1])
  distance = math.sqrt(math.pow(tx,2) + math.pow(ty,2)) #distance formula

  spacebetween = math.floor(distance / maxAllowableDistance)
  # print("space between: %s" % spacebetween)
  
  #for x in range(spacebetween):

  #this will onyl work for a set number of variables (in this case I said I wanted 5 points, i)

  i = 5 #soon to be turned into a loop
  
  linepointsy.append(waypoints[point][1])
  linepointsx.append(waypoints[point][0])
  thecout = i-1 #number of spaces I need
  print("The count: %s" % thecout)
  xspacing = abs(tx/thecout)
  yspacing = abs(ty/thecout)
  print("{%s}x spacing, {%s}y spacing" % (xspacing, yspacing))
  # fillinponts = thecout - 1
  
  
  for x in range(i-2):
    print(x)
    # linepointsx.append(linepointsx[x-1] + xspacing)
    # linepointsy.append(linepointsy[x-1] + yspacing)
    linepointsx.append(linepointsx[0] + xspacing * (x+1))
    linepointsy.append(linepointsy[0] + yspacing * (x+1))

    print("Lines point x:")
    print(linepointsx)
    print(linepointsy)

    #does this work..? now need to create a loop
    print ("line point: (%s, %s)" % (linepointsx[x], linepointsy[x]))

    # x+= 1
    # thecout-=1

for x in range(len(linepointsx)):
  if(waypoints[point] is waypoints[len(waypoints) - 1]):
    linepointsx.append(waypoints[point][0])
    linepointsy.append(waypoints[point][1])
  
  plotFromFieldCoordinates([linepointsx[x], linepointsy[x]])





# for point in range(len(waypoints)):
  # print waypoints[point][0]
  # print waypoints[point][1]
  # print("waypoint: (%s, %s)" % (waypoints[point][0], waypoints[point][1]))
  # plotFromFieldCoordinates([waypoints[point][0], waypoints[point][1]], color='r')
  


plt.show()


