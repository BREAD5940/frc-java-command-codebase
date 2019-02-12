import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

data = np.genfromtxt(open('src\\main\\python\\twoHatchLLtest.csv', 'r'), delimiter=",")

data = np.array(data)

x = data[:,0]
y = data[:,1]

fieldcorners = [[217,40], [1372, 615]]
fieldsize = [54,27]
multiplier = 21.388888888888888888888888888889
print(multiplier)

x = x * multiplier + fieldcorners[0][0]
y = y * multiplier + fieldcorners[0][1]


colors = (0,0,0)
area = np.pi*3

img=mpimg.imread('src\\main\\python\\2019-field.png')
plt.imshow(img)

plt.plot(x, y, color = 'm')
plt.title('Scatter plot of robot pos X vs Y')
plt.xlabel('robot X')
plt.ylabel('robot Y')
plt.show()