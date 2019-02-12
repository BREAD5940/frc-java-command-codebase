# ------- file: myplot.py ------
import matplotlib.pyplot as plt
import numpy as np
import csv

def parseCsv(cesv):
    colors = ['g','r','c','m','y','k','w']
    c=0
    with open(cesv) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                line_count += 1
            else:
                plott(float(row[0]), float(row[1]), float(row[2]), colors[c])
                line_count += 1
                if(c==colors.len()):
                    c=0
                else:
                    c=c+1
        # print(f"Processed {line_count} lines.")

def plott(h,e,w,color):
    ne=1
    nw=1

    if (e<-np.pi/2):
        ne=-1

    if (w<-np.pi/2):
        nw=-1

    # p1x=np.sin(e)
    # print(p1x)
    # p1x=np.square(p1x)
    # print(p1x)
    # p1x=p1x+24
    # print(p1x)
    # p1x=p1x*4
    # print(p1x)
    # p1x=100-p1x
    # print(p1x)
    # p1x=np.sqrt(p1x)
    # print(p1x)
    # p1x=ne*p1x
    # print(p1x)
    # p1x=10+p1x
    # print(p1x)
    # p1x=p1x/2
    # print(p1x)

    p1x=5+np.cos(e)

    # print(p1x)
    print("new")
    p2x=np.cos(w)
    print(p2x)
    p2x=p2x/2
    print(p2x)
    p2x=p2x+p1x
    print(p2x)

    # print(p2x)

    p1y=h+np.sin(e)
    p2y=p1y+np.sin(w)

    plt.scatter(5, h, c='b')
    plt.scatter(p1x, p1y, c=color)
    plt.scatter(p2x, p2y, c=color)

    plt.plot([5,5],[0,h],c='b')
    plt.plot([5,p1x],[h,p1y],c=color)
    plt.plot([p1x,p2x],[p1y,p2y],c=color)


eAngles = [0, np.pi/2, -np.pi/2, np.pi/4, -np.pi/4]
wAngles = [0, np.pi/2, -np.pi/2, np.pi/4, -np.pi/4]

fSize=plt.rcParams['figure.figsize']
fSize[0]=6
fSize[1]=12  #todo fix this so the sizing is less deceptive
plt.rcParams['figure.figsize']=fSize
parseCsv('src/main/python/test.csv')
plt.show()