import json
import matplotlib.pyplot as plt
import numpy as np



if __name__== '__main__':
    errList = []
    pointXList = []
    pointYList = []
    with open("/home/jiongrui/catkin_ws/src/object_data/centerlist.json") as f:
        data = json.load(f)
        #print data
        for p in data:
            #print p
            errX = abs(p[0]-0.30)
            errY = abs(p[1]+0.10)
            print errX
            if errX < 0.1 and errY<0.1:
                errList.append([errX,errY])
                pointXList.append(p[0])
                pointYList.append(p[1])
                

    #print errList
    t = np.asarray(range(len(errList)))
    print t+1
    errArray = np.asarray(errList)
    X = np.asarray(pointXList)
    Y = np.asarray(pointYList)
    print errArray[:,0]
    plt.figure(1)
    plt.plot(t+1,errArray[:,0],'r-o')
    plt.grid(True)
    plt.figure(2)
    plt.plot(t+1,errArray[:,1],'g-o')
    plt.grid(True)


    plt.show()
