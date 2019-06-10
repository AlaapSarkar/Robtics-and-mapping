from PIL import Image, ImageDraw, ImageFilter, ImageEnhance
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy as np
import urllib.request
import json
import math


# Webserver address and port, use 127.0.0.1 instead of localhost, it will be resolved quicker
url = 'http://127.0.0.1:18080'

sonarIDtoAngle = [90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90]

#Store data here
robotTrajectoryX = []
robotTrajectoryY = []

scatterX = []
scatterY = []
globx=scatterX
globy=scatterY

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------
"""

MAP NAME HERE
- this MUST be equal to the map you are using

"""




mapName = 'map1'

# ---------------------------------------------------------------------------------------------------------------------------------------------------------------
# Importing the map and extracting the lines
filepath = './maps/'+mapName+'.map'
lines = []

with open(filepath) as fp:

   recordinglines = False
   line = fp.readline()
   cnt = 1

   while line:

       print("Reading Line {}: {}".format(cnt, line.strip()))
       line = fp.readline()
       cnt += 1

       if recordinglines:
           if line.strip() != "DATA":
                lines.append(line.strip())

       if line.strip() == "LINES":
           recordinglines = True

lines.remove('')
print("Number of Lines: {}".format(len(lines)))


formattedLines = []

for line in lines:
    linedata = line.split()
    formattedLines.append({'p1': [int(linedata[0]), int(linedata[1])], 'p2':[int(linedata[2]), int(linedata[3])]})




# ---------------------------------------------------------------------------------------------------------------------------------------------------------------
# Calculate minimum and maximum points from line segments
# Get map file from folder and transpose map (flip upside down)


minX = 0
minY = 0
maxX = 0
maxY = 0

for line in formattedLines:
    p1x = line['p1'][0]
    p1y = line['p1'][1]
    p2x = line['p2'][0]
    p2y = line['p2'][1]

    if p1x < minX:
        minX = p1x
    if p1y < minY:
        minY = p1y

    if p2x < minX:
        minX = p2x
    if p2y < minY:
        minY = p2y

    if p1x > maxX:
        maxX = p1x
    if p1y > maxY:
        maxY = p1y

    if p2x > maxX:
        maxX = p2x
    if p2y > maxY:
        maxY = p2y

print("minX {} minY {} maxX {} maxY {}".format(minX, minY, maxX, maxY))

lengthX = maxX - minX
lengthY = maxY - minY

mapImage = Image.open('./occupancyMaps/'+mapName+'.png')
mapImage_t = mapImage.transpose(Image.FLIP_TOP_BOTTOM)

maxsize = (mapImage_t.width * 100, mapImage_t.height  * 100)
mapTransposedImage = mapImage_t.resize(maxsize)




## =======================================================================================================================

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

def getOccupancyGridValue(x, y):

    xT = x + abs(minX)
    yT = y + abs(minY)

    gridValue = mapTransposedImage.getpixel((xT, yT))[0]

    normalizedValue = (gridValue - 0) / (255 - 0)

    return normalizedValue
#plt.imshow(mapImage)

#print(getOccupancyGridValue(-9020,-2020))
#print(getOccupancyGridValue(-8500,-1800))
print(getOccupancyGridValue(5000,5000))

"""

Initial Setup Below Here

- Create and setup particles here, remember to set up the weightings of these particles
- Think of a good data structure to represent the particles

"""

n_p=1000
scatterX=np.random.rand(n_p,1)
scatterX[:,0] *= 5000
scatterY=np.random.rand(n_p,1)
scatterY[:,0] *= 5000
weights = np.ones((n_p,1))
pix = np.array(mapImage)
scatterX.dump("xp_matrix.dat")
scatterY.dump("yp_matrix.dat")
weights.dump("w_matrix.dat")



def animate(i, pix):
    size=pix.shape
    scatterX = np.load("xp_matrix.dat")
    scatterY = np.load("yp_matrix.dat")
    weights = np.load("w_matrix.dat")
    link="http://127.0.0.1:18080/"
    f = urllib.request.urlopen(link)
    myfile = f.read()
    data = json.loads(myfile)
    x=data['absolutePosition']['x']
    y=data['absolutePosition']['y']
    th=math.pi/180*data['absolutePosition']['th']
    rot_mat=[[math.cos(th),-math.sin(th)],[math.sin(th),math.cos(th)]]
    tr=data['odometryData']['tr']
    l=data['odometryData']['l']
    rw=data['odometryData']['rw']
    t1=data['odometryData']['t1']
    t2=data['odometryData']['t2']
    sonarIDtoAngle = [90, 50, 30, 10, -10, -30, -50, -90, -90, -130, -150, -170, 170, 150, 130, 90]
    son=data['SonarData']
    n_p = len(scatterX)
    """
    Program Loop

    - The time interval of this is 1 sec (1000ms) found in the FuncAnimation call below the function
    - This is called every iteration (once a second)

    Remember you need 3 things:
    	- Belief Probability Density Function (the particles you have)
    	- System Dynamics (you will need to find how the robot moved using position, then update the particles with this movement plus noise!)
    	- Perceptual Model (we have the occupancy map)

    So...
    	Particles are propogated according to the motion model (prediction), odometery
    """
    # odometery
    dtheta=2*math.pi*(t1-t2)*rw/(tr*l)
    dx=(t1+t2)*math.pi*rw/tr*math.cos(th)
    dy=(t1+t2)*math.pi*rw/tr*math.sin(th)
    
    scatterX+=dx
    scatterX=scatterX+np.random.normal(0,20,(n_p,1))
    scatterY+=dy
    scatterY=scatterY+np.random.normal(0,20,(n_p,1))
    
    """
    	They are weighted according to the likelihood of the observation (correction)
    	They are then resampled to create a new set of particles for the next iteration

    To get the probabilistic value from the occupancy map, use:

    	getOccupancyGridValue(xPosition, yPosition)

    """

    # Request data from the webserver and parse to a array
    res = urllib.request.urlopen(url).read()
    data = json.loads(res)

    # Get robots current position
    robotXPosition = data['absolutePosition']['x']
    robotYPosition = data['absolutePosition']['y']

    robotTrajectoryX.append(robotXPosition)
    robotTrajectoryY.append(robotYPosition)
    
    
    # measure environment
    Oc=pix[:,:,0]
    for i in range(n_p):
        if scatterX[i]<5000 and scatterY[i]<5000 and scatterX[i]>0 and scatterY[i]>0:
            rowy=math.floor(scatterY[i]/100)
            colx=math.floor(scatterX[i]/100)
            sum_diff = 0 
            for j in range(16):
                d=son[j]
                if j == 0 or j == 7 or j==8 or j==15:    
                    d=son[j] + 0.6
                Xj=[[d*math.cos(sonarIDtoAngle[j])],[d*math.sin(sonarIDtoAngle[j])]]
                Xr=np.matmul(rot_mat,Xj)
                Xt=[[scatterX[i]],[scatterY[i]]]+Xr
                colj=math.floor(Xt[0][0]/100)
                rowj=50-math.floor(Xt[1][0]/100)
                d_e=predict_sensor(colx,rowy,colj,rowj,Oc,100,size,5000)
                diffj=abs(d_e-d)
                sum_diff = sum_diff+diffj
            weights[i] = 1-sum_diff
        else:
            if scatterX[i]>5000 or scatterX[i]<0:
                scatterX[i] = np.random.rand(1,1)
                scatterX[i] = scatterX[i] * 5000
            if scatterY[i]>5000 or scatterY[i]<0:
                scatterY[i] = np.random.rand(1,1)
                scatterY[i] = scatterY[i] * 5000
            sum_diff = 0 
            rowy=math.floor(scatterY[i]/100)
            colx=math.floor(scatterX[i]/100)
            for j in range(16):
                d = son[j]
                if j == 0 or j == 7 or j==8 or j==15:    
                    d=son[j] + 0.6
                Xj=[[d*math.cos(sonarIDtoAngle[j])],[d*math.sin(sonarIDtoAngle[j])]]
                Xr=np.matmul(rot_mat,Xj)
                Xt=[[scatterX[i]],[scatterY[i]]]+Xr
                colj=math.floor(Xt[0][0]/100)
                rowj=50-math.floor(Xt[1][0]/100)
                d_e=predict_sensor(colx,rowy,colj,rowj,Oc,100,size,5000)
                diffj=abs(d_e-d)
                sum_diff = sum_diff+diffj
            weights[i] = 1-sum_diff
    min_wei = min(weights)
    #max_wei = max(weights)
    weights -= min_wei
    #weights /= (max_wei-min_wei)
    norm = weights
    
    #resampling
    newX = scatterX
    newY = scatterY
    sum_wei = np.sum(norm)
    for i in range(n_p):
        if i <=100*n_p/100:
            rand_no= np.random.rand(1,1)*sum_wei
            wsum = 0
            for j in range(n_p):
                wsum = wsum + norm[j]
                if wsum > rand_no:
                    newX[i] = scatterX[j]
                    newY[i] = scatterY[j]
                    break
        else:
            newX[i]=5000*np.random.rand(1,1)
            newY[i]=5000*np.random.rand(1,1)
     

    """
    1. Update the particles using system dynamics (the prediction phase algorithm in the slides)
    2. Measure the environment for the particle using the sonar sensors (uses the occupancy map) and use this to update the weights
    3. Normalise weights for the particles

    
    """




    # Plotting
    # clear plot, add data to new figure
    ax.clear()

    #ax.pcolor(tn_image)
    for line in formattedLines:
        ax.plot([line['p1'][0], line['p2'][0]], [line['p1'][1], line['p2'][1]], c='r')

    """
    Draw particles here
    """

    ax.scatter(scatterX, scatterY, s=1, c='m')
    ax.scatter(x,y)
    #ax.plot(robotTrajectoryX, robotTrajectoryY, c='b')
    # Seting updating old points
    scatterX = newX
    scatterY = newY
    scatterX.dump("xp_matrix.dat")
    scatterY.dump("yp_matrix.dat")
    weights.dump("w_matrix.dat")



# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(pix,), interval=1000)
# show the plot

plt.show()


def predict_sensor(x0,y0,x1,y1,O,res,size,r):
    if abs(y1 - y0) < abs(x1 - x0):
        if x0>x1:
            dist=sensorLineLow(x1,y1,x0,y0,O,res,size,r)
        else:
            dist=sensorLineLow(x0,y0,x1,y1,O,res,size,r)
    else:
        if y0 > y1:
            dist=sensorLineHigh(x1,y1,x0,y0,O,res,size,r)
        else:
            dist=sensorLineHigh(x0,y0,x1,y1,O,res,size,r)
    if dist>=r:
        dist=r
    return dist

def sensorLineLow(x0,y0,x1,y1,O,res,size,r):
    x_max=size[1]
    y_max=size[0]
    dx = x1 - x0
    dy = y1 - y0
    yi = 1
    if dy < 0:
      yi = -1
      dy = -dy
    D = 2*dy - dx
    y = y0
    x = x0
    dist = 0
    while dist<r:
        if D > 0:
            y = y + yi
            D = D - 2*dx   
        D = D + 2*dy
        dist=math.sqrt((x-x0)**2+(y-y0)**2)*res
        if x>=x_max or y<=0 or y>=y_max:
            break
        elif O[y][x]==1:
            break
        x = x + 1
    return dist

def sensorLineHigh(x0,y0,x1,y1,O,res,size,r):
    x_max=size[1]
    y_max=size[0]
    dx = x1 - x0
    dy = y1 - y0
    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx
    D = 2*dx - dy
    x = x0
    y = y0
    dist = 0
    while dist<r:
        if D > 0:
            x = x + xi
            D = D - 2*dy
        D = D + 2*dx
        dist=math.sqrt((x-x0)**2+(y-y0)**2)*res
        if y>=y_max or x<=0 or x>=x_max:
            dist=r
            break
        elif O[y][x]==1:
            break
        y = y + 1
    return dist