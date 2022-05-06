import tkinter
import numpy as np
import time

# init tk
root = tkinter.Tk()

cWidth = 500
cHeight = 600

distRange = 350

# create canvas
myCanvas = tkinter.Canvas(root, bg="white", height=cHeight, width=cWidth)

def drawFrame():
    l = np.random.randint(350, size=(6,10))

    myCanvas.delete("all")

    xStart = cWidth/10
    xEnd = cWidth - cWidth/10
    xSpacing = (xEnd - xStart)/9
    yAxisRange = cHeight/9
    yAxisDiff = cHeight/27
    for i in range(6):
        # draw x and y axis
        yBase = (i+1) * yAxisRange + (i+1) * yAxisDiff
        yTop = yBase - yAxisRange
        # axis lines
        myCanvas.create_line( xStart, yBase, xEnd , yBase ) # xaxis
        myCanvas.create_line( xStart, yBase, xStart, yTop ) # yaxis
        myCanvas.create_line( xStart, yTop , xEnd, yTop)
        myCanvas.create_line( xEnd, yBase, xEnd , yTop )

        # find all points
        xP = []
        yP = []
        for j in range(len(l[0])):
            xCoord = xStart + xSpacing * j            
            yCoord = yBase - (l[i][j] * yAxisRange)/distRange
            xP.append(xCoord)
            yP.append(yCoord)
            if i == 0 :
                print(xCoord)
        for j in range(len(l[0])-1):
            myCanvas.create_line( xP[j], yP[j], xP[j+1], yP[j+1])




# add to window and show
myCanvas.pack()

drawFrame()
root.update()
#time.sleep(1)
drawFrame()
root.update()
#time.sleep(1)

drawFrame()
root.update()
#time.sleep(1)
drawFrame()
root.update()
#time.sleep(1)

drawFrame()
root.update()
#time.sleep(1)
drawFrame()
root.update()
#time.sleep(1)

drawFrame()
root.update()
#time.sleep(1)
drawFrame()
root.update()
#time.sleep(1)