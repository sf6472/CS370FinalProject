import sys
import time
import RPi.GPIO as GPIO
from flask import Flask, render_template, request, url_for
from flask_socketio import SocketIO
import os
import matplotlib.pyplot as plt

app = Flask(__name__)
socketio = SocketIO(app)

UUID = "10000000814ae5ed"

commForward = 17
commBackward = 27
commLeft = 23
commRight = 24
arduinoStat = 25
obstacleFront = 5
obstacleBack = 6
obstacleLeft = 19
obstacleRight = 26
speaker = 12
temp1 = 1

distancePluse = 22
pluseCount = 0

distanceTravelled = 0

Direction = ""              #direction "switch"
totalPosition = []          #to save position for every step
currPosition = [0,0]        #current position
totalBlock = []             #to save obstacles location for every step
imgAr = []
def init():
    GPIO.cleanup()          #clean GPIO up first to prevent errors.
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(commForward,GPIO.OUT)
    GPIO.setup(commBackward,GPIO.OUT)
    GPIO.setup(commLeft,GPIO.OUT)
    GPIO.setup(commRight,GPIO.OUT)

    GPIO.setup(arduinoStat, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(obstacleFront, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(obstacleBack, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(obstacleLeft, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(obstacleRight, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(speaker,GPIO.OUT)

    GPIO.setup(distancePluse, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(distancePluse, GPIO.RISING, callback = rotaryInterrupt)

    GPIO.output(commForward,GPIO.LOW)
    GPIO.output(commBackward,GPIO.LOW)

    buzzer = GPIO.PWM(speaker, 1000) # Set frequency to 1 Khz
    buzzer.start(10) # Set dutycycle to 10
    
    if GPIO.input(arduinoStat) == 0:
        print('Check MCU')
    else:
        print('initialized')
    
    
def rotaryInterrupt(w):
    global pluseCount
    pluseCount += 1
    print(pluseCount)

    global totalPosition    
    global currPosition
    global Direction
    global totalBlock
    global imgAr

    currBlock = [GPIO.input(obstacleFront),     #obstacles for current location
                GPIO.input(obstacleBack), 
                GPIO.input(obstacleLeft), 
                GPIO.input(obstacleRight)]

    totalPosition.append(currPosition)          #save current location
    totalBlock.append(currBlock)                #save obstacles for current location

    if Direction == "Forward":
        currPosition = [currPosition[0] + 1, currPosition[1]]
    elif Direction == "Backeard":
        currPosition = [currPosition[0] - 1, currPosition[1]]
    elif Direction == "Left":
        currPosition = [currPosition[0], currPosition[1] - 1]
    elif Direction == "Right":
        currPosition = [currPosition[0], currPosition[1] + 1]
    else:
        print("Direcction Error!")
        #exit(1)
    # simgAr[currPosition[0]][currPosition[1]] = 1
    
    return pluseCount

@app.route('/allStop')
def allStop():
    print("stop")
    GPIO.output(commForward,GPIO.LOW)
    GPIO.output(commBackward,GPIO.LOW)
    GPIO.output(commLeft,GPIO.LOW)
    GPIO.output(commRight,GPIO.LOW)
    templateData = {
        'title' : 'CS370 Final Project',
        'uuid' : UUID,
        'distanceTravelled' : pluseCount,
        'front' : GPIO.input(obstacleFront),
        'back' : GPIO.input(obstacleBack),
        'left' : GPIO.input(obstacleLeft),
        'right' : GPIO.input(obstacleRight),
        'curr' : currPosition,
    }
    return render_template('CarDrive.html', **templateData)

@app.route("/<direction>")
def move(direction):

    global Direction

    print(direction)
    if direction == "Forward":
        Direction = "Forward"   #get direction
        GPIO.output(commForward,GPIO.HIGH)
        GPIO.output(commBackward,GPIO.LOW)
        GPIO.output(commLeft,GPIO.LOW)
        GPIO.output(commRight,GPIO.LOW)
    if direction == "Backeard":
        Direction = "Backeard" 
        GPIO.output(commForward,GPIO.LOW)
        GPIO.output(commBackward,GPIO.HIGH)
        GPIO.output(commLeft,GPIO.LOW)
        GPIO.output(commRight,GPIO.LOW)
    if direction == "Left":
        Direction = "Left" 
        GPIO.output(commForward,GPIO.LOW)
        GPIO.output(commBackward,GPIO.LOW)
        GPIO.output(commLeft,GPIO.HIGH)
        GPIO.output(commRight,GPIO.LOW)
    if direction == "Right":
        Direction = "Right" 
        GPIO.output(commForward,GPIO.LOW)
        GPIO.output(commBackward,GPIO.LOW)
        GPIO.output(commLeft,GPIO.LOW)
        GPIO.output(commRight,GPIO.HIGH)
    templateData = {
        'title' : 'CS370 Final Project',
        'uuid' : UUID,
        'distanceTravelled' : pluseCount,
        'front' : GPIO.input(obstacleFront),
        'back' : GPIO.input(obstacleBack),
        'left' : GPIO.input(obstacleLeft),
        'right' : GPIO.input(obstacleRight),
        'curr' : currPosition,

    }
    return render_template('CarDrive.html', **templateData)

@app.route('/reset')
def reset():
    #global pluseCount   #to specify the one outside this function
    #pluseCount = 0
    global Direction
    global totalPosition
    global currPosition
    global totalBlock
    global imgAr

    Direction = ""
    totalPosition.clear()       #use clear first
    currPosition.clear()
    totalBlock.clear()
    imgAr.clear()
    totalPosition = []          #to save position for every step
    currPosition = [0,0]        #current position
    totalBlock = []             #to save obstacles location for every step
    imgAr = []

    templateData = {
    'title' : 'CS370 Final Project',
    'uuid' : UUID,
    'distanceTravelled' : pluseCount,
    'front' : GPIO.input(obstacleFront),
    'back' : GPIO.input(obstacleBack),
    'left' : GPIO.input(obstacleLeft),
    'right' : GPIO.input(obstacleRight),
    'curr' : currPosition,
    }
    return render_template('CarDrive.html', **templateData)



@app.route('/')
def hello():
    init()
    templateData = {
        'title' : 'CS370 Final Project',
        'uuid' : UUID,
        'distanceTravelled' : pluseCount,
        'front' : GPIO.input(obstacleFront),
        'back' : GPIO.input(obstacleBack),
        'left' : GPIO.input(obstacleLeft),
        'right' : GPIO.input(obstacleRight),
        'curr' : currPosition,

    }
    return render_template('CarDrive.html', **templateData)

@app.route('/showimg')
def showImg():
    global totalPosition
    global totalBlock
    global currPosition
    global imgAr
    imgAr.clear()   #clear it first to prevent repeated calculation

    xMax = -10000
    xMin = 10000
    yMax = -10000
    yMin = 10000

    for i in range(len(totalPosition)):
        xMax = max(xMax, totalPosition[i][0])
        xMin = min(xMin, totalPosition[i][0])
        yMax = max(yMax, totalPosition[i][1])
        yMin = min(yMin, totalPosition[i][1])

    for i in range(2 * (xMax - xMin) + 3):    #add 3 in case obstacle on both sides
        temp = []
        for j in range(2 * (yMax - yMin) + 3):      #add 3 in case obstacle on both sides
            temp.append(0)
        imgAr.append(temp)

    # for i in range(len(totalPosition)):
    #     x = totalPosition[i][0] - xMin + int(len(imgAr)/2)
    #     y = totalPosition[i][1] - yMin + int(len(imgAr[0])/2)
    #     imgAr[x][y]= 1
    
    for i in range(len(totalPosition)):
        x = totalPosition[i][0] - xMin + int(len(imgAr)/2)
        y = totalPosition[i][1] - yMin + int(len(imgAr[0])/2)
        if totalBlock[i][0] == 1:     #if front obstacle
            imgAr[x][y] = 1
            imgAr[x+1][y] = 2
        if totalBlock[i][1] == 1:     #if back obstacle
            imgAr[x][y] = 1
            imgAr[x-1][y] = 2
        if totalBlock[i][2] == 1:     #if left obstacle
            imgAr[x][y] = 1
            imgAr[x][y-1] = 2
        if totalBlock[i][3] == 1:     #if right obstacle
            imgAr[x][y] = 1
            imgAr[x][y+1] = 2
        else:                       #if no obstacle
            imgAr[x][y] = 1
    try:
        xCurr = currPosition[0] - xMin + int(len(imgAr)/2)      #Show current car location on map
        yCurr = currPosition[1] - yMin + int(len(imgAr[0])/2)
        imgAr[xCurr][yCurr] = 3
    except:
        print('nocurrPosition')

    # for i in range(len(totalPosition)):
    #     #for j in range(len(totalPosition[i])):
    #     imgAr[totalPosition[i][0]][totalPosition[i][1]] = 1
    #     # print(totalPosition[i][0])
    #     # print(totalPosition[i][1])
    try:
        if yMax > 0 and xMax > 0:
            plt.imshow(imgAr, cmap="YlGnBu", origin="lower", extent =[-yMax, abs(yMax), -xMax, abs(xMax)])
        else:   #a weird case
            ymi = 0 - abs(yMax) - 10
            yma = abs(yMax)
            xmi = 0 - abs(xMax) -10
            xma = abs(xMax)
            #plt.imshow(imgAr, cmap="YlGnBu", origin="lower")
            plt.imshow(imgAr, cmap="YlGnBu", origin="lower", extent =[ymi, yma, xmi, xma])
        #plt.plot(totalPosition[i])
        plt.savefig('/home/pi/Desktop/CS370FinalProject/server/app/static/map.png', bbox_inches='tight')
        plt.clf()   #clear after plot.
    except:
        plt.clf()   #clear after plot.
        print('no draw')

    templateData = {
    'title' : 'CS370 Final Project',
    'uuid' : UUID,
    'distanceTravelled' : pluseCount,
    'img' : url_for('static', filename='map.png'),
    'front' : GPIO.input(obstacleFront),
    'back' : GPIO.input(obstacleBack),
    'left' : GPIO.input(obstacleLeft),
    'right' : GPIO.input(obstacleRight),
    'curr' : currPosition,
    }
    return render_template('CarDrive.html', **templateData)

if __name__ == "__main__":
    #socketio.run(app, port=5000)
    app.run(host='0.0.0.0', port=5000, debug=True)



## if GPIO.input(obstacleFront) == 1:
## if GPIO.input(obstacleBack) == 1:
## if GPIO.input(obstacleLeft) == 1:
## if GPIO.input(obstacleRight) == 1:
## if GPIO.input(commForward) == 1:
## if GPIO.input(commBackword) == 1:
## if GPIO.input(commLeft) == 1:
## if GPIO.input(commRight) == 1:
## distance: pluseCount