import pygame
import numpy as np
from astar_point_rigid import *
import time


def triangleCoordinates(start, end, triangleSize = 5):
    
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi/2
    # print(math.atan2(start[1] - end[1], end[0] - start[0]))
    rad = math.pi/180

    coordinateList = np.array([[end[0],end[1]],
                              [end[0] + triangleSize * math.sin(rotation - 165*rad), end[1] + triangleSize * math.cos(rotation - 165*rad)],
                              [end[0] + triangleSize * math.sin(rotation + 165*rad), end[1] + triangleSize * math.cos(rotation + 165*rad)]])

    return coordinateList

###################################################
#                  Parameters 
###################################################
clearance = radius = 0
print('Enter Type of robot\n 1 -> point robot \n 2 -> rigid robot \
        \n Enter number :')

robotType = int(input())

if(robotType == 2):
    print("Enter cleareance")
    clearance = int(input())
    print("Enter radius")
    radius = int(input())

print('Enter step size of the robot')
stepSize = int(input())
print('Enter start location s1')
s1 = int(input())
print('Enter start location s2')
s2 = 200-int(input())

print('Enter goal location g1')
g1 = int(input())
print('Enter goal location g2')
g2 = 200-int(input())

# Start time of simulation
startTime = time.time()

# Step size of movement 
threshDistance = stepSize

# Angle between actions
threshAngle = 30

res = 1 #resolution of grid 
scale = 3 #scale of grid


startPosition = np.round((np.array([s1,s2]))/res)
goalPosition = np.round((np.array([g1,g2]))/res)
startOrientation = 0


pygame.init()

white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
yellow = (255,255,0)

size_x = math.ceil(300)
size_y = math.ceil(200)
gameDisplay = pygame.display.set_mode((size_x*scale,size_y*scale))


############################################################
#                 Display Obstacles
############################################################
circlePts = [225,50,25]

polygonPts =  np.array([[20,120],[25,185],[75,185],[100,150],[75,120],[50,150]])
polygonPts[:,1] = (size_y - polygonPts[:,1])

rhombusPts = np.array([[225,10],[200,25],[225,40],[250,25]])
rhombusPts[:,1] = (size_y - rhombusPts[:,1])

ellipsePts = np.array([110,80,80,40])


X =           np.array([95+10*math.cos(math.radians(60)), 95-75*math.cos(math.radians(30))+10*math.cos(math.radians(60)), 95-75*math.cos(math.radians(30)), 95])
Y = size_y -  np.array([30+10*math.sin(math.radians(60)), 30+75*math.sin(math.radians(30))+10*math.sin(math.radians(60)), 30+75*math.sin(math.radians(30)), 30])
rectPts = np.column_stack((X,Y))

pygame.draw.circle(gameDisplay, red, (circlePts[0]*scale,circlePts[1]*scale), circlePts[2]*scale)
pygame.draw.polygon(gameDisplay,red,scale*polygonPts)
pygame.draw.polygon(gameDisplay,red,scale*rhombusPts)
pygame.draw.ellipse(gameDisplay,red,scale*ellipsePts)
pygame.draw.polygon(gameDisplay,red,scale*rectPts)


############################################################
#          Draw Explored Nodes and solution path
############################################################
nodesExplored = {}
q = []

if(not isSafe(startPosition,res,clearance+radius) or not isSafe(goalPosition,res,clearance+radius)):
    basicfont = pygame.font.SysFont(None, 48)
    text = basicfont.render('Start or goal position must be in a valid workspace', True, (255, 0, 0), (255, 255, 255))
    textrect = text.get_rect()
    textrect.centerx = gameDisplay.get_rect().centerx
    textrect.centery = gameDisplay.get_rect().centery
 
    gameDisplay.blit(text, textrect)
    pygame.display.update()
    pygame.time.delay(2000)

else:
    print('Exploring nodes...')
    success,solution = generatePath(q,startPosition,startOrientation,goalPosition,nodesExplored,threshDistance,threshAngle,clearance+radius)
    print('Optimal path found')
    # End of simulation
    endTime= time.time()
    print("Total time taken for exploring nodes "+ str(endTime-startTime) +" seconds.")

    #############################################
    #      Drawing 
    #############################################
    if(success):
        draw = True
        while draw:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
            
            #draw nodesExplored
            for s in nodesExplored:
                if(nodesExplored[s].parent):
                    pt = nodesExplored[s].state[0:2]
                    ptParent = nodesExplored[s].parent.state[0:2]
                    x,y = pt*scale*res
                    x2,y2 = ptParent*scale*res

                    #draw explored nodes
                    pygame.draw.line(gameDisplay,white,(x2,y2),(x,y),1)
                    # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
                    # pygame.draw.circle(gameDisplay,green,(int(x2),int(y2)),2)
                    triangle = triangleCoordinates([x2,y2],[x,y],5)
                    pygame.draw.polygon(gameDisplay, green,[tuple(triangle[0]),tuple(triangle[1]),tuple(triangle[2])])

                #draw start and goal locations
                pygame.draw.rect(gameDisplay,blue,(startPosition[0]*res*scale,startPosition[1]*res*scale, \
                                 res*scale,res*scale))

                pygame.draw.circle(gameDisplay,blue,(int(goalPosition[0]*res*scale),int(goalPosition[1]*res*scale)), \
                                  math.floor(3*1.5*res*scale))

                pygame.draw.rect(gameDisplay,white,(goalPosition[0]*res*scale,goalPosition[1]*res*scale, \
                                 res*scale,res*scale))

                pygame.display.update()

           
            # draw solution path
            for i in range(len(solution)-1,-1,-1):
                pt = solution[i][0:2]
                x,y = pt[0]*scale*res,pt[1]*scale*res
                pygame.draw.circle(gameDisplay,red,(int(x),int(y)),4)
                pygame.display.update()
            pygame.time.delay(4000)
            draw = False

    else:
        basicfont = pygame.font.SysFont(None, 48)
        text = basicfont.render('Path can\'t be generated', True, (255, 0, 0), (255, 255, 255))
        textrect = text.get_rect()
        textrect.centerx = gameDisplay.get_rect().centerx
        textrect.centery = gameDisplay.get_rect().centery
     
        gameDisplay.blit(text, textrect)
        pygame.display.update()
        pygame.time.delay(2000)

pygame.quit()

