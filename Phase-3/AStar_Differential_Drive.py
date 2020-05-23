#!/usr/bin/env python
# coding: utf-8
'''

'''
import math
import numpy as np
import time as t
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches


rpm1 = 0
rpm2 = 0

class Node:
    def __init__(self, x, y,orientation):
        self.x = x
        self.y = y
        self.cost_to_come = math.inf
        self.total_cost = math.inf
        self.parent = None
        self.orientation = orientation
        self.UR = None
        self.UL = None

class Robot:
    def __init__(self, radius, clearance, start, goal,orientation):
        self.radius = radius
        self.clearance = clearance
        self.start = start
        self.goal = goal
        self.orientation = orientation
        
def get_min_node(queue):
    min_node = 0
    for node in range(len(queue)):
        if queue[node].total_cost < queue[min_node].total_cost:
            min_node = node
    return queue.pop(min_node)

def node_exists(x,y,orientation, queue):
    for node in queue:
        if node.x == x and node.y == y and node.orientation == orientation:
            return queue.index(node)
        else:
            return None
def new_round(number):
   return round(number * 2) / 2
       
def cost_to_go(start,goal):
    x1 = start[0]
    x2 = goal[0]
    y1 = start[1]
    y2 = goal[1]
    dist = math.sqrt(((x1-x2)**2)+((y1-y2)**2))
    
    #print(dist)
    return dist
        




def backtrack(node): #create list of parent node locations
    parentList = list()
    parent = node.parent
    while parent is not None:
        parentList.append(parent)
        parent = parent.parent
    return parentList


def check_viableX(point):
    if point >= -500+clearance+radius and point < 500-clearance-radius:
        return True
    else:
        print("Invalid")
        return False
    
def check_viableY(point):
    if point > -500+clearance+radius and point < 500-clearance-radius:
        return True
    else:
        print("Invalid")
        return False


    
def check_circle1(x,y):
    #y = 200 - y
    center = [-200, -300]
    dist = np.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2)
    
    if dist <= 100+radius+clearance:
        print("Don't go in the circle 1!")
        return False
    else:
        return True
def check_circle2(x,y):
    #y = 200 - y
    center = [0, 0]
    dist = np.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2)
    
    if dist <= 100+radius+clearance:
        print("Don't go in the circle 2!")
        return False
    else:
        return True
def check_circle3(x,y):
    #y = 200 - y
    center = [200, 300]
    dist = np.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2)
    
    if dist <= 100+radius+clearance:
        print("Don't go in the circle 3!")
        return False
    else:
        return True
def check_circle4(x,y):
    #y = 200 - y
    center = [200, -300]
    dist = np.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2)
    
    if dist <= 100+radius+clearance:
        print("Don't go in the circle 4!")
        return False
    else:
        return True
    
def check_square1(x,y):
    if y <= 75+radius+clearance and y >= -75-radius-clearance and x <= -325+radius+clearance and x >= -475-radius-clearance:
        print("Don't go in the square 1!")
        return False
    else:
        return True
def check_square2(x,y):
    if y <= 75+radius+clearance and y >= -75-radius-clearance and x >= 325-radius-clearance and x <= 475+radius+clearance:
        print("Don't go in the square 2!")
        return False
    else:
        return True
def check_square3(x,y):
    if y <= 375+radius+clearance and y >= 225-radius-clearance and x <= -125+radius+clearance and x >= -275-radius-clearance:
        print("Don't go in the square 3!")
        return False
    else:
        return True
    
def check_squares(x,y):
    one = check_square1(x,y)
    two = check_square2(x,y)
    three = check_square3(x,y)
    if one == True and two == True and three == True:
        return True
    else:
        return False

def check_circles(x,y):
    one = check_circle1(x,y)
    two = check_circle2(x,y)
    three = check_circle3(x,y)
    four = check_circle4(x,y)
    if one == True and two == True and three == True and four == True:
        return True
    else:
        return False


def plot_workspace(x_start,y_start,x_goal,y_goal):
    fig, ax = plt.subplots()
    fig.set_size_inches(7,7, forward=True)

    
    verts1=[(-475,75),(-325,75),(-325,-75),(-475,-75),(-475,75)]
    codes1 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path1 = Path(verts1, codes1)
    patch1 = patches.PathPatch(path1, facecolor='blue', lw=0)
    ax.add_patch(patch1)
    
    verts2=[(475,75),(325,75),(325,-75),(475,-75),(475,75)]
    codes2 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path2 = Path(verts2, codes2)
    patch2 = patches.PathPatch(path2, facecolor='blue', lw=0)
    ax.add_patch(patch2)
    
    verts3=[(-275,375),(-125,375),(-125,225),(-275,225),(-275,375)]
    codes3 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path3 = Path(verts3, codes3)
    patch3 = patches.PathPatch(path3, facecolor='blue', lw=0)
    ax.add_patch(patch3)
    
    ax.add_patch(patches.Circle((-200,-300), radius=100, color='blue', lw=1))
    ax.add_patch(patches.Circle((0,0), radius=100, color='blue', lw=1))
    ax.add_patch(patches.Circle((200,300), radius=100, color='blue', lw=1))
    ax.add_patch(patches.Circle((200,-300), radius=100, color='blue', lw=1))
    
    
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500,500)
    
    return fig,ax

def try_move(point, UL, UR, orientation):
    t = 0
    r = float(wheel_Diameter) / 2
    L = length
    dt = 0.1
    Xn = point[0]
    Yn = point[1]
    UL = UL * 100
    UR = UR * 100
    orientation = orientation%360
    if orientation < 0:
        orientation = orientation +360
    #print("orient:",orientation)
    Thetan = 3.14 * orientation/180
    #print("Thetan",Thetan)
    cost = 0
    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    while t<.5:
        t = t + dt
        Xs = Xn
        Ys = Yn
        #print(r,UL,UR)
        dx = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        dy = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Xn += dx
        Yn += dy
        cost = cost + math.sqrt(dx**2+dy**2)
        Thetan += (r / L) * (UR - UL) * dt
        if not check_viableX(Xn) or not check_viableY(Yn) or not check_circles(Xn,Yn) or not check_squares(Xn,Yn) :
            print("dont plot")
            return None,None,None,None,None
        plt.plot([Xs, Xn], [Ys, Yn], color="black")
    Thetan = 180 *(Thetan)/3.14
    Thetan = Thetan %360
    if Thetan < 0:
        Thetan = Thetan + 360
    #print("Thetan",Thetan)
    if Thetan == 360.0:
        Thetan = 0
    new_point = [Xn,Yn]
    x = Xn
    y = Yn
    if check_viableX(x) and check_viableY(y) and check_circles(x,y)and check_squares(x,y) :
        return new_point, cost, Thetan, UR,UL
    else:
        return None, None, None,None,None

def astar(fig,ax, robot,rpm1,rpm2):
    import matplotlib.pyplot as plt
    start=t.time()
    plt.ion()
    radius = robot.radius
    clearance = robot.clearance
    orientation = robot.orientation
    
    start_node_pos = robot.start
    goal_node_pos = robot.goal
    
    ax.scatter(start_node_pos[0],start_node_pos[1],color="green",s=5)
    ax.scatter(goal_node_pos[0],goal_node_pos[1],color="red",s=3)
    ax.add_patch(patches.Circle((goal_node_pos[0],goal_node_pos[1]), goal_thresh,fill = False, color='red'))
    
    start_node = Node(start_node_pos[0],start_node_pos[1],orientation)
    start_node.cost_to_come = 0
    start_node.total_cost = cost_to_go(start_node_pos,goal_node_pos)


    div = 50 #1000 must be evenly divisible by this number
    visitedNodes = np.zeros((int(1000/div),int(1000/div),12))
    queue = [start_node]
    
    moves = [[0,rpm1],[rpm1,0],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
    
    frame = 0
    
    while queue:
        current_node = get_min_node(queue)
        current_point = [current_node.x,current_node.y]
        orientation = current_node.orientation
        visitedNodes[int((current_node.x+500)/div)][int((current_node.y+500)/div)][int(orientation//30)]=1
        #print("cost to come: " +str(current_node.cost_to_come) + " total cost: " + str(current_node.total_cost))
        for move in moves:
            rpm1 = move[0]
            rpm2 = move[1]
            orientation = current_node.orientation
            #print(move, current_point, radius, clearance,orientation)
            new_point, cost_to_come,orientation, UR,UL = try_move(current_point, rpm1, rpm2, orientation)
            
            frame +=1
            if new_point is not None:
                if math.sqrt((new_point[0] - goal_node_pos[0])**2+(new_point[1] - goal_node_pos[1])**2)<= goal_thresh:
                    print("goal reached")
                    #print(new_point[0],new_point[1])
                    new_node = Node(new_point[0],new_point[1],int(orientation))
                    new_node.parent = current_node
                    new_node.UR = UR
                    new_node.UL = UL
                    return new_node.parent, new_node

                new_node = Node(new_point[0],new_point[1],int(orientation))
                new_node.parent = current_node
                new_node.UR = UR
                new_node.UL = UL
                
                
                #print(int(new_node.x),int(new_node.y),orientation,int(orientation//30))
                if visitedNodes[int((new_node.x+500)/div)][int((new_node.y+500)/div)][int(orientation//30)]== 0:
                    new_node.cost_to_come = cost_to_come + new_node.parent.cost_to_come
                    new_node.total_cost = new_node.cost_to_come + 1*cost_to_go(new_point,goal_node_pos)
                
                    #print("cost to come: " +str(new_node.cost_to_come) + " total cost: " + str(new_node.total_cost))
                    visitedNodes[int((new_node.x+500)/div)][int((new_node.y+500)/div)][int(orientation//30)]= 1
                    queue.append(new_node)
                    #draw arrow
                    #print(current_node.x,current_node.y,abs(new_node.x-current_node.x),abs(new_node.y-current_node.y))
                else:
                    print("REPEAT!")
                    node_exist_index = node_exists(new_point[0],new_point[0],orientation, queue)
                    if node_exist_index is not None:
                        temp_node = queue[node_exist_index]
                        if temp_node.total_cost > cost_to_come + new_node.parent.cost_to_come+1*cost_to_go(new_point,goal_node_pos):
                            temp_node.total_cost = cost_to_come + new_node.parent.cost_to_come+1*cost_to_go(new_point,goal_node_pos)
                            temp_node.parent = current_node
                if frame%200 == 0:
                    plt.draw()
                    plt.title(str(round(t.time()-start,2))+" seconds")
                    plt.pause(.0001)
                #print(t.time()-a)
            else:
                continue
        

    return None, None

#################################################
start = False
goal = False
orientation_check = False

#change these values for point/rigid robot
radius = (0.354/2)*100
wheel_Diameter = 0.076
length = 0.230
clearance = 5 #10 cm clearance

while start == False:
    x_start = input("Enter robot x position in cm : ")
    x_start = float(x_start)
    y_start = input("Enter robot y position in cm: ") 
    y_start = float(y_start)
    start = check_viableY(y_start)
    if start == True:
        start = check_viableX(x_start)
        if start ==True:
            start = check_circles(x_start,y_start)
            if start == True:
                start = check_squares(x_start,y_start)
            
                
    
while goal == False:
    x_goal = input("Enter goal x position in cm: ") 
    x_goal = float(x_goal)
    y_goal = input("Enter goal y position in cm: ") 
    y_goal = float(y_goal)
    goal = check_viableY(y_goal)
    if goal == True:
        goal = check_viableX(x_goal)
        if goal == True:
            goal = check_circles(x_goal,y_goal)
            if goal == True:
                goal = check_squares(x_goal,y_goal)
                


while orientation_check == False:
    orientation = input("Enter robot orientation in degrees :")
    if int(orientation)%30 == 0:
        orientation = int(orientation)
        orientation_check = True
    else:
        print("Enter a multiple of 30 degrees!")
    

 
rpm1_check = False
rpm2_check = False


while rpm1_check == False:
    rpm1 = input("Enter RPM1: ")
    if float(rpm1)>0:
        rpm1 = float(rpm1)
        rpm1_check = True
    else:
        print("RPM must be greater than 0!")   
        
while rpm2_check == False:
    rpm2 = input("Enter RPM2: ")
    if float(rpm2)>0:
        rpm2 = float(rpm2)
        rpm2_check = True
    else:
        print("RPM must be greater than 0!")           

        
start = t.time()

start_node = [x_start,y_start,orientation]
goal_node = [x_goal,y_goal,orientation]

goal_thresh = int(input("Enter goal threshold in cm: "))

robot1 = Robot(radius, clearance, start_node, goal_node,orientation)

fig,ax = plot_workspace(x_start,y_start,x_goal,y_goal)
plt.draw()
plt.pause(1)

parent, final_node = astar(fig,ax, robot1,rpm1,rpm2)
plt.pause(1)
print("Time to solve: " + str(t.time()-start) + " seconds")
if parent is not None:
    file1 = open("video1-rpms.txt","w") 
    
    parent_list = backtrack(parent)
    #print(final_node.UR,final_node.UL,final_node.x,final_node.y,final_node.orientation)
    #print(parent.UR,parent.UL,parent.x,parent.y,parent.orientation)
    
    current_node = final_node
    UR = current_node.UR
    UL = current_node.UL
    file1.write(str(UR)+","+str(UL)+"\n")
    orientation = parent.orientation
    t = 0
    r = float(wheel_Diameter) / 2
    L = length
    dt = 0.1
    Xn = parent.x
    Yn = parent.y
    #print(orientation)
    orientation = orientation%360
    if orientation < 0:
        orientation = orientation +360
    #print("orient:",orientation)
    Thetan = 3.14 * orientation/180
    #print("Thetan",Thetan)
    cost = 0
    while t<.5:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        plt.plot([Xs, Xn], [Ys, Yn], color="red")
    current_node = parent
    
    for parent in parent_list:
        #print(parent.UR,parent.UL,parent.x,parent.y,parent.orientation)
        UR = current_node.UR
        UL = current_node.UL
        file1.write(str(UR)+","+str(UL)+"\n")
        orientation = parent.orientation
        t = 0
        r = float(wheel_Diameter) / 2
        L = length
        dt = 0.1
        Xn = parent.x
        Yn = parent.y
        #print(orientation)
        orientation = orientation%360
        if orientation < 0:
            orientation = orientation +360
        #print("orient:",orientation)
        Thetan = 3.14 * orientation/180
        #print("Thetan",Thetan)
        cost = 0
        while t<.5:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            plt.plot([Xs, Xn], [Ys, Yn], color="red")
        current_node = parent
        
    file1.close() 
    plt.pause(100)
        
else:
    print("No path to goal point")




