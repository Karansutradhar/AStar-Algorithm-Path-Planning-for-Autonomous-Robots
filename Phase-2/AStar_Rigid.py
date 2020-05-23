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

class Node:
    def __init__(self, x, y,orientation):
        self.x = x
        self.y = y
        self.cost_to_come = math.inf
        self.total_cost = math.inf
        self.parent = None
        self.orientation = orientation

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
        
def try_move(move, current_point, radius, clearance,orientation):
    if move == 'up_30':
        return up_30(current_point, radius, clearance,orientation)
    if move == 'up_60':
        return up_60(current_point, radius, clearance,orientation)
    if move == 'down_30':
        return down_30(current_point, radius, clearance,orientation)
    if move == 'down_60':
        return down_60(current_point, radius, clearance,orientation)
    if move == 'forward':
        return forward(current_point, radius, clearance,orientation)
    

def draw_arrow(ax,x1,y1,x2,y2): #fill visited pixes
    ax.quiver(x1, y1, x2,y2,units='xy' ,scale=1)

def backtrack(node): #create list of parent node locations
    parentList = list()
    parent = node.parent
    while parent is not None:
        parentList.append(parent)
        parent = parent.parent
    return parentList


def check_viableX(point):
    if point >= 0 and point < 300:
        return True
    else:
        print("Invalid")
        return False
    
def check_viableY(point):
    if point > 0 and point < 200:
        return True
    else:
        print("Invalid")
        return False

def check_oval(x,y):
    center = [150, 100]
    rx = 40 + radius+clearance
    ry = 20 + radius+clearance
    dist = (((x - center[0]) ** 2) / (rx ** 2)) + (((y - center[1]) ** 2) / (ry ** 2))
    if dist <= 1:
        print("Don't go in the oval!")
        return False
    else:
        return True
    
    
def check_circle(x,y):
    #y = 200 - y
    center = [225, 150]
    dist = np.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2)
    
    if dist <= 25+radius+clearance:
        print("Don't go in the circle!")
        return False
    else:
        return True
    
def check_rectangle(x,y):
    #y = 200-y
    one = [95,200-170]
    two = [30.5,200-132.5]
    three = [35.5,200-123.9]
    four = [100,200-161.4]
    
    m_bottom = (two[1]-one[1])/(two[0]-one[0])
    m_top = (three[1]-four[1])/(three[0]-four[0])
    m_right = (one[1]-four[1])/(one[0]-four[0])
    m_left = (two[1]-three[1])/(two[0]-three[0])
    
    #solve for y intercept  y=mx+b or b = y-mx
    b_bottom = one[1]-m_bottom*one[0]
    b_top = three[1]-m_top*three[0]
    b_left = two[1]-m_left*two[0]
    b_right = four[1]-m_right*four[0]
    
    # 0 = y -mx -b
    eq_top = y - m_top*x-(b_top+radius+clearance+1) #+1 for rounding error
    eq_bottom = y - m_bottom*x -(b_bottom-radius-clearance)
    eq_left = y -m_left*x-(b_left+radius+clearance+2) # +2 is from rounding
    eq_right = y - m_right*x -(b_right-radius-clearance)
    
    if eq_top <= 0 and eq_bottom >= 0 and eq_left <= 0 and eq_right >= 0:
        print("Don't go in the rectangle!")
        return False
    else:
        return True

def check_diamond(x,y):
    #y = 200-y
    one = [225,200-190]
    two = [200,200-175]
    three = [225,200-160]
    four = [250,200-175]
    
    m_bottom = (two[1]-one[1])/(two[0]-one[0])
    m_top = (three[1]-four[1])/(three[0]-four[0])
    m_right = (one[1]-four[1])/(one[0]-four[0])
    m_left = (two[1]-three[1])/(two[0]-three[0])
    
    #solve for y intercept  y=mx+b or b = y-mx
    b_bottom = one[1]-m_bottom*one[0]
    b_top = three[1]-m_top*three[0]
    b_left = two[1]-m_left*two[0]
    b_right = four[1]-m_right*four[0]
    
    # 0 = y -mx -b
    eq_top = y - m_top*x-(b_top+radius+clearance)
    eq_bottom = y - m_bottom*x -(b_bottom-radius-clearance)
    eq_left = y -m_left*x-(b_left+radius+clearance)
    eq_right = y - m_right*x -(b_right-radius-clearance)
    
    if eq_top <= 0 and eq_bottom >= 0 and eq_left <= 0 and eq_right >= 0:
        print("Don't go in the diamond!")
        return False
    else:
        return True
    

def check_polygon(x,y):
    #y = 200 - y
    one = [20,200-80]
    two = [25,200-15]
    three = [75,200-15]
    four = [100,200-50]
    five = [75,200-80]
    six = [50,200-50] 
    
    
    m_one = (one[1]-two[1])/(one[0]-two[0])
    m_two = (two[1]-three[1])/(two[0]-three[0])
    m_three = (three[1]-four[1])/(three[0]-four[0])
    m_four = (five[1]-four[1])/(five[0]-four[0])
    m_five = (six[1]-five[1])/(six[0]-five[0])
    m_six = (one[1]-six[1])/(one[0]-six[0])
    m_seven = (six[1]-three[1])/(six[0]-three[0])
    
    #solve for y intercept  y=mx+b or b = y-mx
    b_one = one[1]-m_one*one[0]
    b_two = two[1]-m_two*two[0]
    b_three = three[1]-m_three*three[0]
    b_four = four[1]-m_four*four[0]
    b_five = five[1]-m_five*five[0]
    b_six = six[1]-m_six*six[0]
    b_seven = six[1]-m_seven*six[0]
    
    # 0 = y -mx -b
    
    eq_one = y - m_one*(x+clearance+radius)-(b_one)
    eq_two = (y) - m_two*x -(b_two+clearance+radius) 
    eq_three = y -m_three*(x)-(b_three+clearance+radius)
    eq_four = y - m_four*x -(b_four-clearance-radius)
    eq_five = y - m_five*x - (b_five-clearance-radius)
    eq_six = y - (m_six*x) - (b_six-clearance-radius)
    eq_seven = y - m_seven*x-(b_seven) #interior line segment
    

    if eq_one <= 0 and eq_two <= 0 and eq_six>=0 and eq_seven >=0:
        print("Don't go in the polygon1!")
        return False
    
    if eq_three <= 0 and eq_four >= 0 and eq_five >= 0 and eq_seven <= 0:
        print("Don't go in the polygon2!")
        return False
    
    else:
        return True

def plot_workspace(x_start,y_start,x_goal,y_goal):
    fig, ax = plt.subplots()


    verts2=[(95,200-170),(30.5,200-132.5),(35.5,200-123.9),(100,200-161.4),(95,200-170)]
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
    
    verts3=[(225,200-190),(200,200-175),(225,200-160),(250,200-175),(225,200-190)]
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
    
    verts4=[(20,200-80),(25,200-15),(75,200-15),(50,200-50),(20,200-80)]
    codes4 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path4 = Path(verts4, codes4)
    patch4 = patches.PathPatch(path4, facecolor='blue', lw=0)
    ax.add_patch(patch4)
    
    verts5=[(50,200-50),(75,200-15),(100,200-50),(75,200-80),(50,200-50)]
    codes5 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path5 = Path(verts5, codes5)
    patch5 = patches.PathPatch(path5, facecolor='blue', lw=0)
    ax.add_patch(patch5)
    
    ax.add_patch(patches.Circle((225, 200-50), radius=25, color='blue', lw=2))
    ax.add_patch(patches.Ellipse((150, 200-100), 80, 40, 0, color='blue', lw=2))
    
    ax.set_xlim(0, 300)
    ax.set_ylim(0, 200)
    return fig,ax

def up_30(point, radius, clearance,orientation):
    x = point[0]
    y = point[1]
    cost = step_size
    
    dx = math.cos(math.radians(orientation+30))*step_size
    dy = math.sin(math.radians(orientation+30))*step_size
    new_point = [(x+dx),(y+dy)]
    x = new_point[0]
    y = new_point[1]
    if check_viableX(x) and check_viableY(y) and check_circle(x,y) and check_oval(x,y) and check_rectangle(x,y) and check_diamond(x,y)and check_polygon(x,y):
        
        return new_point, cost, orientation+30
    else:
        return None, None,None


def up_60(point, radius, clearance,orientation):
    x = point[0]
    y = point[1]
    cost = step_size
    
    dx = math.cos(math.radians(orientation+60))*step_size
    dy = math.sin(math.radians(orientation+60))*step_size
    new_point = [(x+dx),(y+dy)]
    x = new_point[0]
    y = new_point[1]
    if check_viableX(x) and check_viableY(y) and check_circle(x,y) and check_oval(x,y) and check_rectangle(x,y)and check_diamond(x,y)and check_polygon(x,y):
        return new_point, cost, orientation+60
    else:
        return None, None,None


def down_30(point, radius, clearance,orientation):
    x = point[0]
    y = point[1]
    cost = step_size
    
    dx = math.cos(math.radians(orientation-30))*step_size
    dy = math.sin(math.radians(orientation-30))*step_size
    new_point = [(x+dx),(y+dy)]
    x = new_point[0]
    y = new_point[1]
    if check_viableX(x) and check_viableY(y) and check_circle(x,y) and check_oval(x,y)and check_rectangle(x,y)and check_diamond(x,y)and check_polygon(x,y):
        return new_point, cost, orientation-30
    else:
        return None, None,None


def down_60(point, radius, clearance,orientation):
    x = point[0]
    y = point[1]
    cost = step_size
    
    dx = math.cos(math.radians(orientation-60))*step_size
    dy = math.sin(math.radians(orientation-60))*step_size
    new_point = [(x+dx),(y+dy)]
    x = new_point[0]
    y = new_point[1]
    if check_viableX(x) and check_viableY(y) and check_circle(x,y) and check_oval(x,y)and check_rectangle(x,y)and check_diamond(x,y)and check_polygon(x,y):
        return new_point, cost, orientation-60
    else:
        return None, None,None

def forward(point, radius, clearance,orientation):
    x = point[0]
    y = point[1]
    cost = step_size
    dx = math.cos(math.radians(orientation))*step_size
    dy = math.sin(math.radians(orientation))*step_size
    new_point = [(x+dx),(y+dy)]
    x = new_point[0]
    y = new_point[1]
    if check_viableX(x) and check_viableY(y) and check_circle(x,y) and check_oval(x,y)and check_rectangle(x,y)and check_diamond(x,y)and check_polygon(x,y):
        return new_point, cost, orientation
    else:
        return None, None,None



def djikstra(fig,ax, robot):
    import matplotlib.pyplot as plt
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


    
    visitedNodes = np.zeros((600,400,12))
    queue = [start_node]
    
    moves = ["forward","up_30", "down_30", "up_60", "down_60"]
    
    frame = 0
    
    while queue:
        current_node = get_min_node(queue)
        current_point = [current_node.x,current_node.y]
        orientation = current_node.orientation
        visitedNodes[int(current_node.x*2)][int((current_node.y)*2)][int(orientation%30)]=1
        #print("cost to come: " +str(current_node.cost_to_come) + " total cost: " + str(current_node.total_cost))
        for move in moves:
            a=t.time()
            orientation = current_node.orientation
            #print(move, current_point, radius, clearance,orientation)
            new_point, cost_to_come,orientation = try_move(move, current_point, radius, clearance,orientation)
            
            frame +=1
            if new_point is not None:
                if math.sqrt((new_point[0] - goal_node_pos[0])**2+(new_point[1] - goal_node_pos[1])**2)<= goal_thresh:
                    print("goal reached")
                    #print(new_point[0],new_point[1])
                    new_node = Node(new_point[0],new_point[1],orientation)
                    new_node.parent = current_node
                    return new_node.parent, new_node

                new_node = Node(new_point[0],new_point[1],orientation)
                new_node.parent = current_node

                
                #print((new_round(new_node.x)*2),(new_round(new_node.y)*2),(orientation%30))
                if visitedNodes[int(new_round(new_node.x)*2)][int(new_round(new_node.y)*2)][int(orientation%30)]== 0:
                    new_node.cost_to_come = cost_to_come + new_node.parent.cost_to_come
                    new_node.total_cost = new_node.cost_to_come + 1*cost_to_go(new_point,goal_node_pos)
                    #print("cost to come: " +str(new_node.cost_to_come) + " total cost: " + str(new_node.total_cost))
                    visitedNodes[int(new_round(new_node.x)*2)][int(new_round(new_node.y)*2)][int(orientation%30)]= 1
                    queue.append(new_node)
                    #draw arrow
                    #print(current_node.x,current_node.y,abs(new_node.x-current_node.x),abs(new_node.y-current_node.y))
                    ax.arrow(current_node.x,current_node.y,new_node.x-current_node.x,new_node.y-current_node.y,length_includes_head=True,width = .001,head_width=.5, head_length=.5)
                if frame%200 == 0:
                    plt.draw()
                    plt.pause(.0001)
                else:
                    print("REPEAT!")
                    node_exist_index = node_exists(new_point[0],new_point[0],orientation, queue)
                    if node_exist_index is not None:
                        temp_node = queue[node_exist_index]
                        if temp_node.total_cost > cost_to_come + new_node.parent.cost_to_come+1*cost_to_go(new_point,goal_node_pos):
                            temp_node.total_cost = cost_to_come + new_node.parent.cost_to_come+1*cost_to_go(new_point,goal_node_pos)
                            temp_node.parent = current_node
                #print(t.time()-a)
            else:
                continue
        

    return None, None

#################################################
start = False
goal = False
orientation_check = False

#change these values for point/rigid robot
radius = 0
clearance = 0


while start == False:
    x_start = input("Enter robot x position : ")
    x_start = int(x_start)
    y_start = input("Enter robot y position : ") 
    y_start = int(y_start)
    start = check_viableY(y_start)
    if start == True:
        start = check_viableX(x_start)
        if start ==True:
            start = check_oval(x_start,y_start)
            if start == True:
                start = check_circle(x_start,y_start)
                if start == True:
                    start = check_rectangle(x_start,y_start)
                    if start == True:
                        start = check_diamond(x_start,y_start)
                        if start == True:
                            start = check_polygon(x_start,y_start)
    
while goal == False:
    x_goal = input("Enter goal x position : ") 
    x_goal = int(x_goal)
    y_goal = input("Enter goal y position : ") 
    y_goal = int(y_goal)
    goal = check_viableY(y_goal)
    if goal == True:
        goal = check_viableX(x_goal)
        if goal ==True:
            goal = check_oval(x_goal,y_goal)
            if goal == True:
                goal = check_circle(x_goal,y_goal)
                if goal == True:
                    goal = check_rectangle(x_goal,y_goal)
                    if goal == True:
                        goal = check_diamond(x_goal,y_goal)
                        if goal == True:
                            goal = check_polygon(x_goal,y_goal)


while orientation_check == False:
    orientation = input("Enter robot orientation in degrees :")
    if int(orientation)%30 == 0:
        orientation = int(orientation)
        orientation_check = True
    else:
        print("Enter a multiple of 30 degrees!")

radius_check = False
clearance_check = False
step_size_check = False
#radius,clearance,step size
while radius_check == False:
    radius = input("Enter robot radius (must be integer) :")
    radius = int(radius)
    radius_check = True
    
while clearance_check == False:
    clearance = input("Enter robot clearnace (must be integer) :")
    clearance = int(clearance)
    clearance_check = True
    
while step_size_check == False:
    step_size = input("Enter robot step size (1-10) :")
    if int(step_size) >=1 and int(step_size)<=10:
        step_size = int(step_size)
        step_size_check = True
    else:
        print("Enter a number between 1 and 10!")
        
start = t.time()

start_node = [x_start,y_start,orientation]
goal_node = [x_goal,y_goal,orientation]

goal_thresh = float(input("Enter goal threshold: "))

robot1 = Robot(radius, clearance, start_node, goal_node,orientation)

fig,ax = plot_workspace(x_start,y_start,x_goal,y_goal)


parent, final_node = djikstra(fig,ax, robot1)
plt.pause(1)
print("Time to solve: " + str(t.time()-start) + " seconds")
if parent is not None:
    parent_list = backtrack(parent)
    xend = final_node.x
    yend = final_node.y
    first_parent = final_node.parent
    x = first_parent.x
    y = first_parent.y
    ax.arrow(x,y,xend-x,yend-y,length_includes_head=True,head_width=.3, head_length=.5,color="red")
    xend = x
    yend = y
    for parent in parent_list:
        x = parent.x
        y = parent.y
        orientation = parent.orientation
        ax.arrow(x,y,xend-x,yend-y,length_includes_head=True,head_width=.3, head_length=.5,color="red")
        plt.draw()
        plt.pause(.01)
        xend = x
        yend = y
    plt.pause(10)
        
else:
    print("No path to goal point")




