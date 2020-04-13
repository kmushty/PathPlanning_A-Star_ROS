# usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import sys
import rospy
from geometry_msgs.msg import Twist

blank_list = []
for entries in range(4000000):
    entries = math.inf
    blank_list.append(entries)
cost_matrix = np.array(blank_list, dtype = object).reshape(200,200,100)

blank_list1 = []
for entries in range(4000000):
    entries = 0
    blank_list1.append(entries)
visited_mat = np.array(blank_list, dtype = object).reshape(200,200,100)

blank_list2 = []
for entries in range(4000000):
    entries = 0
    blank_list2.append(entries)
cost_2_come = np.array(blank_list, dtype = object).reshape(200,200,100)


def approximation(a):
    b = math.ceil(a)
    if b - a <= 0.75 and b - a >= 0.25:
        a = b - 0.5
    elif b - a > 0.75 and b - a <= 1:
        a = math.floor(a)
    elif b - a < 0.25 and b - a >=0:
        a = b
    else:
        return a 
    return a


def angle_approximation(a):
    a = int(a)
    a = a/10
    b = math.ceil(a)
    if b - a <= 0.75 and b - a >= 0.25:
        a = b - 0.5
        a = a*10
    elif b - a > 0.75 and b - a <= 1:
        a = math.floor(a)
        a = a*10
    elif b - a < 0.25 and b - a >= 0:
        a = b
        a = a*10
    else:
        a = a*10
        return a
    return a

orientation = 34        
k = angle_approximation(orientation)                   # real value 


# Everything in m.  obstacle space is 10*10 m. Converted in cm
R1 = 30
R2 = 60


#ind1 = 2  #int(input('enter x coordinate of starting node'))  # 5
#ind2 = 2  #int(input('enter y coordinate of starting node'))  # 5
ind = (5,5,k)

#goal1 = 20  #int(input('enter x coordinate of goal node'))  # 295
#goal2 = 20  #int(input('enter y coordinate of goal node'))  # 195 
goal = (60,60)

#d = int(input())



cost_2_come[int(2*ind[0])][int(2*ind[1])][int((ind[2])/5)] = 0


r = 0
c = 0

b = r+c

def obstacle_here(y,x):
    
    
    if y >= 22.5 and y<= 37.5 and x >= 12.5 and x <= 27.5:
        return True
    
    if y <= 2.5 and y<= 17.5 and x >= 37.5 and x <= 57.5:
        return True
    
    if y >= 82.5 and y <= 97.5 and x >= 37.5 and x >= 57.5:
        return True
    
    if (y-70)**2 + (x-20)**2 <= 100:
        return True
    
    if (y-50)**2 + (x-50)**2 <= 100:
        return True 
    
    if (y-30)**2 + (x-80)**2 <= 100:
        return True
    
    if (y-70)**2 + (x-80)**2 <= 100:
        return True
    
    

if obstacle_here(ind[0],ind[1]):
    print("start node is in the obstacle")
    sys.exit()
if obstacle_here(goal[0],goal[1]):
    print("goal node is in the obstacle")
    sys.exit()

    

def action_one(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*0)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R1)/60  
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j

        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 + u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 + u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)
    

def action_two(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*R1)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*0)/60
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
    
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)



def action_three(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*R1)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R1)/60  
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)


def action_four(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*0)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R2)/60    
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)
    
def action_five(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*R2)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*0)/60  
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)
    

def action_six(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*R2)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R2)/60

    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)



def action_seven(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*R1)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R2)/60   
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))

    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost ,(v,w)


def action_eight(i,j,k,R1,R2):
    l = 1.6
    r = 0.4
    dt = 1
    u1 = (2*3.14*r*R2)/60   #R1 and R2 diff for every subfunction
    u2 = (2*3.14*r*R1)/60   
    
    v = (r/2)*((u1/r)+ (u2/r))*0.1
    w = (r/l)*((u2/r) - (u1/r))
    
    theta = k 
    theta = angle_approximation(theta)
    point1 = i
    point2 = j
        
    for iterations in range(25):   # for 1 sec :- 0.05*20

        dj = (r/2)*(u1 +u2)*(math.cos(math.radians(theta))*dt)
        di = (r/2)*(u1 +u2)*(math.sin(math.radians(theta))*dt)
        d_theta = (r/l)*(u2-u1)*dt
        j = j + dj
        i = i + di
        k = k + d_theta
        if obstacle_here(i,j):
            return None
            
    i1 = approximation(i)
    j1 = approximation(j)
    
    if k > 360:
        k2 = int(k - 360)
        k1 = angle_approximation(k2)
        
    elif k < 0:
        k2 = 360 + k
        k1 = angle_approximation(k2)
        
    else:        
        k1 = angle_approximation(k)
        
    if j1 >= 100 or i1 >= 100 or i1 <= 0 or j1 <= 0:
        return None
    
    if obstacle_here(i1,j1):
        return None
    
    else:
        
        d = math.sqrt((i1 - point1)**2 + (j1 - point2)**2)
        heuristic = abs(goal[0] - i1) + abs(goal[1] - j1)
        cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] = cost_2_come[int(2*point1)][int(2*point2)][int(theta/5)] + d
        cost = cost_2_come[int(2*i1)][int(2*j1)][int(k1/5)] + heuristic
        
        return (i1,j1,k1) , cost , (v,w)




def get_neighbours(i,j,k):
    neighbours_cost = []
    index = []
    velocities = []
    
    one = action_one(i,j,k,R1,R2)
    if one is not None:
        neighbours_cost.append(one[1])
        index.append(one[0])
        velocities.append(one[2])
        
    two = action_two(i,j,k,R1,R2)
    if two is not None:
        neighbours_cost.append(two[1])
        index.append(two[0])
        velocities.append(two[2])
        
    three = action_three(i,j,k,R1,R2)
    if three is not None:
        neighbours_cost.append(three[1])
        index.append(three[0])
        velocities.append(three[2])
        
    four = action_four(i,j,k,R1,R2)
    if four is not None:
        neighbours_cost.append(four[1])
        index.append(four[0]) 
        velocities.append(four[2])
        
    five = action_five(i,j,k,R1,R2)
    if five is not None:
        neighbours_cost.append(five[1])
        index.append(five[0])
        velocities.append(five[2])
        
    six = action_six(i,j,k,R1,R2)
    if six is not None:
        neighbours_cost.append(six[1])
        index.append(six[0])
        velocities.append(six[2])
        
    seven = action_seven(i,j,k,R1,R2)
    if seven is not None:
        neighbours_cost.append(seven[1])
        index.append(seven[0]) 
        velocities.append(seven[2])
        
    eight = action_eight(i,j,k,R1,R2)
    if eight is not None:
        neighbours_cost.append(eight[1])
        index.append(eight[0])
        velocities.append(eight[2])
                     
    return neighbours_cost, index , velocities


'''def get_current_value(i,j):
    current_value = cost_matrix[i][j]
    return current_value'''

'''def locate_value_index(mat):
    i,j = np.where(mat == node)
    i = int(i)
    j = int(j)
    return (i,j)'''


def sort_list1(list_a,list_b):
    list_b = [i[1] for i in sorted(zip(list_a, list_b))]
    
def sort_list2(list_a):
    list_a.sort()
    

node_cost = 0
velocity_cost = (0,0)
cost_matrix[int(2*ind[0])][int(2*ind[1])][int(ind[2]/5)] = 0

explore_queue = []  
index_queue = []
velocities_queue = []
index_queue.append(ind)
explore_queue.append(node_cost)
velocities_queue.append(velocity_cost)

breakwhile = False

start_time = time.clock()

visited_list = []
parent_map = {}
velocity_map = {}
parent_map[ind] = None

while len(index_queue) != 0 and not breakwhile:
    node = index_queue[0]
    velocity = velocities_queue[0]
    visited_mat[int(2*node[0])][int(2*node[1])][int(node[2]/5)] = 1
    visited_list.append(node)
    
    velocity_map[node] = velocity

    #print(index_queue[0], explore_queue[0])
    index_queue.pop(0)
    explore_queue.pop(0)
    velocities_queue.pop(0)    
    pair = get_neighbours(node[0],node[1],node[2])
    #print('pair',pair)
    neighbours_cost = pair[0]
    index = pair[1]
    velocities = pair[2]
    #print(index)
    #print(index_queue)
    #print(neighbours_cost)
    
    #######
    if math.sqrt((goal[0] - node[0])**2 + (goal[1] - node[1])**2) <= 10:
        print("goal reached")
        goal_ind = node
        final_velocity = velocity
        print(final_velocity)
        breakwhile = True
    
    #######

    
    for i in range(len(index)):
        if not visited_mat[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)] == 1:    
                                        
            old_cost = cost_matrix[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)]
            if neighbours_cost[i] < old_cost:
                cost_matrix[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)] = neighbours_cost[i]
                #if old_cost != math.inf:
                    #ind_node = index_queue.index((index[i][0], index[i][1]))
                    #index_queue.pop(ind_node)
                    #explore_queue.pop(ind_node)
                
                   
                index_queue.append(index[i])
                explore_queue.append(cost_matrix[int(2*((index[i])[0]))][int(2*(index[i])[1])][int((index[i][2])/5)])  
                velocities_queue.append(velocities[i])
                parent_map[index[i]] = node
                


       
            
    sort_list1(explore_queue,index_queue)
    sort_list1(explore_queue,velocities_queue)
    sort_list2(explore_queue)
    visited_mat[int(2*node[0])][int(2*node[1])][int(node[2]/5)] == 1
   

end_time = time.clock()
print(end_time - start_time)

path_list = []
parent = parent_map[goal_ind]
path_list.append(goal_ind)
while parent is not None:
    path_list.append(parent)
    parent = parent_map[parent]

path_list.reverse()
print(path_list)

vel_path = []
for i in range(len(path_list)):
    vel = velocity_map[path_list[i]]
    vel_path.append(vel)

vel_path.pop(0)
vel_path.append(velocity_cost)
print(vel_path)



def talker(pub):
    msg = Twist()
    
    for i in range(len(vel_path)):
        msg.linear.x = vel_path[i][0]
        msg.angular.z = vel_path[i][1]
        pub.publish(msg)
        
if __name__ == '__main__':
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    rospy.init_node('vel_astar', anonymous = True)
    
    try:
        rate = rospy.Duration.time(25)
        while not rospy.is_shutdown():
            talker(pub)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
            
