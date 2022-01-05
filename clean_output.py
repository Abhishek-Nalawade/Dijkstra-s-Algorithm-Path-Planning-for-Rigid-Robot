import numpy as np
import cv2

out = cv2.VideoWriter('Dijsktra_output.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 60, (400,300))
#defining the queue class to use as a data structure
class queue():
    def __init__(self):
        self.pending = list()

    def add(self, child, ind):
        self.pending.insert(ind, child)

    def remove(self):
        if self.pending:
            return self.pending.pop()
        return None

    def peek(self):
        if self.pending:
            return self.pending[-1]

    def size1(self):
        return len(self.pending)

    def isempty(self):
        if self.pending == []:
            return True
        return False




#created node class in order to save current node and it's parent
class node():
    def __init__(self, current, parent): #, cost):
        self.current = current
        self.parent = parent


#defining obstacles as well as the boundaries of the map
#st[0] = y coordinate in cartesian space     st[1] = x coordinate in cartesian space
def obstacles(st):
    radius = 10
    clearance = 5
    cl = radius + clearance
    s1 = 0.7
    s2 = -1.42814
    x1 = np.arctan(s1)
    x2 = np.arctan(s2)
    d1 = np.cos(np.pi - x1)
    d2 = np.cos(np.pi - x2)
    a = -(cl / d1)
    b = -(cl / d2)
    if (((st[1]) - (90+1)) ** 2) + ((st[0] - (70+1)) ** 2) <= ((35+cl)**2):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in circle")
        return None

    elif (((st[1] - (246+1)) / (60+cl)) ** 2) + (((st[0] - (145+1)) / (30+cl)) ** 2) <= 1:
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in ellipse")
        return None

    elif (st[0] <= ((280+1) + cl) and st[1]>=((200+1)-cl) and st[0]>=((230+1)-cl) and st[1]<=((230+1)+cl)) and not (st[0]<=((270+1)-cl) and st[1]>=((210+1)+cl) and st[0]>=((240+1)+cl) and st[1]<=((230+1)+cl)):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (-0.7*st[1]+1*st[0])>=(73.4-a) and (st[0]+1.42814*st[1])>=(172.55-b) and (-0.7*st[1]+1*st[0])<=(99.81+a) and (st[0]+1.42814*st[1])<=(429.07+b):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in rectangle")
        return None

    elif (st[1] >= ((canvas_size[1]-1) - cl)) or (st[1] <= cl+1):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    elif (st[0] <= cl+1) or (st[0] >= ((canvas_size[0] -1) - cl)):
        canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    else :
        return st


def display_obstacles(st):
    radius = 0
    clearance = 0
    cl = radius + clearance
    s1 = 0.7
    s2 = -1.42814
    x1 = np.arctan(s1)
    x2 = np.arctan(s2)
    d1 = np.cos(np.pi - x1)
    d2 = np.cos(np.pi - x2)
    a = -(cl / d1)
    b = -(cl / d2)
    if (((st[1]) - (90+1)) ** 2) + ((st[0] - (70+1)) ** 2) <= ((35+cl)**2):
        display_canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in circle")
        return None

    elif (((st[1] - (246+1)) / (60+cl)) ** 2) + (((st[0] - (145+1)) / (30+cl)) ** 2) <= 1:
        display_canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in ellipse")
        return None

    elif (st[0] <= ((280+1) + cl) and st[1]>=((200+1)-cl) and st[0]>=((230+1)-cl) and st[1]<=((230+1)+cl)) and not (st[0]<=((270+1)-cl) and st[1]>=((210+1)+cl) and st[0]>=((240+1)+cl) and st[1]<=((230+1)+cl)):
        display_canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (-0.7*st[1]+1*st[0])>=(73.4-a) and (st[0]+1.42814*st[1])>=(172.55-b) and (-0.7*st[1]+1*st[0])<=(99.81+a) and (st[0]+1.42814*st[1])<=(429.07+b):
        display_canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is in rectangle")
        return None

    elif (st[1] >= ((canvas_size[1]-1) - cl)) or (st[1] < cl+1):
        display_canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    elif (st[0] <= cl+1) or (st[0] >= ((canvas_size[0] -1) - cl)):
        display_canvas[canvas_size[0]-1-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    else :
        return st


#removes from the queue
def removing_from_queue():

    check = queue1.remove()
    cs = duplicate_costqueue.pop()
    return check, cs

#checking if the node is in the queue or has been visited previously and then appending the parent to the visited_list
def check_if_visited(check, cs):
    nod = check.current        #checking with the red value of canvas
    if canvas[(canvas_size[0] - 1) - nod[0],nod[1],2] == 255:
        if duplicate_costcanvas[(canvas_size[0] - 1) - nod[0],nod[1]] > cs:
            ind = visited_child_list.index(check.current)
            visited_parent_list[ind] = check.parent
            visited_child_cost[ind] = cs

        return None
    canvas[(canvas_size[0] - 1) - nod[0], nod[1], 2] = 255    #marking visited by changing the color of red band
    display_canvas[(canvas_size[0] - 1) - nod[0], nod[1], 2] = 255
    duplicate_costcanvas[(canvas_size[0] - 1) - nod[0], nod[1],0] = cs
    visited_child_list.append(check.current)
    visited_parent_list.append(check.parent)
    visited_child_cost.append(cs)

    #out.write(canvas[1:301, 1:401])
    out.write(display_canvas[1:301, 1:401])

    return check, cs

#this function performs actions and gets children
def super_move_function(currentnode, cs):

    def moveleft(node1, effort1):
        child = node1.copy()
        child[1] = child[1] - 1
        effort1 = effort1 + 1
        return [child, effort1]

    def moveright(node1, effort1):
        child = node1.copy()
        child[1] = child[1] + 1
        effort1 = effort1 + 1
        return [child, effort1]

    def moveup(node1, effort1):
        child = node1.copy()
        child[0] = child[0] + 1
        effort1 = effort1 + 1
        return [child, effort1]

    def movedown(node1, effort1):
        child = node1.copy()
        child[0] = child[0] - 1
        effort1 = effort1 + 1
        return [child, effort1]

    def up_left(node1, effort1):
        child = node1.copy()
        child[0] = child[0] + 1
        child[1] = child[1] - 1
        effort1 = effort1 + (2)**(1/2)
        return [child, effort1]

    def down_left(node1, effort1):
        child = node1.copy()
        child[0] = child[0] - 1
        child[1] = child[1] - 1
        effort1 = effort1 + (2)**(1/2)
        return [child, effort1]


    def up_right(node1, effort1):
        child = node1.copy()
        child[0] = child[0] + 1
        child[1] = child[1] + 1
        effort1 = effort1 + (2)**(1/2)
        return [child, effort1]

    def down_right(node1, effort1):
        child = node1.copy()
        child[0] = child[0] - 1
        child[1] = child[1] + 1
        effort1 = effort1 + (2)**(1/2)
        return [child, effort1]

    new_child = list()
    node = currentnode.current
    effort = cs
    new_child.append(moveleft(node, effort))
    new_child.append(moveright(node, effort))
    new_child.append(moveup(node, effort))
    new_child.append(movedown(node, effort))
    new_child.append(up_left(node, effort))
    new_child.append(down_left(node, effort))
    new_child.append(up_right(node, effort))
    new_child.append(down_right(node, effort))


    return new_child, node

#checking if the node is in obstacle space and returns ones which are not in the obstacle space in a list and the parent node
def check_if_in_obstacle_space(children, parent1):
    valid_children = list()
    for i in children:
        if canvas[(canvas_size[0] - 1) - i[0][0], i[0][1], 0] == 255:
            continue
        valid_children.append(i)

    return valid_children, parent1


#compares new children with goal state and adds them to the queue if the child is not the goal state
def compare_with_goal(ultimate_children, parent1):
    for child in ultimate_children:
        if child[0] == goal:
            print("\n Goal has been reached \n")
            return child[0], parent1, child[1]
        else:
            duplicate_costqueue.append(child[1])
            duplicate_costqueue.sort(reverse = True)
            index_to_append_in_queue = duplicate_costqueue.index(child[1])
            queue1.add(node(child[0], parent1), index_to_append_in_queue)

    return None


#main body of the code from this line and below
canvas_size = [302,402, 3]
canvas = np.zeros((canvas_size[0],canvas_size[1], canvas_size[2]))
visited_child_list = list()
visited_parent_list = list()
visited_child_cost = list()
duplicate_costqueue = list()
display_canvas = np.zeros((canvas_size[0],canvas_size[1], canvas_size[2]))
duplicate_costcanvas = np.zeros((canvas_size[0],canvas_size[1], 1))
canvas = canvas.astype(np.uint8)
display_canvas = display_canvas.astype(np.uint8)
for_frames = list()

#marking obstacles
for i in range(canvas_size[0]):
    for j in range(canvas_size[1]):
        obstacles([i,j])
        display_obstacles([i,j])


#taking the start and goal node from the user and checking if in obstacle space
n = 1
while n > 0:
    start = list()
    goal = list()
    x1 = input("Enter the x co-ordinate of the start point: ")
    y1 = input("Enter the y co-ordinate of the start point: ")
    x2 = input("Enter the x co-ordinate of the goal point: ")
    y2 = input("Enter the y co-ordinate of the goal point: ")
    start.append(int(y1)+1)
    start.append(int(x1)+1)
    goal.append(int(y2)+1)
    goal.append(int(x2)+1)
    lis = [start, goal]
    strt = list()
    count = 0
    for i in lis:
        strt.append(obstacles(i))
    if strt[0] == None or strt[1] == None:
        print("Error: One of the entered point is either in obstacle space or out of map boundary")
        continue
    else:
        n = 0
    print(start, goal)


first_node = node(start, None)
queue1 = queue()
queue1.add(first_node,0)
duplicate_costqueue.append(0)

canvas[canvas_size[0] - goal[0], goal[1],:] = 128
display_canvas[canvas_size[0] - goal[0], goal[1],:] = 128

#calling the main functions of the Dijkstra
while True:
    new_parent = None
    while new_parent is None:
        new_node, cos = removing_from_queue()
        new_parent= check_if_visited(new_node, cos)
    children_list, parent = super_move_function(new_parent[0], new_parent[1])
    filtered_children, same_parent = check_if_in_obstacle_space(children_list, parent)
    child_parent = compare_with_goal(filtered_children, same_parent)
    if child_parent is not None:
        break

visited_child_list.append(child_parent[0])
visited_parent_list.append(child_parent[1])
visited_child_cost.append(child_parent[2])
parent_info = child_parent[1]


#searching for the route to the goal node or backtracking
route = list()

while parent_info is not None:
    for i in visited_child_list:
        if parent_info == i:
            parent_info = visited_parent_list[visited_child_list.index(i)]
            canvas[(canvas_size[0] - 1) - i[0],i[1],:] = 255
            # display_canvas[(canvas_size[0] - 1) - i[0],i[1],:] = 255
            cv2.circle(display_canvas, (i[1], (canvas_size[0] - 1) - i[0]), 1, (255,255,255), -1)
            #out.write(canvas[1:301, 1:401])
            out.write(display_canvas[1:301, 1:401])
            route.append(i)
            break


canvas[canvas_size[0] - goal[0], goal[1],:] = 128
cv2.imshow("Window",canvas[1:301, 1:401])


out.release()
print("\nVideo file of visualization has been saved")
cv2.waitKey(0)
cv2.destroyAllWindows()
