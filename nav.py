import sys
import turtleAPI as turt
import Queue as queue
import math, rospy

def angle_pid(robot, goalX, goalY):
    global ang_error_queue, ang_error_sum, prev_ang_error
    
    pos = robot.getMCLPose()
    x = pos[0]
    y = pos[1]
    yaw = pos[2]
    dist_to_goal = distance(x, y, goalX, goalY)
    ang_to_goal = math.atan2((goalY-y)/dist_to_goal, (goalX-x)/dist_to_goal)
    print("angle to goal " + str(ang_to_goal))
    print("yaw " + str(yaw))
    ang_error = ang_to_goal - yaw
    while ang_error>math.pi:
        ang_error -= 2*math.pi
    while ang_error < -1*math.pi:
        ang_error += 2*math.pi
    print("error " + str(ang_error))
    kp = 0.2
    ki = 0.01
    kd = 0
    
    #update integral for last five steps
    if ang_error_queue.empty() == False and ang_error_queue.qsize() == 5:
        ang_error_sum -= ang_error_queue.get()
    ang_error_queue.put(ang_error)
    ang_error_sum += ang_error

    #update derivative
    ang_error_deriv = ang_error - prev_ang_error

    ut = kp * ang_error + ki * ang_error_sum + kd * ang_error_deriv

    prev_ang_error = ang_error

    return ut

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))

def dist_pid(robot,goalX,goalY):
    global prev_dist, dist_queue, dist_sum
    kp = .1
    ki = 0.001
    kd = 0.01
    curr = robot.getMCLPose()
    err = distance(curr[0],curr[1],goalX,goalY)
    P = kp*err
    dist_queue.append(err)
    dist_sum += err
    if len(dist_queue)>5:
        dist_sum -= dist_queue.pop(0)
    I = ki * dist_sum
    D = kd * (err - prev_dist)
    prev_dist = err
    Ut = P+I+D
    if Ut > .6:
        return .6
    return Ut
 
def dijkstra(matrix):

    start = len(matrix) - 2
    goal = len(matrix) - 1
    path  = {}

    class PriorityQueue(object):
        def __init__(self):
            self.queue = []

        # for checking if the queue is empty
        def isEmpty(self):
            return len(self.queue) == 0

        def isIn(self, data):
            for thing in self.queue:
                if data == thing[0]:
                    return True
            return False

        # for inserting an element in the queue
        def insert(self, data):
            self.queue.append(data)

        def update(self, data):
            if not self.isIn(data[0]):
                return False

            for thing in self.queue:
                if data[0] == thing[0]:
                    thing[1] = data[1]
                    return True

        # for popping an element based on Priority
        def lowest(self):
            try:
                min = 0
                for i in range(len(self.queue)):
                    if self.queue[i][1] < self.queue[min][1]:
                        min = i
                item = self.queue[min]
                del self.queue[min]
                return item
            except IndexError:
                print()
                exit()

    distance = [sys.maxsize] * len(matrix)
    visited = [False] * len(matrix)

    # create vertex priority queue
    Q = PriorityQueue()

    vertex = start
    visited[vertex] = True

    distance[vertex] = 0

    while vertex != goal:
        for new_vertex in range(0,len(matrix)):
            if matrix[vertex][new_vertex] != -1: # if its adjacent
                if not visited[new_vertex]: # if not visited
                    distance[new_vertex] = distance[vertex] + matrix[vertex][new_vertex] # calculate the new distance
                    visited[new_vertex] = True # visited
                    Q.insert([new_vertex,distance[new_vertex]])
                    path[new_vertex] = vertex

                else:
                    if distance[new_vertex] > distance[vertex] + matrix[vertex][new_vertex]:

                        distance[new_vertex] = distance[vertex] + matrix[vertex][new_vertex] # calculate the new distance
                        Q.update([new_vertex,distance[new_vertex]])
                        path[new_vertex] = vertex
        vertex = Q.lowest()[0]
#    print(path)

    result = []
    result.append(goal)

    vertex = goal
    while vertex != start:
        for key,value in path.items():
            if key == vertex:
#                print(value)
                vertex = value
                result.insert(0,value)

    return result

def calcDistance(nodes, name1, name2):
    x1 = nodes[name1][0]
    y1 = nodes[name1][1]
    x2 = nodes[name2][0]
    y2 = nodes[name2][1]
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def parse(dot_file, start_x, start_y, goal_x, goal_y):
    f = open(dot_file, "r")

    line = f.readline()
    while "graph" not in line:
        line = f.readline()

    #read in nodes
    nodes = {}
    node_nums = {}
    nodes_list = []

    line = f.readline()
    count = 0
    while "label" in line:
        #get name and coordinates from line
        name = line.split(" ")[2]
        nodes[name] = [ float(line[(line.index("(") + 1):(line.index(","))]), float(line[(line.index(",") + 1):(line.index(")"))]) ]
        nodes_list.append(nodes[name])
        node_nums[name] = count
        count += 1
        line = f.readline()
    nodes["start"] = [start_x, start_y]
    nodes_list.append([start_x, start_y])
    nodes["goal"] = [goal_x, goal_y]
    nodes_list.append([goal_x, goal_y])
    node_nums["start"] = count
    node_nums["goal"] = count + 1
    count += 2

    #skip empty lines
    while line == "\n":
        line = f.readline()

    #create adjacency matrix
    matrix = []
    for i in range(count):
        matrix.append([])
        for j in range(count):
            matrix[i].append(-1)

    while "--" in line:
        split = line.split(" ")
        name1 = split[2]
        name2 = split[4][0:(len(split[4]) - 2)]
        distance = calcDistance(nodes, name1, name2)
        matrix[node_nums[name1]][node_nums[name2]] = distance
        matrix[node_nums[name2]][node_nums[name1]] = distance
        line = f.readline()

    #find closest to start and goal
    min_dist_start = calcDistance(nodes, "start", "goal")
    min_dist_goal = min_dist_start
    min_name_start = "goal"
    min_name_goal = "start"

    for name in nodes.keys():
        if name != "start" and name != "goal":
            dist_start = calcDistance(nodes, "start", name)
            if dist_start < min_dist_start:
                min_dist_start = dist_start
                min_name_start = name

            dist_goal = calcDistance(nodes, "goal", name)
            if dist_goal < min_dist_goal:
                min_dist_goal = dist_goal
                min_name_goal = name

    matrix[node_nums["start"]][node_nums[min_name_start]] = min_dist_start
    matrix[node_nums[min_name_start]][node_nums["start"]] = min_dist_start
    matrix[node_nums["goal"]][node_nums[min_name_goal]] = min_dist_goal
    matrix[node_nums[min_name_goal]][node_nums["goal"]] = min_dist_goal

    return matrix, nodes_list

dot_file = sys.argv[1]

r = turt.robot()

goal_coord = sys.argv[2].split(",")
goal_x = float(goal_coord[0])
goal_y = float(goal_coord[1])

if len(sys.argv) == 4:
    start_coord = sys.argv[3].split(",")
    start_x = float(start_coord[0])
    start_y = float(start_coord[1])
else:
    start_coord = r.getMCLPose()
    start_x = start_coord[0]
    start_y = start_coord[1]

matrix, nodes_list = parse(dot_file, start_x, start_y, goal_x, goal_y)
print("adjacency matrix:")
print(matrix)
print
path = dijkstra(matrix)
print("path:")
print(path)
print

print("points:")
for v in path:
    print(nodes_list[v])
print

for i in len(path) - 1:
    ang_error_queue = queue.Queue()
    ang_error_sum = 0
    prev_ang_error = 0
    prev_dist = 0
    dist_sum = 0
    dist_queue = []

    goalX = path[i+1][0]
    goalY = path[i+1][1]
    if not (path[i][0] == goalX and path[i][1] == goalY):
        #turn to goal
        ang_error = angle_pid(r, goalX, goalY)
        while ang_error > 0.01 or ang_error < -0.01 and not rospy.is_shutdown():
            r.drive(ang_error, 0)
            ang_error = angle_pid(r, goalX, goalY)

        #drive to goal
        distance_error = dist_pid(r, goalX, goalY)
        while distance_error > 0.01 and not rospy.is_shutdown():
            ang_error = angle_pid(r, goalX, goalY)
            lin_error = dist_pid(r, goalX, goalY)
            r.drive(ang_error, lin_error)

        r.stop()
