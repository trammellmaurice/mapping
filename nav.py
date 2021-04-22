import sys
import math

def calcDistance(nodes, name1, name2):
    x1 = nodes[name1][0]
    y1 = nodes[name1][1]
    x2 = nodes[name2][0]
    y2 = nodes[name2][1]
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def parse(dot_file, start_x, start_y, goal_x, goal_y):
    f = open(dot_file, "r",encoding = "utf8")

    line = f.readline()
    while "graph" not in line:
        line = f.readline()

    #read in nodes
    nodes = {}
    node_nums = {}

    line = f.readline()
    count = 0
    while "label" in line:
        #get name and coordinates from line
        name = line.split(" ")[2]
        nodes[name] = [ float(line[(line.index("(") + 1):(line.index(","))]), float(line[(line.index(",") + 1):(line.index(")"))]) ]
        node_nums[name] = count
        count += 1
        line = f.readline()
    nodes["start"] = [start_x, start_y]
    nodes["goal"] = [goal_x, goal_y]
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

    return matrix

dot_file = sys.argv[1]

goal_coord = sys.argv[2].split(",")
goal_x = float(goal_coord[0])
goal_y = float(goal_coord[1])

if len(sys.argv) == 4:
    start_coord = sys.argv[3].split(",")
    start_x = float(start_coord[0])
    start_y = float(start_coord[1])
else:
    start_coord = r.getPositionTup()
    start_x = start_coord[0]
    start_y = start_coord[1]

print(parse(dot_file, start_x, start_y, goal_x, goal_y))
