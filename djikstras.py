"""

"""
import sys

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
                    if distance[new_vertex] >= distance[vertex] + matrix[vertex][new_vertex]:

                        distance[new_vertex] = distance[vertex] + matrix[vertex][new_vertex] # calculate the new distance
                        Q.update([new_vertex,distance[new_vertex]])
                        path[new_vertex] = vertex
        vertex = Q.lowest()[0]
    # print(path)

    result = []
    result.append(goal)

    vertex = goal
    while vertex != start:
        for key,value in path.items():
            if key == vertex:
                # print(value)
                vertex = value
                result.insert(0,value)

    return result

print(dijkstra(matrix = [[-1, 15, 10, -1, -1],
                   [15, -1, 11, -1,  6],
                   [10, 11, -1,  5, -1],
                   [-1, -1,  5, -1, -1],
                   [-1,  6, -1, -1, -1]])
                   )
