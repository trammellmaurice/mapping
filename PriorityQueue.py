class PriorityQueue(object):
    def __init__(self):
        self.queue = []

    def isIn(self, data):
        for thing in self.queue:
            if data == thing[0]:
                return True

    def update(self, data):
        if not self.isIn(data[0]):
            return False

        for thing in self.queue:
            if data[0] == thing[0]:
                thing[1] = data[1]
                return True


    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == 0

    # for inserting an element in the queue
    def insert(self, data):
        self.queue.append(data)

    # for popping an element based on Priority
    def lowest(self):
        try:
            min = 0
            for i in range(len(self.queue)):
                if self.queue[i][1] < self.queue[min][1]:
                    min = i
            item = self.queue[min]
            # del self.queue[min][1]
            return item
        except IndexError:
            print()
            exit()

Q = PriorityQueue()
Q.insert([1,12])
Q.insert([4,22])
Q.update([1,10])
print(Q.queue)
print(Q.lowest())
print(Q.queue)
