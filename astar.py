class node:
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __repr__(self):
        return "[%s, g=%d, h=%d, f=%d]\n" % (self.position, self.g, self.h, self.f)

class astar:
    def __init__(self, heuristicFunction, successorFunction):
        if heuristicFunction == None:
            heuristicFunction = lambda position, goal: 1
        self.heuristicFunction = heuristicFunction

        if successorFunction == None:
            successorFunction = lambda world, position: []
        self.successorFunction = successorFunction

        self.sortingFunction = lambda n: n.f

    def find(self, world, start, goal):
        opened = [node(start)]
        closed = []

        return self.find_recursive(world, opened, closed, goal)

    def create_node(self, position, goal, parent = None):
        n = node(position, parent)
        n.g = parent.g + 1 if parent is not None else 1
        n.h = self.heuristicFunction(n.position, goal)
        n.f = n.g + n.h
        return n

    def filter_node(self, n, opened, closed):
        for op in opened:
            if n.position == op.position:
                return n.f < op.f

        for cl in closed:
            if n.position == cl.position:
                return n.f < cl.f

        return True

    def find_recursive(self, world, opened, closed, goal):
        if len(opened) == 0:
            return None

        opened = sorted(opened, key=self.sortingFunction)
        n = opened.pop(0)

        if n.position == goal:
            return n

        successors = self.successorFunction(world, n.position)
        successors = map(lambda position: self.create_node(position, goal, n), successors)
        
        successors = filter(lambda s: self.filter_node(s, opened, closed), successors)
        opened = list(opened) + list(successors)
        closed.append(n)

        return self.find_recursive(world, opened, closed, goal)
