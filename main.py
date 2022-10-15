# Intro to AI

import sys


class Node:
    def __init__(self, data, parent, depth, heuristic=None):
        self.pre = parent
        self.data = data
        self.depth = depth
        self.f = heuristic  # Defaults to None

    def __repr__(self):
        """Formats the data, so it doesn't print out the memory address when outputting the list."""
        return f"{self.data}"

    def addChildren(self):
        """Expands the node in the self argument."""
        children = []
        depth = self.depth + 1
        if self.data[2] == 0:  # Checks which side boat is on
            oneChicken = [self.data[0] + 1, self.data[1], (self.data[2] + 1) % 2, self.data[3] - 1,
                          self.data[4], (self.data[5] + 1) % 2]
            twoChickens = [self.data[0] + 2, self.data[1], (self.data[2] + 1) % 2, self.data[3] - 2,
                           self.data[4], (self.data[5] + 1) % 2]
            oneWolf = [self.data[0], self.data[1] + 1, (self.data[2] + 1) % 2, self.data[3],
                       self.data[4] - 1, (self.data[5] + 1) % 2]
            chickWolf = [self.data[0] + 1, self.data[1] + 1, (self.data[2] + 1) % 2, self.data[3] - 1,
                         self.data[4] - 1, (self.data[5] + 1) % 2]
            twoWolves = [self.data[0], self.data[1] + 2, (self.data[2] + 1) % 2, self.data[3],
                         self.data[4] - 2, (self.data[5] + 1) % 2]
        else:
            oneChicken = [self.data[0] - 1, self.data[1], (self.data[2] + 1) % 2, self.data[3] + 1,
                          self.data[4], (self.data[5] + 1) % 2]
            twoChickens = [self.data[0] - 2, self.data[1], (self.data[2] + 1) % 2, self.data[3] + 2,
                           self.data[4], (self.data[5] + 1) % 2]
            oneWolf = [self.data[0], self.data[1] - 1, (self.data[2] + 1) % 2, self.data[3],
                       self.data[4] + 1, (self.data[5] + 1) % 2]
            chickWolf = [self.data[0] - 1, self.data[1] - 1, (self.data[2] + 1) % 2, self.data[3] + 1,
                         self.data[4] + 1, (self.data[5] + 1) % 2]
            twoWolves = [self.data[0], self.data[1] - 2, (self.data[2] + 1) % 2, self.data[3],
                         self.data[4] + 2, (self.data[5] + 1) % 2]

        # Using extend so append doesn't need to be used 5 times
        children.extend([Node(twoChickens, self, depth), Node(oneChicken, self, depth), Node(oneWolf, self, depth),
                         Node(chickWolf, self, depth), Node(twoWolves, self, depth)])

        return children


def addpq(object, queue):
    num = len(queue)
    x = 0
    queue2 = []
    while x < num:
        queue2.append(queue[x])
        x = x + 1
    queue2.append(object)
    if not queue:
        queue.append(object)
        return queue

    i = 0
    j = 0

    notadded = True
    while i < num:
        queue2[j] = queue[i]
        if object.f > queue[i].f and notadded:
            queue2[j] = object
            j = j + 1
            queue2[j] = queue[i]
            notadded = False
        j = j + 1
        i = i + 1
    return queue2


def poppq(queue):
    object = queue.pop(len(queue) - 1)
    return object


def heuristic(curr, goal):
    diff = abs(goal[0] - curr[0]) + abs(goal[1] - curr[1]) + abs(goal[2] - curr[2])
    return diff / 3


def heuristic2(curr):
    """Calculates f(n) = g(n) + h(n) with g(n), the path cost, being equal to the depth and h(n), the heuristic,
     being the total number of animals on the initial side - 1."""
    heur = curr[3] + curr[4] - 1
    return heur


def aStarChildren(currNode):
    """Expands the node in the self argument."""
    children = []
    # depth = self.depth + 1
    if currNode[2] == 0:  # Checks which side boat is on
        oneChicken = [currNode[0] + 1, currNode[1], (currNode[2] + 1) % 2, currNode[3] - 1,
                      currNode[4], (currNode[5] + 1) % 2]
        twoChickens = [currNode[0] + 2, currNode[1], (currNode[2] + 1) % 2, currNode[3] - 2,
                       currNode[4], (currNode[5] + 1) % 2]
        oneWolf = [currNode[0], currNode[1] + 1, (currNode[2] + 1) % 2, currNode[3],
                   currNode[4] - 1, (currNode[5] + 1) % 2]
        chickWolf = [currNode[0] + 1, currNode[1] + 1, (currNode[2] + 1) % 2, currNode[3] - 1,
                     currNode[4] - 1, (currNode[5] + 1) % 2]
        twoWolves = [currNode[0], currNode[1] + 2, (currNode[2] + 1) % 2, currNode[3],
                     currNode[4] - 2, (currNode[5] + 1) % 2]
    else:
        oneChicken = [currNode[0] - 1, currNode[1], (currNode[2] + 1) % 2, currNode[3] + 1,
                      currNode[4], (currNode[5] + 1) % 2]
        twoChickens = [currNode[0] - 2, currNode[1], (currNode[2] + 1) % 2, currNode[3] + 2,
                       currNode[4], (currNode[5] + 1) % 2]
        oneWolf = [currNode[0], currNode[1] - 1, (currNode[2] + 1) % 2, currNode[3],
                   currNode[4] + 1, (currNode[5] + 1) % 2]
        chickWolf = [currNode[0] - 1, currNode[1] - 1, (currNode[2] + 1) % 2, currNode[3] + 1,
                     currNode[4] + 1, (currNode[5] + 1) % 2]
        twoWolves = [currNode[0], currNode[1] - 2, (currNode[2] + 1) % 2, currNode[3],
                     currNode[4] + 2, (currNode[5] + 1) % 2]

    # Using extend so append doesn't need to be used 5 times
    children.extend([oneChicken, twoChickens, oneWolf, chickWolf, twoWolves])

    return children


def checkFron(child, explored):
    """Checks to see if the child is already in the explored list."""
    for c in explored:
        if c.data == child:
            return False

    return True


def checkFron2(child, explored, depth):
    for c in explored:
        if c.data == child and depth >= c.depth:
            return False
    return True


def isValid(child):
    """Makes sure the move is a legal one or one that doesn't end in game over."""
    for x in child:
        if x < 0:
            return False
    if child[1] > child[0] and child[0] != 0:
        return False
    if child[4] > child[3] and child[3] != 0:
        return False
    return True


def astar(start, goal, hMode):
    counter = 0
    frontier = []
    explored = []

    # initiate the frontier with the root node
    if hMode == 1:
        costEstimate = heuristic(start, goal)
    else:
        costEstimate = heuristic2(start)
    frontier = addpq(Node(start, None, 0, costEstimate), frontier)

    i = 0
    while i < 1:
        if not frontier:
            print("It's impossible to get to the goal state")
            return False

        currNode = poppq(frontier)
        #print(f"Current Node: {currNode.data}\tCurrent Cost: {currNode.depth}\tFrontier: {currNode.f}")
        counter += 1
        explored.append(currNode)

        if currNode.data == goal:
            counter -= 1
            print("This is the solution path: ")
            buffer1 = []
            prinNode = currNode
            while prinNode.pre:
                buffer1.append(prinNode.data)
                prinNode = prinNode.pre
            buffer1.append(prinNode.data)
            a = len(buffer1) - 1
            buffer2 = []
            while a >= 0:
                buffer2.append(buffer1[a])
                a -= 1
            print(buffer2)
            print("Number of nodes expanded: ", counter)
            return buffer2
            #print(len(buffer2))
            # code here to print out path and number of nodes expanded
            #return True

        # expand the chosen node
        children = aStarChildren(currNode.data)
        for child in children:
            valid = isValid(child)
            if valid and checkFron(child, explored):
                if hMode == 1:
                    costEstimate = heuristic(child, goal)
                else:
                    costEstimate = heuristic2(child)
                frontier = addpq(Node(child, currNode, currNode.depth + 1, costEstimate + currNode.depth), frontier)
                explored.append(Node(child, currNode, currNode.depth + 1, costEstimate + currNode.depth))


def bfs(start, goal):
    """Performs depth-first or breadth-first search. The mode argument manages the data as a stack or queue depending
    on if the search algorithm being used is DFS OR BFS."""
    frontier = []  # nodes seen but not explored
    explored = []  # nodes explored
    expandedNodes = 0
    currNode = Node(start, None, 0)
    frontier.append(currNode)

    while frontier:
        currNode = frontier.pop(0)
        explored.append(currNode)

        if currNode.data == goal:
            solutionPath = []
            prinNode = currNode
            while prinNode.pre:  # Puts the path in the solutionPath list
                solutionPath.append(prinNode)
                prinNode = prinNode.pre
            solutionPath.append(prinNode)
            print(f"This is the solution path:\n{solutionPath[::-1]}")  # Reverses list
            print(f"Depth reached {solutionPath[0].depth}")
            print(f"Number of nodes expanded: {expandedNodes}")
            #print(len(solutionPath))
            return solutionPath[::-1]
            # return True

        children = currNode.addChildren()
        expandedNodes += 1  # This will only add when a node is expanded so no need for subtracting 1 at the end
        for child in children:
            valid = isValid(child.data)
            if valid and checkFron(child.data, explored):
                frontier.append(child)
                explored.append(child)

    print(f"No solution found")
    return False


def dfs(start, goal, maxDepth):
    """Performs depth-first or breadth-first search. The mode argument manages the data as a stack or queue depending
    on if the search algorithm being used is DFS OR BFS."""
    frontier = []  # nodes seen but not explored
    explored = []  # nodes explored
    expandedNodes = 0
    currNode = Node(start, None, 0)
    frontier.append(currNode)

    while frontier:
        currNode = frontier.pop()
        expandedNodes += 1
        explored.append(currNode)

        if currNode.data == goal:
            expandedNodes -= 1
            solutionPath = []
            prinNode = currNode
            while prinNode.pre: # Puts the path in the solutionPath list
                solutionPath.append(prinNode)
                prinNode = prinNode.pre
            solutionPath.append(prinNode)
            print(f"This is the solution path:\n{solutionPath[::-1]}")  # Reverses list
            #print("There are ", len(solutionPath), " nodes in the path")
            print(f"Depth reached {solutionPath[0].depth}")
            print(f"Number of nodes expanded: {expandedNodes}")
            return solutionPath[::-1]

        if currNode.depth +1 <= maxDepth:  # This is for the IDDFS function to keep track of depth
            children = currNode.addChildren()

            for child in children:
                valid = isValid(child.data)
                if valid and checkFron2(child.data, explored, currNode.depth + 1):
                    frontier.append(child)
                    explored.append(child)

    #print(f"No solution found at depth {maxDepth}")
    return False

def iddfs(start, goal):
    """Increments max depth and runs DFS to find the lowest depth a solution can be found with it."""
    depth = 0
    while True:
        solFound = dfs(start, goal, depth)
        if solFound:
            return depth, solFound
        depth += 1


def getData(fileName):
    """Gets the states from the txt files and turns them into lists."""
    state = []
    with open(fileName) as file:
        fileData = file.read().splitlines()
        for line in fileData:
            data = line.split(",")  # Separates at the comma, but turns numbers into strings
            data = [int(i) for i in data]  # Turns all the numbers into integers
            state += data  # Combines to one list instead of a list of lists
    return state


def fileOut(solution, output):
    """Outputs states to file."""
    with open(output, "w") as f:
        for state in solution:
            # Since we returned different values for our functions we have to use a try and except statement
            try:
                print(*state.data, file=f)
            except AttributeError:
                print(*state, file=f)


def main():
    #print(len(sys.argv))
    if len(sys.argv) < 5:
        print("Not enough commands!\n[initial state file] [goal state file] [mode] [output file] and optionally [heuristic mode (default 1)] ")
        exit()
    startFile = sys.argv[1]
    goalFile = sys.argv[2]
    mode = sys.argv[3]
    output = sys.argv[4]
    try:
        hMode = sys.argv[5]
    except IndexError:
        hMode = 1
    #print(len(sys.argv))
    #print(hMode)

    start = getData(startFile)
    goal = getData(goalFile)

    if mode.upper() == "BFS":
        print("Running BFS...\n")
        solution = bfs(start, goal)
        fileOut(solution, output)
        print("Complete")
    elif mode.upper() == "DFS":
        print("Running DFS...\n")
        solution = dfs(start, goal, sys.maxsize)
        fileOut(solution, output)
        print("Complete")
    elif mode.upper() == "IDDFS":
        print("Running IDDFS...\n")
        depth, solution = iddfs(start, goal)
        print(f"Solution found using iddfs at depth: {depth}")
        fileOut(solution, output)
        print("Complete")
    elif mode.upper() == "ASTAR":
        print("Running A*...\n")
        solution = astar(start, goal, hMode)
        fileOut(solution, output)
        print("Complete")
    else:
        print("Not a valid mode. The options are BFS, DFS, IDDFS, or ASTAR\n")


if __name__ == "__main__":
    main()
