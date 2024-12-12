# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    def dfshelper(problem, state, visited_states, directions):

        visited_states.add(state)
        if problem.isGoalState(state):
            return directions

        for successor in problem.getSuccessors(state):
            if successor[0] not in visited_states:
                directions.append(successor[1])
                if dfshelper(problem, successor[0], visited_states, directions):
                    return directions
                else:
                    directions.pop()

        return 0

    visited_states = set()
    directions = []
    return dfshelper(problem, problem.getStartState(), visited_states, directions)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited_states = set()
    directions = []

    queue = util.Queue()
    queue.push(problem.getStartState())
    visited_states.add(problem.getStartState())

    queue1 = util.Queue()
    queue1.push(directions)

    solution = []

    while not queue.isEmpty():
        state = queue.pop()
        directions = queue1.pop()

        if problem.isGoalState(state):
            return directions

        for successor in problem.getSuccessors(state):
            if successor[0] not in visited_states:
                tmp = []
                tmp.append(successor[1])
                tmp1 = directions + tmp
                queue.push(successor[0])
                queue1.push(tmp1)
                visited_states.add(successor[0])

    return solution

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited_states = set()

    dij = {}
    dij[problem.getStartState()] = 0

    dir = {}
    dir[problem.getStartState()] = []

    solutin = []
    sol_value = 0

    while True:

        node = problem.getStartState()
        value = -1
        directions = []
        for state in dij:
            if state not in visited_states:
                if value == -1:
                    node = state
                    value = dij[state]
                    directions = dir[state]
                else:
                    if value > dij[state]:
                        node = state
                        value = dij[state]
                        directions = dir[state]

        if value == -1: break
        visited_states.add(node)

        l = []
        l1 = []
        for successor in problem.getSuccessors(node):
            if problem.isGoalState(successor[0]):
                l.append(successor)
            else:
                l1.append(successor)

        l2 = l + l1
        for successor in l2:

            if problem.isGoalState(successor[0]):
                # print("goal ===========", successor[0], "------ -", directions)
                tmp = []
                tmp.append(successor[1])
                tmp = directions + tmp
                if len(solutin) == 0:
                    solutin = tmp
                    sol_value = value + successor[2]
                elif sol_value > value + successor[2]:
                    solutin = tmp
                    sol_value = value + successor[2]
                continue

            if successor[0] not in visited_states:
                tmp = []
                tmp.append(successor[1])
                tmp = directions + tmp

                if len(solutin) != 0:
                    if value + successor[2] > sol_value:
                        continue
                    else:
                        if successor[0] in dij.keys():
                            if dij[successor[0]] > value + successor[2]:
                                dij[successor[0]] = value + successor[2]
                                dir[successor[0]] = tmp
                        else:
                            dij[successor[0]] = value + successor[2]
                            dir[successor[0]] = tmp
                else:
                    if successor[0] in dij.keys():
                        if dij[successor[0]] > value + successor[2]:
                            dij[successor[0]] = value + successor[2]
                            dir[successor[0]] = tmp
                    else:
                        dij[successor[0]] = value + successor[2]
                        dir[successor[0]] = tmp

    return solutin

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    def fun(node):
        node_s, node_e, node_action, node_cost = node
        return node_cost + heuristic(node_e, problem=problem)

    fringe = util.PriorityQueueWithFunction(fun)

    visited_states = dict()
    fringe.push((None, problem.getStartState(), None, 0))

    while not fringe.isEmpty():
        node_s, node_e, node_action, node_cost = fringe.pop()

        if hash(node_e) in visited_states:
            continue

        visited_states[hash(node_e)] = (node_s, node_action)

        # return goal
        if problem.isGoalState(node_e):
            s, action = visited_states[hash(node_e)]
            actions = []
            while action is not None:
                actions.append(action)
                s, action = visited_states[hash(s)]

            return actions[::-1]

        # add successors
        for next, action, added_cost in problem.getSuccessors(node_e):
            if hash(next) not in visited_states:
                fringe.push((node_e, next, action, node_cost + added_cost))

    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch