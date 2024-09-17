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
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
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
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    # print("Start:", problem.getStartState()) # Start: (5, 5)
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState())) # False
    # print("Start's successors:", problem.getSuccessors(problem.getStartState()))   # Start's successors: [((5, 4), 'South', 1), ((4, 5), 'West', 1)]

    
    startPlace = problem.getStartState()
    st = util.Stack()
    st.push([startPlace, []])

    visited = set()

    while not st.isEmpty():
        parent_node, actions = st.pop()

        if problem.isGoalState(parent_node):
            return actions

        if parent_node not in visited:
            visited.add(parent_node)

            for nextState, action, _ in problem.getSuccessors(parent_node):
                if nextState not in visited:
                    st.push([nextState, actions + [action]])
                    
    

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    startPlace = problem.getStartState()
    q = util.Queue()
    q.push([startPlace, []])

    visited = set()

    while not q.isEmpty():
        parent_node, actions = q.pop()

        if problem.isGoalState(parent_node):
            return actions

        if parent_node not in visited:
            visited.add(parent_node)

            for nextState, action, _ in problem.getSuccessors(parent_node):
                if nextState not in visited:
                    q.push([nextState, actions + [action]])
    

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    startPlace = problem.getStartState()
    priorityQueue = util.PriorityQueue()
    priorityQueue.push([startPlace, [], 0], 0)

    visited = set()

    while not priorityQueue.isEmpty():
        
        parent_node_and_actions_list = priorityQueue.pop()
        parent_node = parent_node_and_actions_list[0]
        actions = parent_node_and_actions_list[1]
        parent_cost = parent_node_and_actions_list[2]

        if problem.isGoalState(parent_node):
            return actions

        if parent_node not in visited:
            visited.add(parent_node)

            for nextState, action, priority in problem.getSuccessors(parent_node):
                if nextState not in visited:
                    priorityQueue.push([nextState, actions + [action], parent_cost+priority], parent_cost+priority)

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    startPlace = problem.getStartState()
    priorityQueue = util.PriorityQueue()
    start_priority = heuristic(startPlace, problem)
    priorityQueue.push([startPlace, [], 0], start_priority)

    visited = {startPlace:0}

    while not priorityQueue.isEmpty():
        
        parent_node_and_actions_list = priorityQueue.pop()
        parent_node = parent_node_and_actions_list[0] # parent node
        actions = parent_node_and_actions_list[1] # actions 
        parent_cost = parent_node_and_actions_list[2] # costs

        if problem.isGoalState(parent_node):
            return actions

        for nextState, action, priority in problem.getSuccessors(parent_node):
            heuristic_cost = heuristic(nextState, problem)
            new_cost = parent_cost + priority
            total_priority = new_cost + heuristic_cost
            if nextState not in visited or new_cost < visited[nextState]:
                visited[nextState] = new_cost
                priorityQueue.update([nextState, actions + [action], new_cost], total_priority)

   

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
