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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # hash table para armazenar nos visitados - a chave eh a coordenada do eixo x
    visited = dict()
    # fila de prioridade para ordenar os nos que devem ser visitados em ordem, conforme o custo de f(n)
    border = util.PriorityQueue()

    # dados para o no raiz
    start_state = problem.getStartState()
    path_cost = 0
    heuristic_cost = heuristic(problem.getStartState(), problem)

    node_dict = dict()
    # organizamos as informacoes de cada no em um dicionario de dicionarios:
        # chave: coordenadas do no no tabuleiro (state)
        # atributos:
        # actions: quais foram as direcoes tomadas do no raiz ate o no em questao
        # path_cost: o custo real do caminho do no raiz ao no em questao
    node_dict[start_state] = {'actions': [], 'path_cost': path_cost}
    # colocamos o primeiro no na fila de prioridade que define a fronteira
    border.push(start_state, path_cost + heuristic_cost)

    # criamos uma lista vazia para a chave do no raiz
    visited[start_state[0]] = []
    # enquanto a fronteira nao estiver vazia
    while border.count > 0:
        node = border.pop()
        # verificamos se o no eh o no meta
        if problem.isGoalState(node):
            return node_dict[node]['actions']
        else:
            # adicionamos o no na lista de visitados (na posicao da sua chave na hash table)
            visited[node[0]].append(node)

            # para cada filho do no atual, verificamos se ja foi visitado
            for child in problem.getSuccessors(node):
                # criamos um dicionario para o no
                child_node = {'state': child[0], 'action': child[1], 'cost': child[2]}
                if child_node['state'][0] not in visited: # se a chave nao esta no dicionario
                    visited[child_node['state'][0]] = [] # criamos uma lista vazia nessa chave
                if child_node['state'] not in visited[child_node['state'][0]]: # testa se o no ja foi visitado acessando
                                                                                # sua chave na hash table
                    child_path_cost = node_dict[node]['path_cost'] + child_node['cost']
                    child_path = node_dict[node]['actions'] + [child_node['action']]
                    if child_node['state'] not in node_dict.keys(): # se o no nao esta na fronteira
                        # nesse caso o no nao esta nem na borda e nem nos visitados
                        # adicionamos o no na borda com a lista das acoes e o custo para chegar ate ele
                        # e tambem no dicionario que armazena informacoes de todos os nos
                        node_dict[child_node['state']] = {'actions': child_path, 'path_cost': child_path_cost}
                        border.push(child_node['state'], child_path_cost + heuristic(child_node['state'], problem))
                    elif node_dict[child_node['state']] > child_path_cost:
                        # no esta na borda porem com custo mais elevado que o atual
                        # substituimos pelo caminho de menor custo
                        node_dict[child_node['state']] = {'actions': child_path, 'path_cost': child_path_cost}
                        border.update(child_node['state'], child_path_cost + heuristic(child_node['state'], problem))
    # caso a fronteira esteja vazia e o no meta nao foi encontrado retornamos um erro
    return RuntimeError



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
