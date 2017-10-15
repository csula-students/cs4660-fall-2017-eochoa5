"""
Searches module defines all different search algorithms
"""

from graph import graph as gp
import sys
import heapq
from graph import utils
from queue import PriorityQueue
import math

class PriorityEntry(object):

    def __init__(self, dist, node):
        self.dist = dist
        self.node = node

    def __lt__(self, other):
        return self.dist < other.dist


def nodes_path_to_edges_path(graph, lis):
    res = []
    for i in range(len(lis)-1):
        res.append(gp.Edge(lis[i], lis[i+1], graph.distance(lis[i], lis[i+1])))

    return res


def bfs(graph, initial_node, dest_node):
    """
    Breadth First Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """
    queue = [(initial_node, [initial_node])]
    while queue:
        (vertex, path) = queue.pop(0)
        for next in set(graph.neighbors(vertex)) - set(path):
            if next == dest_node:
                lis = path + [next]
                return nodes_path_to_edges_path(graph, lis)
            else:
                queue.append((next, path + [next]))


def dfs_helper(graph, initial_node, dest_node, path=None):
    if path is None:
        path = [initial_node]
    if initial_node == dest_node:
        yield path

    neighbors = graph.neighbors(initial_node)
    for p in path:
        if p in neighbors:
            neighbors.remove(p)

    for node in neighbors:
        yield from dfs_helper(graph, node, dest_node, path + [node])


def dfs(graph, initial_node, dest_node):
    """
    Depth First Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """

    return nodes_path_to_edges_path(graph, list(dfs_helper(graph, initial_node, dest_node))[0])


def dijkstra_search(graph, initial_node, dest_node):
    """
    Dijkstra Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """
    dist = {}
    prev = {}
    dist[initial_node] = 0
    Q = []

    for v in graph.get_all_nodes():
        if v != initial_node:
            dist[v] = sys.maxsize
            prev[v] = None

        heapq.heappush(Q, PriorityEntry(dist[v], v))

    while Q:
        u = heapq.heappop(Q).node

        if u == dest_node:
            S = []
            u = dest_node

            while u in prev:
                S.insert(0, u)
                u = prev[u]
            S.insert(0, u)

            return nodes_path_to_edges_path(graph, S)

        for v in graph.neighbors(u):
            alt = dist[u] + graph.distance(u, v)
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

                # dec priority
                for x in Q:
                    if x.node == v:
                        Q.remove(x)
                        heapq.heappush(Q, PriorityEntry(alt, v))


def a_star_search(graph, initial_node, dest_node):
    """
    A* Search
    uses graph to do search from the initial_node to dest_node
    returns a list of actions going from the initial node to dest_node
    """


    closedSet = set()
    openSet = {initial_node}
    cameFrom = {}
    gScore = {}
    fScore = {}

    for my_node in graph.get_all_nodes():
        gScore[my_node] = sys.maxsize
        fScore[my_node] = sys.maxsize

    # The cost of going from start to start is zero.
    gScore[initial_node] = 0

    # For the first node, that value is completely heuristic.
    fScore[initial_node] = heuristic_cost_estimate(initial_node, dest_node)

    while openSet:
        current = find_node_min_fscore_in_openSet(openSet, fScore)
        if current == dest_node:
             return nodes_path_to_edges_path(graph, reconstruct_path(cameFrom, current))

        if current in openSet:
            openSet.remove(current)
        closedSet.add(current)

        for neighbor in graph.neighbors(current):
            if neighbor in closedSet:
                continue		# Ignore the neighbor which is already evaluated.

            if neighbor not in openSet:	# Discover a new node
                openSet.add(neighbor)

            # The distance from start to a neighbor
            tentative_gScore = gScore[current] + graph.distance(current, neighbor)
            if tentative_gScore >= gScore[neighbor]:
                continue

            cameFrom[neighbor] = current
            gScore[neighbor] = tentative_gScore
            fScore[neighbor] = gScore[neighbor] + heuristic_cost_estimate(neighbor, dest_node)

def find_node_min_fscore_in_openSet(openSet, fScore):

    answer = openSet.pop()
    min = fScore[answer]

    for node in openSet:
        if fScore[node] < min:
            answer = node

    return answer


def heuristic_cost_estimate(initial_node, dest_node):
    dx = abs(initial_node.data.x - dest_node.data.x)
    dy = abs(initial_node.data.y - dest_node.data.y)

    return  dx + dy


def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path


def main():

    g = utils.parse_grid_file(gp.AdjacencyMatrix(), "../test/fixtures/grid-4.txt")
    print(g)
    as_test = a_star_search(g,
                  gp.Node(utils.Tile(4, 0, "@1")),
                gp.Node(utils.Tile(6, 201, "@4")))


    print(utils.convert_edge_to_grid_actions(as_test))


if __name__ == "__main__":
    main()