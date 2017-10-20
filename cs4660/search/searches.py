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



def heuristic(dest_node, node):

    x = abs(dest_node.data.x - node.data.x)
    y = abs(dest_node.data.y - node.data.y)

    return x + y

def a_star_search(graph, start, goal):
    frontier = []

    heapq.heappush(frontier, PriorityEntry(0, start))

    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while frontier:

        current = heapq.heappop(frontier).node

        if current == goal:
            L =  reconstruct_path(came_from, current)
            res = []
            for i in range(1, len(L)-1):
                res.append(gp.Edge(L[i], L[i+1], 1))

            return res

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.distance(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                heapq.heappush(frontier, PriorityEntry(priority, next))
                came_from[next] = current

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