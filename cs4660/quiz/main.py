"""
quiz2!

Use path finding algorithm to find your way through dark dungeon!

Tecchnical detail wise, you will need to find path from node 7f3dc077574c013d98b2de8f735058b4
to f1f131f647621a4be7c71292e79613f9

TODO: implement BFS
TODO: implement Dijkstra utilizing the path with highest effect number
"""
from graph import graph as gp
import sys
import heapq
import json
import codecs
from graph import utils

# http lib import for Python 2 and 3: alternative 4
try:
    from urllib.request import urlopen, Request
except ImportError:
    from urllib2 import urlopen, Request

GET_STATE_URL = "http://192.241.218.106:9000/getState"
STATE_TRANSITION_URL = "http://192.241.218.106:9000/state"

def get_state(room_id):
    """
    get the room by its id and its neighbor
    """
    body = {'id': room_id}
    return __json_request(GET_STATE_URL, body)


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

class PriorityEntry(object):

    def __init__(self, dist, node):
        self.dist = dist
        self.node = node

    def __lt__(self, other):
        return self.dist < other.dist


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


def transition_state(room_id, next_room_id):
    """
    transition from one room to another to see event detail from one room to
    the other.

    You will be able to get the weight of edge between two rooms using this method
    """
    body = {'id': room_id, 'action': next_room_id}
    return __json_request(STATE_TRANSITION_URL, body)



def __json_request(target_url, body):
    """
    private helper method to send JSON request and parse response JSON
    """
    req = Request(target_url)
    req.add_header('Content-Type', 'application/json; charset=utf-8')
    jsondata = json.dumps(body)
    jsondataasbytes = jsondata.encode('utf-8')   # needs to be bytes
    req.add_header('Content-Length', len(jsondataasbytes))
    reader = codecs.getreader('utf-8')
    response = json.load(reader(urlopen(req, jsondataasbytes)))
    return response

if __name__ == "__main__":
    # Your code starts here
    empty_room = get_state('7f3dc077574c013d98b2de8f735058b4')

    my_graph = gp.ObjectOriented()

    my_graph.add_node(utils.Tile(empty_room['location']['x'], empty_room['location']['y'], empty_room['id'] ))

    for neighbor in empty_room['neighbors']:
        my_graph.add_node(utils.Tile(neighbor['location']['x'], neighbor['location']['y'], neighbor['id'] ))

        # for all neighbors find their neighbors and find neighbors again ...


        # get edges given node and neighbors
        print(transition_state(empty_room['id'], neighbor['id']))

        


















