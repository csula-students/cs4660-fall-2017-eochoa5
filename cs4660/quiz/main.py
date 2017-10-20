"""
quiz2!

Use path finding algorithm to find your way through dark dungeon!

Tecchnical detail wise, you will need to find path from node 7f3dc077574c013d98b2de8f735058b4
to f1f131f647621a4be7c71292e79613f9

TODO: implement BFS
TODO: implement Dijkstra utilizing the path with highest effect number
"""

import json
import codecs

# http lib import for Python 2 and 3: alternative 4
try:
    from urllib.request import urlopen, Request
except ImportError:
    from urllib2 import urlopen, Request

GET_STATE_URL = "http://192.241.218.106:9000/getState"
STATE_TRANSITION_URL = "http://192.241.218.106:9000/state"

START = '7f3dc077574c013d98b2de8f735058b4'
END = 'f1f131f647621a4be7c71292e79613f9'

def get_state(room_id):
    """
    get the room by its id and its neighbor
    """
    body = {'id': room_id}
    return __json_request(GET_STATE_URL, body)

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
    response = urlopen(req, jsondataasbytes)
    reader = codecs.getreader('utf-8')
    return json.load(reader(response))


def BFS(initial_node, dest_node):
    dist = {}
    parent = {}
    edge_to = {}
    dist[initial_node] = 0
    Q = []
    Q.append((0, initial_node))

    while Q:
        u = get_state(Q.pop()[1])
        neighbors = u['neighbors']

        for i in range(len(neighbors)):
            v = neighbors[i]
            if not v['id'] in dist:
                edge = transition_state(u['id'], v['id'])
                edge_to[v['id']] = edge
                dist[v['id']] = dist[u['id']] + 1
                parent[v['id']] = u['id']

                if v['id'] != dest_node:
                    Q.append((dist[v['id']], v['id']))

        Q = sorted(Q, reverse=True, key=lambda x:x[0])

    path = []
    id = dest_node
    while id in parent:
        path.insert(0, edge_to[id])
        id = parent[id]

    return path


def dijkstra_search(initial_node, dest_node):
    dist = {}
    prev = {}
    edge_to = {}
    dist[initial_node] = 0

    Q = []
    Q.append((0, initial_node))
    seen = []

    while Q:
        u = get_state(Q.pop()[1])
        seen.append(u['id'])
        neighbors = u['neighbors']

        for i in range(len(neighbors)):
            v = neighbors[i]
            edge = transition_state(u['id'], v['id'])
            alt = dist[u['id']] + edge['event']['effect']

            if v['id'] not in seen and (v['id'] not in dist or dist[v['id']] < alt):
                if v['id'] in dist:
                    Q.remove((dist[v['id']], v['id']))

                # set priority to alt
                Q.append((alt, v['id']))
                dist[v['id']] = alt
                prev[v['id']] = u['id']
                edge_to[v['id']] = edge

        # sort list queue by distance nlogn
        Q = sorted(Q, key=lambda x:x[0])

    path = []
    id = dest_node

    while id in prev:
        path.insert(0, edge_to[id])
        id = prev[id]

    return path


def find_total(path):
    prev = START
    sum = 0

    for i in range(len(path)):
        prev_node = get_state(prev)
        next = path[i]['id']
        sum += path[i]['event']['effect']

        print(prev_node['location']['name'] + "(" + prev + "):" + path[i]['action']
              + "(" + path[i]['id'] + "):" + str(path[i]['event']['effect']))

        prev = next

    print("\n total : " + str(sum))

if __name__ == "__main__":

    print("\n Executing Dijkstra...: \n")
    find_total(dijkstra_search(START, END))

    print("\n Executing BFS...: \n")
    find_total(BFS(START, END))






