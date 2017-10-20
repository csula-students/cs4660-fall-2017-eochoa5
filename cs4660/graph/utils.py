"""
utils package is for some quick utility methods
such as parsing
"""

from . import graph as gp

class Tile(object):
    """Node represents basic unit of graph"""
    def __init__(self, x, y, symbol):
        self.x = x
        self.y = y
        self.symbol = symbol

    def __str__(self):
        return 'Tile(x: {}, y: {}, symbol: {})'.format(self.x, self.y, self.symbol)
    def __repr__(self):
        return 'Tile(x: {}, y: {}, symbol: {})'.format(self.x, self.y, self.symbol)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y and self.symbol == other.symbol
        return False
    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(str(self.x) + "," + str(self.y) + self.symbol)



def parse_grid_file(graph, file_path):
    """
    ParseGridFile parses the grid file implementation from the file path line
    by line and construct the nodes & edges to be added to graph
    Returns graph object
    """

    with open(file_path) as f:
        content = f.read().splitlines()

    count = len(content[1:-1])

    if count > 100:
        graph = gp.AdjacencyList()

    row = []
    for y in range(count):
        line = content[y+1]
        i = 1
        curr = []
        while i < len(line)-3:
            x = int((i-1)/2)
            node1 = gp.Node(Tile(x, y, line[i]+line[i+1]))
            graph.add_node(node1)
            i += 2

            if node1.data.symbol != "##":

                if node1.data.x > 0:
                    other_node = curr[-1]
                    if other_node.data.symbol != "##":
                        graph.add_edge(gp.Edge(other_node, node1, 1))
                        graph.add_edge(gp.Edge(node1, other_node, 1))

                if node1.data.y > 0:
                    other_node = row[x]
                    if other_node.data.symbol != "##":
                        graph.add_edge(gp.Edge(other_node, node1, 1))
                        graph.add_edge(gp.Edge(node1, other_node, 1))

            curr.append(node1)

        row = curr

    return graph


def convert_edge_to_grid_actions(edges):
    """
    Convert a list of edges to a string of actions in the grid base tile
    e.g. Edge(Node(Tile(1, 2), Tile(2, 2), 1)) => "S"

    """

    answer = ""

    if edges is not None:

        for edge in edges:
            x1 = edge.from_node.data.x
            y1 = edge.from_node.data.y

            x2 = edge.to_node.data.x
            y2 = edge.to_node.data.y

            if x2 > x1:
                answer += "E"
            elif x2 < x1:
                answer += "W"
            elif y2 > y1:
                answer += "S"
            elif y2 < y1:
                answer += "N"

    return answer


def main():

    parse_grid_file(gp.ObjectOriented(), "../test/fixtures/grid-1.txt")




if __name__ == "__main__":
    main()