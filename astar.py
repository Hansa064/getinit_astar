import heapq
import json
from argparse import ArgumentParser, FileType
from collections import defaultdict


def create_parser():
    parser = ArgumentParser(
            description="Navigate from a given source to a given target using the A* algorithm without an heuristic",
    )
    parser.add_argument("-g", "--graph",
                        help="The path to the JSON graph file",
                        type=FileType("r"), required=True)
    parser.add_argument(
            "source",
            help="The name of the source planet to start navigation from",
    )
    parser.add_argument(
            "target",
            help="The name of the planet to navigate to",
    )
    return parser


def create_graph(data):
    graph = defaultdict(dict)
    for edge in data["edges"]:
        graph[edge["source"]][edge["target"]] = edge["cost"]
        graph[edge["target"]][edge["source"]] = edge["cost"]
    return graph


def astar(graph, source, target):
    known = []
    known_nodes = {}
    closed = set()

    def push_node(node, costs, path):
        entry = (costs, node, path)
        heapq.heappush(known, entry)
        known_nodes[node] = entry

    def pop_node():
        entry = heapq.heappop(known)
        del known_nodes[entry[1]]
        return entry

    def expand_node(node, parent_costs, path):
        for node, costs in graph[node].items():
            if node in known_nodes:
                continue
            # A* is defined as: f(x) = g(x) + h(x)
            # with g(x) beeing the costs from source to x
            # and h(x) beeing the estemated costs from x to target.
            # As we don't know the positions of the planets, we use "0" as
            # estimated distance.
            tentative_g =  parent_costs + costs
            if node in known_nodes and tentative_g >= known_nodes[node][0]:
                continue
            elif node in known_nodes:
                known_nodes[node] = (tentative_g, node, path + [node])
            else:
                push_node(node, tentative_g, path + [node])

    push_node(source, 0, [source])
    while known:
        costs, node, path = pop_node()
        if node == target:
            return path, costs
        closed.add(node)
        expand_node(node, costs, path)
    return None


def main(args):
    parser = create_parser()
    arguments = parser.parse_args(args)

    data = json.load(arguments.graph)
    node_ids = {node["label"]: i for i, node in enumerate(data["nodes"])}
    node_names = {i: node["label"] for i, node in enumerate(data["nodes"])}
    graph = create_graph(data)
    source_id = node_ids[arguments.source]
    target_id = node_ids[arguments.target]
    
    path = astar(graph, source_id, target_id)
    if path is None:
        print("No path found!")
        return 1
    path_names = [node_names[node] for node in path[0]]
    print("Path found!:\n\t%s" % " -> ".join(path_names))
    print("Costs: %f" % path[1])
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv[1:]))

