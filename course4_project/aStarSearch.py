import numpy as np
import csv

# *** COURSE 4: ROBOT MOTION PLANNING AND CONTROL ***
# Assignment 1 - A* Search
# the workspace of the planar robot is -0.5 <= x <= 0.5 and -0.5 <= y <= 0.5
#


def a_star_search(nodes, edges):
    """
    :param nodes: N x 4 matrix as [#, x, y, heuristic-cost-to-go], # is the number of node, (x, y) is the position,
        heuristic-cost-to-go is an optimistic estimate of the path length from that node to the goal node.
    :param edges: M x 3 matrix as [ID1, ID2, cost], where ID1 and ID2 are the IDs of the nodes connected by the edge
        and cost is the cost of traversing that edge (in either direction)
    :return: { success: <boolean>, path: <array> }
    """
    nodes = np.array(nodes)
    edges = np.array(edges)
    n = len(nodes)  # Number of nodes
    start = 1
    end = n
    search_map = build_search_map(edges)  # create search tree of bidirectional graph by edges table

    past_cost = [np.inf] * n  # The cost of the best known path to this node, key = <node id> - 1
    past_cost[0] = 0  # Set the 1st node as a start
    optimist_ctg = nodes[:, 3]  # The optimistic cost to go for each node
    est_tot_cost = [np.inf] * n  # Estimated cost of the best solution
    est_tot_cost[0] = nodes[0, 3]
    parent_node = [None] * n  # parent node

    node_open = []  # a list of nodes to explore from, <object array>.
    closed = set()  # a list of nodes already explored, <data set>
    node_open.append({
        'node': start,
        'cost': float(nodes[0, 3])
    })  # initiate with node 1

    while len(node_open) > 0:
        # exam the current open node, which has the lowest cost
        current = node_open.pop()
        # this is because my OPEN list can not eliminate duplicated parts
        if current['node'] in closed:
            continue
        closed.add(current['node'])
        if current['node'] == end:
            return {'success': True, 'path': find_path(parent_node, end)}

        current_node = current['node']
        siblings = search_map[str(current_node)]
        # for each nodes connected with current node and not in CLOSED
        for s in siblings:
            nbr = int(s[0])
            cost = float(s[1])
            if nbr in closed:
                continue

            tentative_past_cost = past_cost[current_node - 1] + cost

            if tentative_past_cost < past_cost[nbr - 1]:
                past_cost[nbr - 1] = tentative_past_cost
                parent_node[nbr - 1] = current_node
                est_tot_cost[nbr - 1] = tentative_past_cost + float(optimist_ctg[nbr - 1])
                # put the nbr in sorted list OPEN
                node_open.append({
                    'node': nbr,
                    'cost': est_tot_cost[nbr - 1]
                })
                node_open.sort(key=lambda x: x['cost'], reverse=True)
    return {'success': False, 'path': [1]}


def build_search_map(edges):
    """
    Create a search map by edges
    Example:
    {
        node1: [(node2, cost2), (node3, cost3)]
    }
    """
    search_map = {}
    for edge in edges:
        id1, id2, cost = edge
        if id1 not in search_map:
            search_map[id1] = []
        if id2 not in search_map:
            search_map[id2] = []
        search_map[id1].append((id2, cost))
        search_map[id2].append((id1, cost))
    return search_map


def find_path(parents, end):
    """
    find the path from the parent node list and the end node
    """
    path = []
    while end is not None:
        path.append(end)
        end = parents[end - 1]
    path.reverse()
    return path


def read_csv(filename):
    """
    Read csv file
    """
    res = []
    with open(filename) as csvfile:
        content = csv.reader(csvfile, delimiter=',')
        for row in content:
            res.append(row)
    return res


def write_csv(filename, data):
    """
    Write to csv file
    """
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)


# start from here
node_list = read_csv('results/nodes.csv')
edge_list = read_csv('results/edges.csv')
result = a_star_search(node_list, edge_list)
# print(result['success'])
# print(result['path'])
write_csv('results/path.csv', result['path'])
