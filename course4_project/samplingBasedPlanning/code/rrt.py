import random
import collections
import math


class RrtSampling:
    def __init__(self, k, obstacles, r):
        """
        :param k: divide each of dimensions into k intervals, creating k^2 grid cells
               obstacles: obstacle list
               r: radius of the robot
        """
        self.k = k
        self.r = r
        # pixel of the sampling
        self.px = 1/k
        self.obstacles = obstacles
        # fill in the node with id (from 1) for each node in T, utilized for searching
        # each node has two properties: parent node and its id
        self.board = [[None for _ in range(self.k + 1)] for _ in range(self.k + 1)]

    def run(self, n):
        """
        Sampling by Rapidly Exploring Random Tree (RRT)
        This mode is sampling from 2-dimensional Workspace: -0.5 <= x <= 0.5, -0.5 <= y <= 0.5
        For programming and mathematics simplicity,
        the coordinate of node is (0, 0) <= (x, y) <= (k, k) by default
        while it will be transferred to normal position for returning params

        :param:  n: number of sampling nodes
        :return: nodes: sampled nodes
                 edges: the edges for build a search tree
        """
        nodes = [[1, -0.5, -0.5, math.sqrt(2)]]
        edges = []
        # reset the board
        self.board = [[None for _ in range(self.k + 1)] for _ in range(self.k + 1)]
        self.board[0][0] = Node(None, 1)

        # the number of nodes on search tree T
        counter = 0

        while counter < n:
            # sample from X
            x_samp = (
                random.randrange(0, self.k + 1),
                random.randrange(0, self.k + 1)
            )
            # if the sample is on the tree, drop it
            if self.board[x_samp[0]][x_samp[1]] is not None:
                continue
            # search the nearest node in the tree
            node_id, x_nearest = self.search_nearest(x_samp)
            # generate the new node
            x_new = self.new_node(x_nearest, x_samp)

            if not self.has_collision(x_nearest, x_new):
                # if the new node is collision free, add x_new to tree
                new_id = len(nodes) + 1
                x, y = x_nearest
                cost = math.sqrt(
                    pow((x_new[0] - x), 2) + pow((x_new[1] - y), 2)
                ) * self.px
                optimist_cost = math.sqrt(
                    pow((x_new[0] - self.k), 2) + pow((x_new[1] - self.k), 2)
                ) * self.px
                nodes.append([
                    int(new_id),
                    round(x_new[0] * self.px - 0.5, 4),
                    round(x_new[1] * self.px - 0.5, 4),
                    round(optimist_cost, 4)
                ])
                edges.append([
                    int(node_id),
                    int(new_id),
                    round(cost, 4)
                ])

                self.board[x_new[0]][x_new[1]] = Node(self.board[x][y], new_id)
                counter += 1

                if x_new == (self.k, self.k):
                    path = self.get_path()
                    return {
                        'success': True,
                        'nodes': nodes,
                        'edges': edges,
                        'path': path
                    }

        return {
            'success': False
        }

    def search_nearest(self, x_samp):
        """
        Search the nearest node in tree to the x_samp, by Breadth First Search
        :param board: 2-dimensional array for searching
        :param x_samp: sampled point, the search base
        :return: nearest node in T to x_samp
        """
        dx = [1, 0, -1, 0]
        dy = [0, 1, 0, -1]
        queue = collections.deque()
        queue.append(x_samp)
        visited = [[False for _ in range(len(self.board[0]))] for _ in range(len(self.board[1]))]

        while len(queue) > 0:
            x, y = queue.popleft()
            # loop the left, right, top, bottom of (x, y)
            for index in range(4):
                x2 = x + dx[index]
                y2 = y + dy[index]
                if not self.in_bound(x2, y2):
                    continue
                if self.board[x2][y2] is None and not visited[x2][y2]:
                    # this node is not in the tree or has been visited
                    queue.append((x2, y2))
                    visited[x2][y2] = True
                if self.board[x2][y2] is not None:
                    # this node is in the tree, than we found the nearest node
                    return self.board[x2][y2].id, (x2, y2)

    def in_bound(self, x, y):
        """
        Test if the (x, y) is in bound of board
        """
        return 0 <= x < len(self.board) and 0 <= y < len(self.board[0])

    def has_collision(self, node1, node2):
        """
        Checker if there is any collision between the segment and the obstacles
        :param: node1: node1 position in board
                node2: node2 position in board
        :return: True if any collision found
        """
        x1, y1 = node1
        x2, y2 = node2

        for obstacle in self.obstacles:
            x0, y0, r = obstacle

            # transfer to position in board
            x0 = round((x0 + 0.5) / self.px)
            y0 = round((y0 + 0.5) / self.px)
            r = round((r / 2 + self.r) / self.px, 4)

            # distance from obstacle central to each node
            d1 = math.sqrt(pow((x0 - x1), 2) + pow((y0 - y1), 2))
            d2 = math.sqrt(pow((x0 - x2), 2) + pow((y0 - y2), 2))

            if d1 <= r or d2 <= r:
                # if any node is inside the obstacle
                return True
            else:
                X1X0 = [node1, (x0, y0)]
                X1X2 = [node1, node2]
                X2X0 = [node2, (x0, y0)]

                angle_left = self.angle(X1X2, X1X0)
                angle_right = self.angle(X1X2, X2X0)
                if round(math.cos(angle_left) * math.cos(angle_right), 4) < 0:
                    return True
                elif angle_left == 0 and round(angle_right - math.pi, 4) == 0:
                    return True
                elif round(angle_left - math.pi, 4) == 0 and angle_right == 0:
                    return True
        return False

    @staticmethod
    def new_node(x_nearest, x_samp):
        """
        Local planner to find a motion from x_nearest to x_samp
        8-connected grid point implemented in this local planner
        The new node depends on the slope, details described on README.pdf

        :param x_nearest: (x, y), nearest node in tree to sampled node with id
        :param x_samp: (x, y), sampled node
        :return: x_new: new node to be added to the tree
        """
        x, y = x_nearest
        x2, y2 = x_samp

        # treat the singularity, theta is pi/2 or -pi/2
        if x == x2 and y > y2:
            return x, y - 1
        if x == x2 and y < y2:
            return x, y + 1

        slope = (y2 - y) / (x2 - x)

        if x2 > x:
            # when x_sample is at right side of x_nearest, theta is from -pi/2 to pi/2
            factor = 1
        else:
            # when x_sample is at left side of x_nearest, theta is from pi/2 to 3*pi/2
            factor = -1

        # get the x_new by slope from x_nearest to x_sample
        if slope <= -2:
            return x, y - factor
        elif -2 < slope <= -1 / 2:
            return x + factor, y - factor
        elif -1 / 2 < slope <= 1 / 2:
            return x + factor, y
        elif 1 / 2 < slope <= 2:
            return x + factor, y + factor
        elif slope > 2:
            return x, y + factor

    @staticmethod
    def angle(v1, v2):
        """
        Calculate the angle between two vectors, return radius
        """
        dx1 = v1[1][0] - v1[0][0]
        dy1 = v1[1][1] - v1[0][1]
        dx2 = v2[1][0] - v2[0][0]
        dy2 = v2[1][1] - v2[0][1]
        if dx1 != 0:
            angle1 = math.atan2(dy1, dx1)
        else:
            angle1 = math.pi / 2
        if dx2 != 0:
            angle2 = math.atan2(dy2, dx2)
        else:
            angle2 = math.pi / 2

        if angle1 * angle2 >= 0:
            return math.fabs(angle1 - angle2)
        else:
            return math.fabs(angle1) + math.fabs(angle2)

    def get_path(self):
        """
        Get the path from end node of search tree
        """
        node = self.board[self.k][self.k]
        path = [node.id]
        p = node.parent
        while p.parent is not None:
            p = p.parent
            path.append(p.id)
        path.reverse()
        return path


class Node:
    """
    Search Tree Node
    """
    def __init__(self, parent, id):
        self.parent = parent
        self.id = id
