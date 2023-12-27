from bs4 import BeautifulSoup
import numpy as np
import bisect

INF = 100000000


class Interval:
    def __init__(self, is_safe=False, start_time=0, end_time=0):
        self.is_safe = is_safe
        self.start_time = start_time
        self.end_time = end_time

    def empty(self):
        return not self.is_safe and not self.start_time and not self.end_time

    def __eq__(self, other):
        return self.start_time == other.start_time and self.end_time == other.end_time

    def __str__(self):
        return "{" + str(self.is_safe) + ", " + str(self.start_time) + ", " + str(self.end_time) + "}"


class PathComponent:
    def __init__(self, x=0, y=0, time=0):
        self.x = x
        self.y = y
        self.time = time

    def __str__(self):
        return f"({self.x}, {self.y}, {self.time})"


class Obstacle:
    def __init__(self):
        self.path = []


class Map:
    def __init__(self, map_path: str):
        self.cost = 1

        with open(map_path, 'r') as xml_file:
            soup = BeautifulSoup(xml_file, "lxml")
        if soup.map is None:
            raise ValueError("ERROR: nothing in map tag")
        self.soup = soup.map
        self.map = None
        self.dynamic_obstacles = []
        self.width = 0
        self.height = 0
        self.start_i, self.start_j = 0, 0
        self.goal_i, self.goal_j = 0, 0

        self.allow_diagonal = True
        self.cut_corners = True
        self.allow_squeeze = True

        self.__get_map()

    def __get_map(self):
        data_ptrs = [self.soup.width, self.soup.height, self.soup.startx,
                     self.soup.starty, self.soup.finishx, self.soup.finishy]
        
        print(self.soup.abcd)

        for data_ptr in data_ptrs:
            if data_ptr is None or data_ptr <= 0:
                raise ValueError("ERROR: invalid value of the width, height, start of goal position")

        for i in range(2, len(data_ptrs)):
            data_ptrs[i] -= 1

        self.width, self.height, self.start_i, self.start_j, self.goal_i, self.goal_j = data_ptrs

        self.map = [[[] for j in range(self.width)] for i in range(self.height)]
        if self.soup.grid is None:
            raise ValueError("ERROR: nothing in grid tag")
        grid = self.soup.grid
        for i, row in enumerate(grid.find_all("row")):
            for j, el in enumerate(row.get_text().split()):
                el = int(el)
                if bool(el):
                    interval = Interval(is_safe=False, start_time=0, end_time=INF)
                    self.map[i][j].append(interval)

        dynamic_obstacles = self.soup.dynamicobstacles
        for obstacle in dynamic_obstacles.find_all("obstacle"):
            self.dynamic_obstacles.append(Obstacle())
            for point in obstacle.find_all("point"):
                x, y, time = int(point["x"]), int(point["y"]), int(point["time"])
                if self.dynamic_obstacles[-1].path:
                    prev_point = self.dynamic_obstacles[-1].path[-1]
                    distance = self.get_distance(x - 1, y - 1, prev_point.x, prev_point.y)

                    if ((distance == 0 and (time * self.cost - prev_point.time) <= 0) or
                            (distance != 0 and distance != (time * self.cost - prev_point.time))):
                        raise ValueError("ERROR: invalid path representation")

                self.dynamic_obstacles[-1].path.append((x - 1, y - 1, time * self.cost))

    def get_successors(self, node):
        successors = []
        for i in range(max(0, node.i - 1), min(node.i + 2, self.get_height())):
            for j in range(max(0, node.j - 1), min(node.j + 2, self.get_width())):
                if self.is_traversable(i, j):
                    cost = self.cost
                    if (i + j) % 2 == (node.i + node.j) % 2:
                        if not self.check_diagonal_successor(node, i, j):
                            continue
                        cost = np.sqrt(2)

                    min_time = node.g + cost / 2
                    max_time = self.map[node.i][node.j][node.interval].end_time

                    if min_time >= max_time:
                        continue

                    self.set_intervals(i, j)

                    interval = (self.get_safe_interval_id(i, j, min_time))
                    successors.append((i, j, cost, interval))
                    # while interval < len(self.map[i][j]) and self.map[i][j][interval].start_time < max_time:
                    #     successor = cells2nodes[(i, j, interval)]
                    #     new_g = max(
                    #         node.g + cost,
                    #         self.map[i][j][interval].startTime + cost / 2
                    #     )
                    #
                    #     if successor is None:
                    #         successor = Node(
                    #             i,
                    #             j,
                    #             interval,
                    #             new_g,
                    #             getHeuristic(i, j, map),
                    #             node
                    #         )
                    #         cells2nodes[(i, j, interval)] = successor
                    #         OPEN.append(successor)
                    #     successors.append((successor, new_g))
                    #
                    # interval += 1

        return successors

    def check_diagonal_successor(self, node, i, j):
        if not self.allow_diagonal or (node.i == i and node.j == j):
            return False

        near_cell1 = self.is_traversable(node.i, j)
        near_cell2 = self.is_traversable(i, node.j)

        return (near_cell1 and near_cell2) or \
               (self.cut_corners and (near_cell1 or near_cell2)) or \
               (self.allow_squeeze and not near_cell1 and not near_cell2)

    def get_distance(self, i1, j1, i2, j2):
        return (abs(i2 - i1) + abs(j2 - j1)) * self.cost

    def get_cost(self):
        return self.cost

    def is_traversable(self, i, j):
        return not self.map[i][j].empty() or self.map[i][j][0].is_safe

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_start_cell(self):
        return self.start_i, self.start_j

    def get_goal_cell(self):
        return self.goal_i, self.goal_j

    def point_on_segment(self, point, p1, p2) -> bool:
        point, p1, p2 = np.array(point), np.array(p1), np.array(p2)
        diff1, diff2 = p1 - point, p2 - point
        return diff1[0] * diff2[1] - diff1[1] * diff2[0] == 0 and diff1[0] * diff1[1] + diff2[0] * diff2[1] >= 0

    def set_intervals(self, i, j):
        if self.map[i][j]:
            return

        collision_intervals: list[Interval] = []

        for obstacle in self.dynamic_obstacles:
            path = obstacle.path
            for point_i in range(1, len(path)):
                if not self.point_on_segment((j, i), [path[point_i - 1].x, path[point_i - 1].y],
                                             [path[point_i].x, path[point_i].y]):
                    continue

                collision_intervals.append(
                    Interval(
                        False,
                        path[point_i - 1].time + self.get_distance(i, j, path[point_i - 1].y,
                                                                   path[point_i - 1].x) - self.cost / 2,
                        path[point_i - 1].time + self.get_distance(i, j, path[point_i - 1].y,
                                                                   path[point_i - 1].x) + self.cost / 2
                    )
                )

                if self.get_distance(path[point_i - 1].y, path[point_i - 1].x, path[point_i].y, path[point_i].x) == 0:
                    collision_intervals[-1].end_time = path[point_i].time + self.cost / 2

        collision_intervals.sort(key=lambda x: x.start_time)

        if not collision_intervals:
            self.map[i][j].append(Interval(True, 0, INF))
            return

        for interval in collision_intervals:
            if not self.map[i][j] or interval.start_time > self.map[i][j][-1].end_time:
                self.map[i][j].append(interval)

            self.map[i][j][-1].end_time = max(self.map[i][j][-1].end_time, interval.end_time)

        if self.map[i][j][0].start_time > 0:
            self.map[i][j].append(Interval(True, self.map[i][j][-1].end_time, INF))
            for interval in range(len(self.map[i][j]) - 2, -1, -1):
                self.map[i][j][interval].is_safe = True
                self.map[i][j][interval].end_tme = self.map[i][j][interval].start_time
                self.map[i][j][interval].start_time = 0 if interval == 0 else self.map[i][j][interval - 1].end_time
        else:
            for interval in range(len(self.map[i][j])):
                self.map[i][j][interval].is_safe = True
                self.map[i][j][interval].start_time = self.map[i][j][interval].end_time

                if interval + 1 == len(self.map[i][j]):
                    self.map[i][j][interval].end_time = INF
                    continue

                self.map[i][j][interval].end = self.map[i][j][interval + 1].start_time

    def get_safe_interval_id(self, i, j, time):
        def compare(time, interval):
            return time < interval.endTime

        return bisect.bisect_right(self.map[i][j], time, key=lambda x: compare(time, x)) - 1

    def get_interval_start(self, i, j, interval):
        return self.map[i][j][interval].startTime

    def get_interval_end(self, i, j, interval):
        return self.map[i][j][interval].endTime
