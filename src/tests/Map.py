from bs4 import BeautifulSoup

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

        self.__get_map()

    def __get_map(self):
        data_ptrs = [self.soup.width, self.soup.height, self.soup.startx,
                     self.soup.starty, self.soup.finishx, self.soup.finishy]

        for data_ptr in data_ptrs:
            if data_ptr is None or data_ptr <= 0:
                raise ValueError("ERROR: invalid value of the width, height, start of goal position")

        for i in range(2, len(data_ptrs)):
            data_ptrs[i] -= 1

        self.width, self.height, self.start_i, self.start_j, self.goal_i, self.goal_j = data_ptrs

        self.map = [[Interval() for j in range(self.width)] for i in range(self.height)]
        if self.soup.grid is None:
            raise ValueError("ERROR: nothing in grid tag")
        grid = self.soup.grid
        for i, row in enumerate(grid.find_all("row")):
            for j, el in enumerate(row.get_text().split()):
                el = int(el)
                if bool(el):
                    interval = self.map[i][j]
                    interval.is_safe = False
                    interval.start_time = 0
                    interval.end_time = INF

        dynamic_obstacles = self.soup.dynamicobstacles
        for obstacle in dynamic_obstacles.find_all("obstacle"):
            self.dynamic_obstacles = []
            for point in obstacle.find_all("point"):
                x, y, time = int(point["x"]), int(point["y"]), int(point["time"])
                if self.dynamic_obstacles:
                    prev_point = self.dynamic_obstacles[-1].path[-1]
                    distance = self.get_distance(x - 1, y - 1, prev_point.x, prev_point.y)

                    if ((distance == 0 and (time * self.cost - prev_point.time) <= 0) or
                            (distance != 0 and distance != (time * self.cost - prev_point.time))):
                        raise ValueError("ERROR: invalid path representation")

                self.dynamic_obstacles[-1].path.append((x - 1, y - 1, time * self.cost))

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

    def __call__(self, i, j, interval):
        return self.map[i][j][interval]

    def __call__(self, i, j):
        return self.map[i][j]
