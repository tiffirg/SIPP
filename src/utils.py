from typing import Callable, Dict, Iterable, List, Optional, Tuple, Type, Union


class Interval:
    def __init__(self, is_safe=False, start_time=0, end_time=0):
        self.is_safe = is_safe
        self.start_time = start_time
        self.end_time = end_time

    def empty(self):
        return not self.is_safe and not self.start_time and not self.end_time

    def __eq__(self, other):
        return self.start_time == other.start_time and self.end_time == other.end_time
    
    def __repr__(self) -> str:
        return "{" + str(self.is_safe) + ", " + str(self.start_time) + ", " + str(self.end_time) + "}"


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