#!/usr/bin/env python
# coding: utf-8

from bs4 import BeautifulSoup
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation
import numpy as np
from bisect import bisect_left
import sys


class Map:
    def __init__(self, xml_file):
        print("start init")
        with open(xml_file, 'r') as xml_file:
            soup = BeautifulSoup(xml_file, "lxml")
        

        self.width = int(soup.width.text)
        self.height = int(soup.height.text)
        self.start = [int(soup.startx.text) - 1, int(soup.starty.text) - 1]
        self.finish = [int(soup.finishx.text) - 1, int(soup.finishy.text) - 1]
        self.grid = [
            [int(cell) for cell in row.text.split(' ')]
            for row in soup.find_all('row')
        ]
        

        self.obstacles = [
            [
                [int(point['x']) - 1, int(point['y']) - 1, float(point['time'])]
                for point in obstacle.find_all('point')
            ]
            for obstacle in soup.find_all('obstacle')
        ]

        self.path_time = int(soup.find('summary')['pathlength'])
        # self.path_time = max([obstacle[-1][-1] for obstacle in self.obstacles])



        if self.path_time == 0:
            self.path = [self.start + [0.0]]
            self.path_time = 100
        else:
            self.path = [
                [int(point['x']) - 1, int(point['y']) - 1, float(point['time'])]
                for point in soup.find('path').find_all('point')
            ]


    def getPosition(self, path, time):
        point_i = bisect_left(path, time, key=lambda point: point[2])

        if point_i == len(path):
            return path[-1][:2]

        point = np.array(path[point_i])
        if point[2] == time:
            return point[0:2]
        else:
            prev_point = np.array(path[point_i - 1])
            k = (time - prev_point[2]) / (point[2] - prev_point[2])
            return (prev_point + k * (point - prev_point))[:2]


def create_animation(map, zoom_eps=None):
    fig, ax = plt.subplots(figsize=(7, 4))
    plt.rcParams['animation.ffmpeg_path'] = '/opt/homebrew/bin/ffmpeg'
    cmap = mcolors.ListedColormap(['white', 'black'])
    ax.imshow(map.grid, cmap)

    # Grid lines
    ticks_deltas = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]
    ax.tick_params(which='major', top=True, bottom=False, labeltop=True, labelbottom=False)
    ax.tick_params(which='minor', top=False, bottom=False, left=False, right=False)
    x_index = bisect_left(ticks_deltas, (2 * zoom_eps[0] if zoom_eps else map.width) / 10)
    y_index = bisect_left(ticks_deltas, (2 * zoom_eps[1] if zoom_eps else map.height) / 10)
    ax.set_xticks(np.arange(0, map.width, ticks_deltas[x_index]))
    ax.set_yticks(np.arange(0, map.height, ticks_deltas[y_index]))
    ax.set_xticks(np.arange(-0.5, map.width, ticks_deltas[x_index - 1 + (x_index == 0)]), minor=True)
    ax.set_yticks(np.arange(-0.5, map.height, ticks_deltas[y_index - 1 + (y_index == 0)]), minor=True)
    ax.grid(which='minor', color='black', linewidth=0.5, linestyle='--')

    # Start positions of obstacles and agent
    start_x = [map.obstacles[i][0][0] for i in range(len(map.obstacles))]
    start_y = [map.obstacles[i][0][1] for i in range(len(map.obstacles))]
    if max(map.width, map.height) < 50:
        points, = ax.plot(start_x, start_y, marker="o", ls='', label='obstacles')
        agent, = ax.plot(map.start[0], map.start[1], marker="o", color='red', ls='', label='agent')
    else:
        points, = ax.plot(start_x, start_y, marker="o", ls='', ms=200/max(map.width, map.height), label='obstacles')
        agent, = ax.plot(map.start[0], map.start[1], marker="o", color='red', ls='', ms=200/max(map.width, map.height), label='agent')
    ax.plot(map.start[0], map.start[1], marker="$S$", color='red', ls='', label='start')
    ax.plot(map.finish[0], map.finish[1], marker="$F$", color='red', ls='', label='finish')

    if zoom_eps is not None:
        ax.set_xlim(map.start[0] - zoom_eps[0], map.start[0] + zoom_eps[0])
        ax.set_ylim(map.start[1] + zoom_eps[1], map.start[1] - zoom_eps[1])

    # Update positions of obstacles and agents
    def update(time):
        x, y = [0] * len(map.obstacles), [0] * len(map.obstacles)
        for i in range(len(map.obstacles)):
            x[i], y[i] = map.getPosition(map.obstacles[i], time)
        points.set_data((x, y))

        x, y = map.getPosition(map.path, time)
        agent.set_data((x, y))

        if zoom_eps is not None:
            ax.set_xlim(x - zoom_eps[0], x + zoom_eps[0])
            ax.set_ylim(y + zoom_eps[1], y - zoom_eps[1])
        return points, agent, 
        
    anim = animation.FuncAnimation(fig, update, interval=5, blit=True, repeat=False,
                                frames=np.linspace(0, map.path_time + 2, 800 * (map.path_time // 300 + 1)))

    print("Animation is created, ready to save")
    return anim


def get_zoom():
    default_values = [10, 10]
    if "-zoom" not in sys.argv:
        return None
    
    index = sys.argv.index("-zoom")
    if index + 1 >= len(sys.argv):
        return default_values
    elif index + 2 >= len(sys.argv):
        values = [sys.argv[index + 1], sys.argv[index + 1]]
    else:
        values = [sys.argv[index + 1], sys.argv[index + 2]]

    for i in range(2):
        try:
            values[i] = int(values[i])
            if values[i] <= 0:
                values[i] = default_values[i]
        except:
            values[i] = default_values[i]
    return values


def get_outputfile():
    if "-o" not in sys.argv:
        return None
    
    index = sys.argv.index("-o")
    if index + 1 >= len(sys.argv):
        return None
    
    return sys.argv[index + 1]




if len(sys.argv) < 2:
    print("Error: input file is not specified")
    sys.exit()

input_file = sys.argv[1]
try:
    map = Map(input_file)
except:
    print("Invalid input file.")
    sys.exit()

output_file = get_outputfile()
if output_file is None:
    output_file = input_file[:-4] + ".mp4"

zoom_eps = get_zoom()

# plt.show()

writermp4 = animation.writers['ffmpeg'](fps=50, bitrate=4000)

try:
    anim = create_animation(map, zoom_eps)
except:
    print("animation err")
    sys.exit()

try:
    anim.save(output_file, writermp4, dpi=400)
except Exception as e:
    print(e)
    sys.exit()