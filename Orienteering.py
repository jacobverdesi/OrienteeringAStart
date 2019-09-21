from typing import Any

from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass
from heapq import heappush, heappop


class Terrain(Enum):
    OPENLAND = 1
    ROUGHMEADOW = 2
    EASYFOREST = 3
    SLOWRUNFOREST = 4
    WALKFOREST = 5
    IMPASSABLEVEG = 6
    LAKE = 7
    ROAD = 8
    TRAIL = 9
    OUTOFBOUNDS = 10

def makeMap(terrain_image, elevation_file):
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    with open(elevation_file) as textFile:
        elev = [line.split() for line in textFile]
    map = [[point for i in range(0, len(elev[0]) - 5)] for j in range(0, len(elev))]
    for y in range(0, len(map)):
        for x in range(0, len(map[y])):
            whole = elev[y][x].split('e+')
            elev[y][x] = round(float(whole[0]) * int(math.pow(10, int(whole[1]))), 6)
            z = elev[y][x]
            color = pix[x, y]
            hex = rgb2hex(color[0], color[1], color[2])
            map[y][x] = point(x, y, float(z), getTerrain(hex))
    return map

@dataclass
class point:
    x: int
    y: int
    z: float
    terrain: Terrain

@dataclass
class maps:
    width: int
    height: int
    map:Any

    def neighbors(self, id):
        (x, y) = id
        results = [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1),
                   (x - 1, y), (x + 1, y),
                   (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]
        return results


def rgb2hex(r, g, b):
    return "#{:02x}{:02x}{:02x}".format(r, g, b)


def getTerrain(color):
    if color == "#f89412":
        return Terrain.OPENLAND
    elif color == "#ffc000":
        return Terrain.ROUGHMEADOW
    elif color == "#ffffff":
        return Terrain.EASYFOREST
    elif color == "#02d03c":
        return Terrain.SLOWRUNFOREST
    elif color == "#028828":
        return Terrain.WALKFOREST
    elif color == "#054918":
        return Terrain.IMPASSABLEVEG
    elif color == "#0000ff":
        return Terrain.LAKE
    elif color == "#473303":
        return Terrain.ROAD
    elif color == "#000000":
        return Terrain.TRAIL
    elif color == "#cd0065":
        return Terrain.OUTOFBOUNDS


def printMap(map):
    with open("map.txt", "w") as myfile:
        for y in range(0, len(map)):
            for x in range(0, len(map[0])):
                myfile.write("[" + str(map[y][x].x) + "," + str(map[y][x].y) + "] ")
                points = map[y][x]
                print(points.terrain, end=' ')
            myfile.write("\n")
            print()
    myfile.close()


def heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(map, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    start = map[start[1]][start[0]]
    heap = [point(start)]
    while heap:
        current = heappop(heap)
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < map.shape[0]:
                if 0 <= neighbor[1] < map.shape[1]:
                    if map[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # map bound y walls
                    continue
            else:
                # map bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(heap, (fscore[neighbor], neighbor))


def main(terrain_image, elevation_file, path_file, season, output):
    map1=maps(500,395, makeMap(terrain_image, elevation_file))

    printMap(map1)
    start = (168, 236)
    goal = (178, 222)
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    pix[168, 236] = (255, 0, 0, 255)
    pix[178, 222] = (255, 0, 0, 255)

    print(astar(map1, start, goal))
    terrain.save("save.png")


if __name__ == '__main__':
    main("terrain.png", "mpp.txt", "red.txt", "winter", "redWinter.png")
