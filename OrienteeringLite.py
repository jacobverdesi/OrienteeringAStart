from typing import Any
from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass
from heapq import heappush, heappop, heapify
import time
from RenderMap import *

def init(terrain_image,elevation_file):
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    with open(elevation_file) as textFile:
        elev = [line.split() for line in textFile]
    return pix,elev
class Terrain(Enum):
    OPENLAND = 1
    ROUGHMEADOW = 3
    EASYFOREST = 2
    SLOWRUNFOREST = 4
    WALKFOREST = 5
    IMPASSABLEVEG = 100
    LAKE = 6
    ROAD = .7
    TRAIL = .8
    OUTOFBOUNDS = 100


    # grid = [[point for i in range(0, len(elev[0]) - 5)] for j in range(0, len(elev))]
    # for y in range(0, len(grid)):
    #     for x in range(0, len(grid[y])):
    #         whole = elev[y][x].split('e+')
    #         elev[y][x] = round(float(whole[0]) * int(math.pow(10, int(whole[1]))), 6)
    #         z = elev[y][x]
    #         color = pix[x, y]
    #         hex = "#{:02x}{:02x}{:02x}".format(color[0], color[1], color[2])
    #         grid[y][x] = point(x, y, float(z), 0, 0, 0, None, getTerrain(hex))
    # return grid


def runCourse(pix,elev, path_file):
    with open(path_file) as textFile:
        points = [[int(x) for x in line.split()] for line in textFile]
    stops=[points[0]]
    for stop in points:
        whole =elev[stop[1]][stop[0]].split('e+')
        stop.append(round(float(whole[0]) * int(math.pow(10, int(whole[1]))), 6))
        paths = []
    visited = []
    for i in range(0, len(points) - 1):
        start = points[i]
        next = points[i + 1]
        stops.append(next)
        print("Finding point: ", i + 1, "/", len(points) - 1, end=' ')

        path, visits = astar(pix,elev,start, next)

        for p in path:
            if p not in paths:
                paths.append(p)
        for v in visits:
            if v not in visited:
                visited.append(v)
    return paths, visited, stops


def neighbors(map, current):
    x = current.x
    y = current.y
    neighbors = [(-1, -1), (0, -1), (1, -1),
                 (-1, 0), (1, 0),
                 (-1, 1), (0, 1), (1, 1), ]
    result = []
    for i, j in neighbors:
        if 0 < x + i < len(map[0]) and 0 < y + j < len(map):
            result.append(map[y + j][x + i])
    return result


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


def heuristic(start, goal):
    (x, y, z) = start[0],start[1],start[2]
    (gx, gy, gz) = goal[0], goal[1], goal[2]
    dx = abs(x - gx)  # *10.29
    dy = abs(y - gy)  # * 7.55
    dz = abs(z - gz)
    # score = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    score=math.sqrt(dx*dx+dy*dy)
    return math.sqrt(score*score+dz*dz)
    #return dx + dy + dz


def terrainHeuristic(pix,start, goal):
    distance = heuristic(start, goal)
    # print(start.terrain)

    terrainMulti = getTerrain(pix[goal[0]][goal[1]]).value
    return distance * terrainMulti


def astar(pix,elev, start, goal):

    fScore= heuristic(start, goal)
    print("Predicted distance: ",fScore)
    closed = []
    visited = []
    heap = []
    gScore=0
    hScore=0
    heappush(heap, (fScore,gScore,hScore,start))

    startT = time.time()
    while heap:
        current = heappop(heap)
        print(current)
        visited.append(current)

        # print("Estimated distance from: ", heuristic(current, goal), "/", start.f)
        if current[3][0] == goal[0] and current[3][1] == goal[1]:

            end = time.time()
            print(end - startT)
            print("Found point: ", current.x, ",", current.y, " Distance: ", current[0])
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.prev
            return path[::-1], visited
        for neighbor in [(-1, -1), (0, -1), (1, -1),(-1, 0), (1, 0),(-1, 1), (0, 1), (1, 1)]:
            startH = time.time()
            if neighbor in closed:
                continue
            neighborG = current.g + terrainHeuristic(current, neighbor)
            # print(neighbor.g)
            if neighbor in heap and neighborG >= neighbor.g:
                continue

            else:
                neighbor.prev = current
                neighbor.g = neighborG
                neighbor.f = neighborG + heuristic(neighbor, goal)
                heappush(heap, neighbor)
            closed.append(current)
        endH = time.time()
        print(endH - startH)
    return [],[]


def main(terrain_image, elevation_file, path_file, season, output):
    pix,elev=init(terrain_image,elevation_file)
    path, visited, stops = runCourse(pix,elev,path_file)
    constructRender("output/elevationPathMap.png", map=map, path=path, stops=stops)
    constructRender("output/visitedMap.png", map=map, visited=visited, path=path, stops=stops, outline=0)
    constructRender("output/pathMap.png", terrain=terrain_image, path=path, stops=stops, outline=1)


if __name__ == '__main__':
    # main("testcases/distanceCalc/terrain.png", "testcases/distanceCalc/mpp.txt", "testcases/distanceCalc/100Xpath.txt", "winter","redWinter.png")
    main("testcases/default/terrain.png", "testcases/default/mpp.txt", "testcases/default/red.txt", "winter",
         "redWinter.png")
