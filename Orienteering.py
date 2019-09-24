from typing import Any
import typing
from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass
from heapq import heappush, heappop
import os

from Video import make_video


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


@dataclass
class point:
    x: int
    y: int
    z: float
    f: int
    g: int
    prev: Any
    terrain: Terrain

    def __lt__(self, other):
        return self.f < other.f


def makeMap(terrain_image, elevation_file):
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    with open(elevation_file) as textFile:
        elev = [line.split() for line in textFile]
    grid = [[point for i in range(0, len(elev[0]) - 5)] for j in range(0, len(elev))]
    for y in range(0, len(grid)):
        for x in range(0, len(grid[y])):
            whole = elev[y][x].split('e+')
            elev[y][x] = round(float(whole[0]) * int(math.pow(10, int(whole[1]))), 6)
            z = elev[y][x]
            color = pix[x, y]
            hex = "#{:02x}{:02x}{:02x}".format(color[0], color[1], color[2])
            grid[y][x] = point(x, y, float(z), 0, 0, None, getTerrain(hex))
    return grid


def runCourse(map, path_file):
    with open(path_file) as textFile:
        points = [[int(x) for x in line.split()] for line in textFile]
    stops = [points[0]]
    paths = []
    visited = []
    for i in range(0, len(points) - 1):
        start = points[i]
        next = points[i + 1]
        stops.append(next)
        print("Finding point: ",i+1,"/",len(points)-1,end=' ')

        path, visits = astar(map, start, next)
        for y in range(0, len(map)):
            for x in range(0, len(map[y])):
                map[y][x].f = 0
                map[y][x].g = 0
                map[y][x].prev = None
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
    (x, y, z) = start.x, start.y, start.z
    (gx, gy, gz) = goal.x, goal.y, goal.z
    dx = abs(x - gx)*10.29
    dy = abs(y - gy)*7.55
    dz = abs(z - gz)
    D=1
    D2=1
    score = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    return score+dz


def astar(map, start, goal):
    start = map[start[1]][start[0]]
    goal = map[goal[1]][goal[0]]
    start.f = heuristic(start, goal)
    print("Predicted distance: ", start.f)
    closed = []
    visited = []
    heap = []
    heappush(heap, start)
    while heap:
        current = heappop(heap)
        visited.append(current)

        #print("Estimated distance from: ", heuristic(current, goal), "/", start.f)
        if current.x == goal.x and current.y == goal.y:
            print("Found point: ",current.x,",",current.y," Distance: ",current.g)
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.prev
            return path[::-1], visited
        closed.append(current)

        for neighbor in neighbors(map, current):
            score = current.g + heuristic(current, neighbor)
            if neighbor in closed and score >= neighbor.g:
                continue
            if score < neighbor.g or neighbor not in heap:
                neighbor.prev = current
                neighbor.g = score
                neighbor.f = score + heuristic(neighbor, goal)
                heappush(heap, neighbor)


def draw(terrain_image, map, visited, path, stops):
    print("Drawing Maps")
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    for y in range(0, len(map)):
        for x in range(0, len(map[0])):
            z = int(map[y][x].z)

            z = (z - 187) * 4
            if z < 50:
                pix[x, y] = (0, 0, z, 255)
            elif z >= 50 and z < 100:
                pix[x, y] = (0, z, z, 255)
            elif z >= 100 and z < 150:
                pix[x, y] = (0, z, 0, 255)
            elif z >= 150 and z < 200:
                pix[x, y] = (z, z, 0, 255)
            elif z >= 200:
                pix[x, y] = (z, 0, 0, 255)
    terrain.save("output/elevationMap.png")
    print("Elevation Map Done")
    # for index in range(0, len(visited)):
    #     visit = visited[index]
    #     z = int(255 * index / len(visited))
    #     pix[visit.x, visit.y] = (z, z, z, 255)
    #     terrain.save("images/image" + str(index) + ".png")
    # print("Visited Map Done")

    for x, y in path:
        for i in range (-1,2):
            for j in range(-1,2):

                if pix[x+i,y+j]!=(150, 50, 150, 255):
                    if x!=0 or y!=0 :
                        pix[x+i, y+j] = (200, 100, 230, 255)
        pix[x, y] = (150, 50, 150, 255)

    for i in range(0, len(stops)):
        pix[stops[i][0], stops[i][1]] = (100, 50, 230, 255)
        for y in range(-2,3,1):
            for x in range(-2,3,1):
                if x != 0 or y != 0:
                    pix[stops[i][0]+x, stops[i][1]+y] = (50, 20, 180, 255)

    start, goal = stops[0], stops[len(stops) - 1]
    pix[start[0], start[1]] = (255, 60, 255, 255)
    pix[goal[0], goal[1]] = (255, 60, 255, 255)

    terrain.save("output/pathMap.png")
    print("Path Map Done")

    #make_video(len(visited), 'output/output.avi', 100)


def main(terrain_image, elevation_file, path_file, season, output):
    map = makeMap(terrain_image, elevation_file)
    # printMap(map)

    path, visited, stops = runCourse(map, path_file)
    draw(terrain_image, map, visited, path, stops)


if __name__ == '__main__':
    main("testcases/default/terrain.png", "testcases/default/mpp.txt", "testcases/default/red.txt", "winter",
         "redWinter.png")
