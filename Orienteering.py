from typing import Any
from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass
from heapq import heappush, heappop, heapify
import time
from RenderMap import *


class Terrain(Enum):
    ROAD = .1
    TRAIL = .1
    OPENLAND = .5
    ROUGHMEADOW = .5
    EASYFOREST = .5
    SLOWRUNFOREST = .5
    WALKFOREST = 1
    LAKE = 2
    ICE = 3
    IMPASSABLEVEG = 100
    OUTOFBOUNDS = 200


@dataclass
class point:
    x: int
    y: int
    z: float
    f: int
    g: int
    h: int
    prev: Any
    terrain: Terrain

    def __repr__(self):
        return str(self.x)+","+str(self.y)+" f: "+str(round(self.f,1))+" = "+str(round(self.g,1))+" + "+str(round(self.h,1))
    def __lt__(self, other):
        return self.f < other.f

def makeMap(terrain_image, elevation_file,season):
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
            grid[y][x] = point(x, y, float(z), 0, 0,0, None, getTerrain(hex))
    for y in range(0, len(grid)):
        for x in range(0, len(grid[y])):
            if grid[y][x].terrain == Terrain.LAKE:
                for neighbor in neighbors(grid,grid[y][x]):
                    neighbor.terrain=Terrain.ICE
                    pix[x,y]=(123,255,255,255)
    return terrain,grid

def neighbors(map, current):
    x = current.x
    y = current.y
    neighbors = [(-1, -1), (0, -1), (1, -1),
                 (-1, 0), (1, 0),
                 (-1, 1), (0, 1), (1, 1) ]
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
    #score = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    score=math.sqrt(dx*dx+dy*dy)
    return math.sqrt(score*score+dz*dz)
    #return dx+dy+dz

def terrainHeuristic(start,goal):
    distance=heuristic(start,goal)
    #print(start.terrain)
    terrainMulti=goal.terrain.value
    return distance*terrainMulti

def astar(map, start, goal):

    start = map[start[1]][start[0]]
    goal = map[goal[1]][goal[0]]
    start.f=start.h = heuristic(start, goal)

    print("Predicted distance: ", start.h)
    closed = []
    visited = []
    heap = []
    heappush(heap, start)


    while heap:
        current = heappop(heap)
        #print(current)
        visited.append(current)

        #print("Estimated distance from: ", heuristic(current, goal), "/", start.f)
        if current.x == goal.x and current.y == goal.y:
            print("Found point: ",current.x,",",current.y," Distance: ",current.g)
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.prev
            return path[::-1], visited
        for neighbor in neighbors(map, current):
            if neighbor in closed:
                continue
            neighborG = current.g + terrainHeuristic(current, neighbor)
            if neighbor in heap  and neighborG >= neighbor.g:
                continue
            else:
                neighbor.prev = current
                neighbor.g = neighborG
                neighbor.h= heuristic(neighbor, goal)
                neighbor.f = neighborG+neighbor.h
                heappush(heap, neighbor)
            closed.append(current)

def runCourse(map, path_file):
    with open(path_file) as textFile:
        points = [[int(x) for x in line.split()] for line in textFile]
    stops = [points[0]]
    paths = []
    visited = []
    startT = time.time()
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
    end = time.time()
    print(end - startT)
    return paths, visited, stops

def main(terrain_image, elevation_file, path_file, season, output):
    print("Loading Map")
    season_image,map = makeMap(terrain_image, elevation_file,season)
    print("Map loaded, running A*")
    path, visited, stops = runCourse(map, path_file)
    constructRender("output/elevationPathMap.png",map=map, path=path, stops=stops )
    constructRender("output/visitedMap.png",map=map,visited=visited,path=path,stops=stops ,outline=0)
    constructRender("output/pathMap.png",terrain=season_image, path=path, stops=stops ,outline=1)



if __name__ == '__main__':
    #main("testcases/distanceCalc/terrain.png", "testcases/distanceCalc/mpp.txt", "testcases/distanceCalc/100Xpath.txt", "winter","redWinter.png")
    #main("testcases/elevation/terrain.png", "testcases/elevation/mpp.txt", "testcases/elevation/elPath.txt", "winter","redWinter.png")

    main("testcases/default/terrain.png", "testcases/default/mpp.txt", "testcases/default/easy.txt","winter","redWinter.png")
