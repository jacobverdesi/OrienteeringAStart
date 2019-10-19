import queue
import sys
from typing import Any
from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass
from heapq import heappush, heappop
import time
from RenderMap import *


class Terrain(Enum):
    ROAD = 1
    TRAIL = .95
    OPENLAND = 1.25
    ROUGHMEADOW = 1.5
    EASYFOREST = 2
    SLOWRUNFOREST = 3
    WALKFOREST = 4
    LAKE = 7
    ICE = 8
    MUD = 7.5
    LEAVES=6
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
    visited:bool
    inHeap:bool

    def __repr__(self):
        return str(self.x) + "," + str(self.y) + " f: " + str(round(self.f, 1))

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
            elev[y][x] = round(float(whole[0]) * int(math.pow(10, int(whole[1]))), 7)
            z = elev[y][x]
            color = pix[x, y]
            hex = "#{:02x}{:02x}{:02x}".format(color[0], color[1], color[2])
            grid[y][x] = point(x, y, float(z), 0, 0,0, None, getTerrain(hex),False,False)
    if season=="winter" or season=="spring":
        frontier=[]
        for y in range(0, len(grid)):
            for x in range(0, len(grid[y])):
                if grid[y][x].terrain == Terrain.LAKE:
                    for neighbor in neighbors(grid,grid[y][x]):
                        if neighbor.terrain!= Terrain.LAKE and neighbor.terrain != Terrain.OUTOFBOUNDS:
                            if season=="winter":
                                frontier.append(neighbor)
                            else:
                                frontier.append((neighbor, grid[y][x].z))
        if season == "winter":
            Icebfs(pix, grid, frontier)
        else:
            Mudbfs(pix, grid, frontier)
    if season=="fall":
        for y in range(0, len(grid)):
            for x in range(0, len(grid[y])):
                if grid[y][x].terrain == Terrain.TRAIL:
                    for neighbor in neighbors(grid, grid[y][x]):
                        if neighbor.terrain == Terrain.EASYFOREST:
                            grid[y][x].terrain = Terrain.LEAVES
                            pix[x, y] = (120,120,140, 255)
    return terrain,grid

def Icebfs(pix,grid,frontier):
    q = queue.Queue()
    for curr in frontier:
        q.put(curr)
    q.put(None)
    depth=0
    while depth<7:
        new=q.get()
        if new==None:
            depth+=1
            q.put(None)
            continue
        for neighbor in neighbors(grid, grid[new.y][new.x]):
            if neighbor.terrain == Terrain.LAKE:
                neighbor.terrain=Terrain.ICE
                pix[neighbor.x,neighbor.y]=(123,255,255,255)
                q.put(neighbor)

def Mudbfs(pix,grid,frontier):
    q = queue.Queue()
    index=0
    visited=[]
    for curr in frontier:
        #print(curr[0],curr[1])
        if(curr[0].z-curr[1]<=1):
            q.put((curr[0],curr[1],index))
            curr[0].terrain = Terrain.MUD
            pix[curr[0].x, curr[0].y] = (141, 76, 0, 255)
            visited.append([])
            visited[index].append(curr[0])
            index+=1

    q.put(None)
    depth=0
    while depth<14:
        temp=q.get()
        if temp==None:
            depth+=1
            q.put(None)
            continue
        new, start ,index= temp[0], temp[1],temp[2]
        for neighbor in neighbors(grid, grid[new.y][new.x]):
            #print(neighbor.x,neighbor.y,neighbor.z-start)
            if (neighbor.terrain!=Terrain.MUD and neighbor.terrain!=Terrain.LAKE and neighbor.terrain!=Terrain.OUTOFBOUNDS) and neighbor.z-start<=6:
                neighbor.terrain=Terrain.MUD
                pix[neighbor.x,neighbor.y]=(141,76,0,255)
                q.put((neighbor,start,index))

def neighbors(map, current):
    x = current.x
    y = current.y
    neighbors = [(-1, -1), (0, -1), (1, -1),(-1, 0), (1, 0),(-1, 1), (0, 1), (1, 1)]
    #neighbors = [(0, -1),(-1, 0), (1, 0),(0, 1)]
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

def heuristic(start, goal):
    (x, y, z) = start.x, start.y, start.z
    (gx, gy, gz) = goal.x, goal.y, goal.z
    dx = abs(x - gx)*10.29
    dy = abs(y - gy)*7.55
    dz = abs(z - gz)
    score=math.sqrt(dx*dx+dy*dy)
    return math.sqrt(score*score+dz*dz)

def terrainHeuristic(start,goal):
    distance=heuristic(start,goal)
    terrainMulti=goal.terrain.value
    return terrainMulti*distance

def astar(map, start, goal):

    start = map[start[1]][start[0]]
    goal = map[goal[1]][goal[0]]
    start.f=start.h = heuristic(start, goal)

    visited = []
    heap = []
    heappush(heap, start)
    start.inHeap=True
    start.visited=True

    while heap:
        current = heappop(heap)
        current.inHeap=False
        current.visited=True
        visited.append(current)
        if current.x == goal.x and current.y == goal.y:
            distance = 0
            path = []
            while current:
                if current.prev is not None:
                    distance+=heuristic(current,current.prev)
                path.append((current.x, current.y))
                current = current.prev
            return path[::-1], visited,round(distance)

        for neighbor in neighbors(map, current):
            if neighbor.visited:
                continue
            neighborG = current.g + terrainHeuristic(current, neighbor)
            if neighbor.inHeap and neighborG >= neighbor.g:
                continue
            else:
                neighbor.prev = current
                neighbor.g = neighborG
                neighbor.h= heuristic(neighbor, goal)
                neighbor.f = neighborG+neighbor.h
                neighbor.inHeap=True
                heappush(heap, neighbor)


def runCourse(map, path_file):
    print("Running",path_file)
    with open(path_file) as textFile:
        points = [[int(x) for x in line.split()] for line in textFile]
    stops = [points[0]]
    paths = []
    visited = []
    startT = time.time()
    totalDistance=0
    print("Found point: ",end=" ")
    for i in range(0, len(points) - 1):
        start = points[i]
        next = points[i + 1]
        stops.append(next)
        path, visits, distance = astar(map, start, next)
        totalDistance+=distance
        print(points[0],",",points[1],end=" ")

        for p in path:
            if p not in paths:
                paths.append(p)

        for v in visits:
            if v.visited:
                visited.append(v)
        for y in range(0, len(map)):
            for x in range(0, len(map[y])):
                map[y][x].f = 0
                map[y][x].g = 0
                map[y][x].prev = None
                map[y][x].visited=False
                map[y][x].inHeap=False

    end = time.time()
    print()
    print("Time took:",end - startT)
    print("Total Distance: ",totalDistance)
    return paths, visited, stops

def main(terrain_image, elevation_file, path_file, season, output):
    season_image,map = makeMap(terrain_image, elevation_file,season)
    path, visited, stops = runCourse(map, path_file)
    #constructRender("output/elevationPathMap.png",map=map, path=path, stops=stops ) #Draw an elevation Map
    #constructRender("output/visitedMap.png",map=map,visited=visited,path=path,stops=stops ,outline=0) # Draw a visited map
    constructRender("output/"+output,terrain=season_image, path=path, stops=stops ,outline=1)

    #Render3d(map,terrain_image) # Make a 3D render!


if __name__ == '__main__':
    if(len(sys.argv)==6):
        main(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5])
    else:
        print("Arguments terrain.png mpp.txt red.txt winter redWinter.png")
    #main("testcases/default/terrain.png", "testcases/default/mpp.txt", "testcases/default/red.txt","summer","redWinter.png")
    #main("testcases/distanceCalc/terrain.png", "testcases/distanceCalc/mpp.txt", "testcases/distanceCalc/100Xath.txt", "winter","redWinter.png")
    #main("testcases/elevation/terrain.png", "testcases/elevation/mpp.txt", "testcases/elevation/elPath.txt", "winter","redWinter.png")
    #main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/brown.txt","winter","brown/brownWinter.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/red.txt","winter","red/redWinter.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/white.txt","winter","white/whiteWinter.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/brown.txt", "summer","brown/brownSummer.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/red.txt", "summer","red/redSummer.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/white.txt", "summer","white/whiteSummer.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/brown.txt", "fall","brown/brownFall.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/red.txt", "fall","red/redFall.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/white.txt", "fall","white/whiteFall.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/brown.txt", "spring","brown/brownSpring.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/red.txt", "spring","red/redSpring.png")
    # main("testcases/normal/terrain.png", "testcases/normal/mpp.txt", "testcases/normal/white.txt", "spring","white/whiteSpring.png")
