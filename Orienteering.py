from typing import Any
import typing
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
    grid = [[point for i in range(0, len(elev[0]) - 5)] for j in range(0, len(elev))]
    for y in range(0, len(grid)):
        for x in range(0, len(grid[y])):
            whole = elev[y][x].split('e+')
            elev[y][x] = round(float(whole[0]) * int(math.pow(10, int(whole[1]))), 6)
            z = elev[y][x]
            color = pix[x, y]
            hex = rgb2hex(color[0], color[1], color[2])
            grid[y][x] = point(x, y, float(z),0,0,0, getTerrain(hex))
    return grid


@dataclass
class point:
    x: int
    y: int
    z: float
    f: int
    h: int
    g: int
    terrain: Terrain

def neighbors(map,current):
       (x, y) = current.x,current.y
       neighbors=[(-1,-1),(0,-1),(1,-1),
                  (-1, 0),       (1, 0),
                  (-1, 1),(0, 1),(1, 1),]
       result=[]
       for i,j in neighbors:
           if x+i >0 and x+i < len(map[0]) and y+j>0 and y+j < len(map):
               result.append(map[y+j][x+i])
       return result

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


def heuristic(start, goal):
    (x,y,z)=start.x,start.y,start.z
    (gx,gy,gz)=goal.x,goal.y,goal.z
    xscore=abs(x-gx)
    yscore=abs(y-gy)
    zscore=abs(z-gz)
    #print(xscore,yscore,zscore)
    score=xscore+yscore+zscore
    return score
def astar(map, start, goal):

    start = map[start[1]][start[0]]
    goal = map[goal[1]][goal[0]]
    #start.h=heuristic(start,goal)
    gscore={(start.x,start.y):0}
    fscore={(start.x,start.y):heuristic(start,goal)}
    closed=[]
    parents=[]
    heap=[start]
    while heap:
        print(heap)
        current = heappop(heap)

        if current == goal:
            path = []
            path.append(current)
            while current in parents:
                path.append(current)

                current = parents[parents.index(current)]
            return path
        closed.append(current)

        for neighbor in neighbors(map,current):
            score=gscore[(current.x,current.y)]+heuristic(current,neighbor)
            if neighbor in closed and score>= gscore.get((neighbor.x,neighbor.y),0):
                continue
            if score < gscore.get((neighbor.x,neighbor.y),0) or neighbor not in [i[1]for i in heap]:
                parents[neighbor]=(current.x,current.y)
                gscore[(neighbor.x,neighbor.y)]=score
                fscore[(neighbor.x,neighbor.y)]=score+heuristic(neighbor,goal)
                heappush(heap,(fscore[(neighbor.x,neighbor.y)], neighbor))



def main(terrain_image, elevation_file, path_file, season, output):
    print("test")
    map =makeMap(terrain_image, elevation_file)
    #printMap(map)
    start = (168, 236)
    goal = (178, 222)
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    pix[168, 236] = (255, 0, 0, 255)
    pix[178, 222] = (255, 0, 0, 255)
    print(astar(map, start, goal))
    terrain.save("save.png")


if __name__ == '__main__':
    main("terrain.png", "mpp.txt", "red.txt", "winter", "redWinter.png")