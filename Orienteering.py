from typing import Any
import typing
from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass
from heapq import heappush, heappop
from cv2 import VideoWriter, VideoWriter_fourcc, imread, resize
import os

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
            grid[y][x] = point(x, y, float(z),0,0,0,None, getTerrain(hex))
    return grid


def make_video(images, outimg=None, fps=5, size=None,
               is_color=True, format="XVID"):
    """
    Create a video from a list of images.

    @param      outvid      output video
    @param      images      list of images to use in the video
    @param      fps         frame per second
    @param      size        size of each frame
    @param      is_color    color
    @param      format      see http://www.fourcc.org/codecs.php
    @return                 see http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html

    The function relies on http://opencv-python-tutroals.readthedocs.org/en/latest/.
    By default, the video will have the size of the first image.
    It will resize every image to this size before adding them to the video.
    """
    fourcc = VideoWriter_fourcc(*format)
    vid = None
    for image in images:
        image.save("imagetest.png")

        # if not os.path.exists(image):
        #     raise FileNotFoundError(image)
        img = imread("imagetest.png")
        if vid is None:
            if size is None:
                size = img.shape[1], img.shape[0]
            vid = VideoWriter(outimg, fourcc, float(fps), size, is_color)
        if size[0] != img.shape[1] and size[1] != img.shape[0]:
            img = resize(img, size)
        vid.write(img)
    vid.release()
    return vid

@dataclass
class point:
    x: int
    y: int
    z: float
    f: int
    h: int
    g: int
    prev: Any
    terrain: Terrain

    def __lt__(self, other):
        return self.f < other.f

def neighbors(map,current):
       x = current.x
       y = current.y
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
    score=xscore+yscore+zscore*3
    return score
def astar(map, start, goal):
    start = map[start[1]][start[0]]
    goal = map[goal[1]][goal[0]]
    start.f=heuristic(start,goal)
    print("Distance: ",start.f)
    closed=[]
    heap=[]
    heappush(heap,start)
    index=0
    images=[]
    elevation = Image.new("RGBA", (len(map[0]), len(map)))
    elev = elevation.load()

    while heap:
        current = heappop(heap)
        index+=1
        z=int(current.z)
        z = int(255*index/4000)
        elev[current.x, current.y] = (z,z,z, 255)
        elevation.save('elev2.png')
        images.append(elevation)
        #print(current)
        print("Estimated distance from: ",heuristic(current,goal),"/",start.f)
        if current.x == goal.x and current.y == goal.y:
            path = []
            while current:
                path.append((current.x,current.y))
                current = current.prev
            return path[::-1] ,images

        closed.append(current)

        for neighbor in neighbors(map,current):
            score=current.g+heuristic(current,neighbor)
            if neighbor in closed and score>= neighbor.g:
                continue
            if score < neighbor.g or neighbor not in heap:
                neighbor.prev=current
                neighbor.g=score
                neighbor.f=score+heuristic(neighbor,goal)
                heappush(heap,neighbor)


def main(terrain_image, elevation_file, path_file, season, output):
    print("test")
    map =makeMap(terrain_image, elevation_file)
    #printMap(map)
    start = (100, 100)
    goal = (100,130)
    terrain = Image.open(terrain_image)
    pix = terrain.load()


    #path=astar(map,start,goal)
    #for x,y in path:
    #    pix[x,y]=(255,0,0,255)
    pix[start[0], start[1]] = (255, 255, 0, 255)
    pix[goal[0], goal[1]] = (255, 255, 0, 255)
    #terrain.save("save.png")
    elevation = Image.new("RGBA", (len(map[0]), len(map)))
    elev = elevation.load()
    max,min=187,250

    # for y in range(0, len(map)):
    #     for x in range(0, len(map[0])):
    #         z=int(map[y][x].z)
    #         z=(z-187)*4
    #         if z<60:
    #             elev[x,y]=(z,z,z,255)
    #         elif z>=60 and z < 130:
    #             elev[x, y] = (z, z, z, 255)
    #         elif z>=130:
    #             elev[x, y] = (z, z, z, 255)
    path,images=astar(map,start,goal)
    for x,y in path:
       elev[x,y]=(255,0,0,255)
    elev[start[0], start[1]] = (255, 255, 0, 255)
    elev[goal[0], goal[1]] = (255, 255, 0, 255)
    images[len(images)-1].save("elev.png")
    make_video(images,'output.avi')


if __name__ == '__main__':
    main("terrain.png", "mpp.txt", "red.txt", "winter", "redWinter.png")