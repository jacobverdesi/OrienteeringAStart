from PIL import Image
import math
from enum import Enum
from dataclasses import dataclass

class Terrain(Enum):
    OPENLAND=1
    ROUGHMEADOW=2
    EASYFOREST=3
    SLOWRUNFOREST=4
    WALKFOREST=5
    IMPASSABLEVEG=6
    LAKE=7
    ROAD=8
    TRAIL=9
    OUTOFBOUNDS=10
@dataclass
class point():
    x:int
    y:int
    z:int
    color:str
def rgb2hex(r,g,b):
    return "#{:02x}{:02x}{:02x}".format(r,g,b)
def getTerrain(color):
    if color=="#F89412":
        return Terrain.OPENLAND
    elif color=="#FFC000":
        return Terrain.ROUGHMEADOW
    elif color=="#FFFFFF":
        return Terrain.EASYFOREST
    elif color=="#02D03C":
        return Terrain.EASYFOREST
    elif color=="#028828":
        return Terrain.EASYFOREST
    elif color=="#054918S":
        return Terrain.EASYFOREST
    elif color=="FFFFFF":
        return Terrain.EASYFOREST

def main(terrain_image,elevation_file,path_file,season,output):
    terrain=Image.open(terrain_image)
    pix=terrain.load()

    # pix[0,0] = (0,0,0,0)
    # print(pix[0, 0])
    # terrain.save("save.png")
    with open(elevation_file) as textFile:
        elev = [line.split() for line in textFile]
    points=[]
   # m=map(len(elev[0])-5,len(elev))

    for y in range(0,len(elev)):
        for x in range(0,len(elev[y])-5):
            whole=elev[y][x].split('e+')
            elev[y][x]=round(float(whole[0])*int(math.pow(10,int(whole[1]))),3)
            z=elev[y][x]
            color=pix[x,y]
            color=rgb2hex(color[0],color[1],color[2])
            points.append(point(x,y,int(z),color))

    for i in points:
        print(i)



if __name__ == '__main__':
    main("terrain.png", "mpp.txt", "red.txt", "winter", "redWinter.png")