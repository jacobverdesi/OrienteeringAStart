from PIL import Image
import math




def main(terrain_image,elevation_file,path_file,season,output):
    terrain=Image.open(terrain_image)
    # pix=terrain.load()
    # print(pix[0,0])
    # pix[0,0] = (0,0,0,0)
    # print(pix[0, 0])
    # terrain.save("save.png")
    with open(elevation_file) as textFile:
        elev = [line.split() for line in textFile]

    #print
    vertices=[]
    edge=[]
    for y in range(0,len(elev)):
        for x in range(0,len(elev[y])):
            whole=elev[y][x].split('e+')
            elev[y][x]=round(float(whole[0])*int(math.pow(10,int(whole[1]))),3)
            z=elev[y][x]
            vertices.append((x,y,z))
           # print(vertices)
    for y in range(0, len(elev)):
        for x in range(0, len(elev[y])):
            #print('X: ',x,' Y: ',y,' Z: ',elev[y][x])
            curr=vertices.index((x,y,elev[y][x]))
            if x>0 :
                edge.append((curr,vertices.index((x-1,y,elev[y][x-1]))))  #west
            if x<len(elev[y])-1:
                edge.append((curr,vertices.index((x+1, y, elev[y][x+1])))) #east
            if y>0:
                edge.append((curr, vertices.index((x, y - 1, elev[y-1][x]))))  #north
            if y<len(elev)-1:
                edge.append((curr, vertices.index((x, y + 1, elev[y + 1][x]))))  #
        print(y/len(elev)*100,'%')
    print(edge)
    print(len(edge))

if __name__ == '__main__':
    main("terrain.png", "mpp2.txt", "red.txt", "winter", "redWinter.png")