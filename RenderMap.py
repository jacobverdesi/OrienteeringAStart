from PIL import Image
import plotly.graph_objects as go

def Render3d(map,terrain_image):
    dataX=[]
    dataY=[]
    dataZ=[]
    dataColor=[]
    terrain = Image.open(terrain_image)
    pix = terrain.load()
    for y in range(0, len(map)):
        lineX=[]
        lineY=[]
        lineZ=[]

        for x in range(0, len(map[y])):

            z=map[y][x].z
            lineX.append(x)
            lineY.append(y)
            lineZ.append(z)
        dataX.append(lineX)
        dataY.append(lineY)
        dataZ.append(lineZ)
    dataY.reverse()
    fig = go.Figure(data=[go.Surface(z=dataZ,x=dataX,y=dataY)])
    fig.update_layout(title='Orienteering', autosize=True, scene=dict(aspectratio=dict(y=10.29,x=7.55,z=1)),
                      margin=dict(l=65, r=50, b=65, t=90))
    fig.show()
def drawElevationMap(map):
    terrain = Image.new("RGBA",(len(map[0]),len(map)))
    pix = terrain.load()
    for y in range(0, len(map)):
        for x in range(0, len(map[0])):
            z = int(map[y][x].z)
            z = (z - 187) * 4
            #pix[x,y]=(0,z,0,255)
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
    return terrain
def visitedMap(terrain_image,visited):
    pix = terrain_image.load()
    for index in range(0, len(visited)):
        visit = visited[index]
        z = int(190* index / len(visited))
        if index == 0 or index == len(visited):
            pix[visit.x, visit.y] = (255, 60, 255, 255)
        pix[visit.x, visit.y] = (60+z, 245, 60+(190-z), 255)

    return terrain_image


def drawPath(terrain_image, path,outline):
    pix = terrain_image.load()
    for x, y in path:
        if outline==1:
            for i in range (-1,2):
                for j in range(-1,2):
                    if pix[x+i,y+j]!=(150, 50, 150, 255):
                        if x!=0 or y!=0 :
                            pix[x+i, y+j] = (200, 100, 235, 255)
        pix[x, y] = (150, 50, 150, 255)
    return terrain_image

def drawStops (terrain_image,stops,outline):
    pix = terrain_image.load()

    for i in range(0, len(stops)):
        pix[stops[i][0], stops[i][1]] = (100, 50, 230, 255)
        if outline==1:
            for y in range(-2,3,1):
                for x in range(-2,3,1):
                    if x != 0 or y != 0:
                        pix[stops[i][0]+x, stops[i][1]+y] = (50, 20, 180, 255)
        else:
            pix[stops[i][0], stops[i][1]] = (50, 20, 180, 255)

    start, goal = stops[0], stops[len(stops) - 1]
    pix[start[0], start[1]] = (255, 100, 50, 255)
    pix[goal[0], goal[1]] = (255, 100, 50, 255)

    return terrain_image

def constructRender(output,terrain=None, map=None,visited=None,path=None,stops=None,outline=0):
        if terrain != None:
            terrain_image=terrain
        elif map != None:
            terrain_image=drawElevationMap(map)
        if visited!= None:
            terrain_image=visitedMap(terrain_image,visited)
        if path!= None :
            terrain_image=drawPath(terrain_image,path,outline)
        if stops!= None :
            terrain_image=drawStops(terrain_image,stops,outline)
        terrain_image.save(output)

    #make_video(len(visited), 'output/output.avi', 100)