from PIL import Image





def main(terrain_image,elevation_file,path_file,season,output):
    terrain=Image.open(terrain_image)
    pix=terrain.load()
    print(pix[0,0])
    pix[0,0] = (0,0,0,0)
    print(pix[0, 0])
    terrain.save("save.png")
    elev=[[0]*395]*500
    for line in open(elevation_file):
        elev=line.split()
        print(elev)

if __name__ == '__main__':
    main("terrain.png", "mpp.txt", "red.txt", "winter", "redWinter.png")