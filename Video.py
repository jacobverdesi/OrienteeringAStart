from cv2 import VideoWriter, VideoWriter_fourcc, imread, resize
def make_video(index, outimg=None, fps=15, size=None,is_color=True, format="XVID"):

    fourcc = VideoWriter_fourcc(*format)
    vid = None
    for i in range(1,index):

        img = imread("images/image"+str(i)+".png")
        if vid is None:
            if size is None:
                size = img.shape[1], img.shape[0]
            vid = VideoWriter(outimg, fourcc, float(fps), size, is_color)
        if size[0] != img.shape[1] and size[1] != img.shape[0]:
            img = resize(img, size)
        vid.write(img)
    vid.release()
    return vid