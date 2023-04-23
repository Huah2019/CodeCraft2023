import cv2
import numpy as np
import os

file_list = ['1.txt', '2.txt']
h = 100
w = 100
scale = 10

locs = []
for i in range(1, 10):
    if i == 1:
        dir = "Img/1.png"
    elif i == 2:
        dir = "Img/2.png"
    elif i == 3:
        dir = "Img/3.png"
    elif i == 4:
        dir = "Img/4.png"
    elif i == 5:
        dir = "Img/5.png"
    elif i == 6:
        dir = "Img/6.png"
    elif i == 7:
        dir = "Img/7.png"
    elif i == 8:
        dir = "Img/8.png"
    else:
        dir = "Img/9.png"
    loc = cv2.imread(dir)
    loc = cv2.resize(loc, (20, 20))
    locs.append(loc)

for file in file_list:
    f = open(file)
    lines = f.readlines()
    img = np.zeros([h*scale, w*scale, 3], np.uint8)

    for i in range(h*scale):
        for j in range(w*scale):
            img[i][j] = [255, 255, 255]

    for i in range(h):
        for j in range(w):
            if lines[i][j] == '#':
                n = 10
                m = 10
                p = (i*scale-n//2, j*scale-m//2)
                for x in range(n):
                    for y in range(m):
                        img[p[0]+x][p[1]+y] = [0, 0, 0]

    for i in range(h):
        for j in range(w):
            if lines[i][j] >= '1' and lines[i][j] <= '9':
                loc = locs[ord(lines[i][j])-ord('1')]
                n, m, _ = loc.shape
                p = (i*scale-n//2, j*scale-m//2)
                for x in range(n):
                    for y in range(m):
                        img[p[0]+x][p[1]+y] = loc[x][y]
    id_r = 0
    for i in range(h):
        for j in range(w):
            x = j+1
            y = i+1
            center = (x*scale, y*scale)
            if lines[i][j] == 'A':
                radius = 5
                img = cv2.circle(img, center, radius, [
                    0, 255, 0], 2, cv2.LINE_AA)

    id_p = 0
    for i in range(h):
        for j in range(w):
            x = j+1
            y = i+1
            sx = x*scale-40
            if x*scale < 100:
                sx += 50
            sy = y*scale-10
            if y*scale < 100:
                sy += 10
            center = (sx, sy)
            if lines[i][j] >= '1' and lines[i][j] <= '9':
                radius = 6
                type = ord(lines[i][j])-ord('0')
                color = (255, 0, 0)
                if id_p % 2 == 1:
                    color = (0, 0, 255)
                img = cv2.putText(
                    img, str(id_p), center, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                id_p += 1

    name, format = file.split('.')
    cv2.imwrite(name+".jpg", img)
