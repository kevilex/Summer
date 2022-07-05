#! /usr/bin/env python3
import cv2, numpy, os

for i in len(os.listdir(directory)):

    #formaterer til array og gjør farge-korrektur
    img = cv2.imread("/home/kevin/Pictures/Db/RAW" + i + ".jpg")
    img_array = numpy.array(img)
    img_rgb = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)

    #finner midten av bilde
    res = [img_rgb.shape[1], img_rgb.shape[0]]
    mid = [round(res[0]/2), round(res[1]/2)] 

    #cropper til område som burde inneholde objektet
    yCrop = [mid[1]-128, mid[1]+128]
    xCrop = [mid[0]-128, mid[0]+128]
    img_crop = img_rgb[yCrop[0]:yCrop[1], xCrop[0]:xCrop[1]]


    #CIFAR-10 specs
    cifarDim = [32,32]
    cifarImg_crop = img_rgb[xCrop[0]:xCrop[1], yCrop[0]:yCrop[1]]
    resize = cv2.resize(cifarImg_crop, dim, interpolation = cv2.INTER_AREA)
    CIFAR = cv2.resize(cifarImg_crop, dim, interpolation = cv2.INTER_AREA)
    cv2.imwrite("/home/kevin/Pictures/Db/CIFAR-10" + i + "CIFAR-10.jpg", CIFAR)


    #MNIST specs
    MNISTDim = [28,28]
    MINSTImg_crop = []
    resize = cv2.resize(img_rgb, dim, interpolation = cv2.INTER_AREA)
    MNIST = cv2.cvtColor(MINSTImg_crop, cv2.COLOR_RGB2GRAY)
    cv2.imwrite("/home/kevin/Pictures/Db/" + i + "MNIST.jpg", MNIST)

    #eventuelt legge til i database etterpå, ser også for meg at RAW mappen har undermapper med beskrivelse av objekt og bakgrunn
