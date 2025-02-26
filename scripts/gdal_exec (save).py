#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


origen_z = 0
TrackbarName = ""
rowf = 0
rowi = 0
colf = 0
coli = 0
cell_size = 2
Y1 = 0
rectangulo_x_down = 0
rectangulo_y_down = 0
rectangulo_x_up = 0
rectangulo_y_up = 0
contador_L = 0
fp = None
fp2 = None
fp3 = None
realimage = None
realimage_clean = None
image = None
dem = None
demcropped = None
demcropped2 = None
miniDem = None
miniDem2 = None
miniDem3 = None
miniLaCabrera = None
whiteImage = None
goalsImage = None
showImage = None
copiashowImage = None
x = 0
y = 0
key = ''
alpha_slider = 0
alpha_slider_max = 80
alpha = 0.0
tl = np.array([-3.6883697, 40.9166661])
tr = np.array([-3.5208329, 40.9175445])
bl = np.array([-3.6874979, 40.8324559])
br = np.array([-3.5201733, 40.8333317])
dem_bl = np.array([-3.6881451, 40.8319589])
dem_tr = np.array([-3.5202371, 40.9180035])

resolution = 2.0
resize_factor = 10

def lerp(p1, p2, t):
    return np.array([((1 - t) * p1[0]) + (t * p2[0]), ((1 - t) * p1[1]) + (t * p2[1])])

def world2dem(coordinate, dem_size):
    demRatioX = ((dem_tr.x - coordinate.x) / (dem_tr.x - dem_bl.x))
    demRatioY = 1 - ((dem_tr.y - coordinate.y) / (dem_tr.y - dem_bl.y))
    output = np.array([demRatioX * dem_size.width, demRatioY * dem_size.height])
    return output

def pixel2world(x, y, size):
    rx = x / miniLaCabrera.shape[1]
    ry = y / miniLaCabrera.shape[0]
    rightSide = lerp(tr, br, ry)
    leftSide = lerp(tl, bl, ry)
    return lerp(leftSide, rightSide, rx)

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.count_ = 0

def Remap(value, from1, to1, from2, to2):
    return (value - from1) / (to1 - from1) * (to2 - from2) + from2

def MAX_angle(a, b, c, d, f, g, h, i):
    return max(a, b, c, d, f, g, h, i)

def newAlgorithm3():
    global demcropped, demcropped2, miniDem, miniDem2, miniLaCabrera, goalsImage, realimage, origen_z
    demcropped = dem
    demcropped2 = np.zeros(demcropped.shape, dtype=np.int16)
    iWidth = demcropped.shape[1]
    iHeight = demcropped.shape[0]
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
    for x in range(1, iWidth - 1):
        for y in range(1, iHeight - 1):
            if x == iWidth // 2 and y == iHeight // 2:
                origen_z = demcropped[y, x]
                cv.circle(realimage, (x, y), 3, (0, 0, 0), cv.FILLED, cv.LINE_8, 0)
                cv.circle(miniLaCabrera, (x, y), 3, (0, 0, 0), cv.FILLED, cv.LINE_8, 0)
                cv.circle(goalsImage, (x, y), 3, (0, 0, 0), cv.FILLED, cv.LINE_8, 0)
                cv.circle(miniDem2, (x, y), 3, (0, 0, 0), cv.FILLED, cv.LINE_8, 0)
                cv.circle(miniDem, (x, y), 3, (0, 0, 0), cv.FILLED, cv.LINE_8, 0)
            pixel_x = np.sum(sobel_x * demcropped[y-1:y+2, x-1:x+2]) / (8 * cell_size)
            pixel_y = np.sum(sobel_y * demcropped[y-1:y+2, x-1:x+2]) / (8 * cell_size)
            val = int(np.sqrt(pixel_x**2 + pixel_y**2) * 100)
            if val > 255:
                val = 255
            demcropped2[y, x] = val
    _, miniDem = cv.threshold(demcropped2, alpha, 255, cv.THRESH_BINARY_INV)

def on_trackbar_alpha(val):
    global alpha, miniDem
    alpha = val
    _, miniDem = cv.threshold(demcropped2, alpha, 255, cv.THRESH_BINARY_INV)
    miniDem = miniDem.astype(np.uint8)

    cv.imshow("Sobel Gradient", miniDem)

def drawCircle2(event, x, y, flags, param):
    global contador_L, fp2, Y1, x_init,y_init, x_final, y_final
    Y1 = abs(y - miniDem.shape[0])
    print("x: ", x, "y: ", y)

    if event == cv.EVENT_LBUTTONDOWN and contador_L == 0:
        contador_L = 1
        cv.circle(miniLaCabrera, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(goalsImage, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(realimage, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem2, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        x_init = x
        y_init = y

    elif event == cv.EVENT_LBUTTONDOWN and contador_L == 1:
        contador_L = 0
        cv.circle(miniLaCabrera, (x, y), 3, (0, 0, 255), cv.FILLED, cv.LINE_8, 0)
        cv.circle(realimage, (x, y), 3, (0, 0, 255), cv.FILLED, cv.LINE_8, 0)
        cv.circle(goalsImage, (x, y), 3, (0, 0, 255), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem2, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        x_final = x
        y_final = y


def add_color(pix, b, g, r):
    if pix[0] + b < 255 and pix[0] + b >= 0:
        pix[0] += b
    if pix[1] + g < 255 and pix[1] + g >= 0:
        pix[1] += g
    if pix[2] + r < 255 and pix[2] + r >= 0:
        pix[2] += r

def Welcome_message():
    text = ("La interfaz de este TFG te permite seleccionar graficamente \nel punto de origen y destino sobre un mapa de ejemplo.\nA continuacion, podras planificar la trayectoria entre los\npuntos mediante la interfaz de Rviz.\n\nPulsando los botones '1', '2','3','4' o '5' podras intercambiar\nentre distintas vistas del mismo mapa. Haciendo click izquierdo podras \nseleccionar el punto deseado como origen, y en segundo lugar \ncomo destino.\n\nPulsando y arrastrando el slider podras variar el grado de \ninclinacion del mapa de coste.Finalmente, podras guardar \nlos resultados pulsando la letra 's' del teclado.\nSi deseas empezar de cero puedes puslar la letra 'r'.\n\nPulsa cualquier tecla para empezar")
    fontFace = cv.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    thickness = 1
    textSize = cv.getTextSize("La interfaz de este TFG te permite...........................", fontFace, fontScale, thickness)[0]
    canvas = np.zeros((textSize[1] * 33, int(textSize[0] * 1.5), 3), dtype=np.uint8)
    canvas.fill(0)
    line = 0
    for lineText in text.split('\n'):
        textOrg = (0, textSize[1] * (line + 1))
        cv.putText(canvas, lineText, textOrg, fontFace, fontScale, (255, 255, 255), thickness)
        line += 2
    cv.imshow("Bienvenido", canvas)
    cv.waitKey(0)
    cv.destroyAllWindows()


def main(args=None):
    global image, realimage, dem, miniLaCabrera, miniDem2, miniDem, goalsImage, copiashowImage, key
    rclpy.init(args=args)
    Welcome_message()
    while True:
        image = cv.imread("/home/david/ros2_ws/src/cv_gdal/resources/final/xxxm/mapa_XXXm.tif", cv.IMREAD_LOAD_GDAL | cv.IMREAD_COLOR)
        realimage = cv.imread("/home/david/ros2_ws/src/cv_gdal/resources/final/xxxm/mapa_XXXm_real.tif", cv.IMREAD_LOAD_GDAL | cv.IMREAD_COLOR)
        dem = cv.imread("/home/david/ros2_ws/src/cv_gdal/resources/final/xxxm/prueba_XXXm9.tif", cv.IMREAD_LOAD_GDAL | cv.IMREAD_ANYDEPTH)
        dem = dem.astype(np.int16)
        realimage = cv.resize(realimage, dem.shape[::-1], interpolation=cv.INTER_LINEAR)
        if dem.dtype != np.int16:
            raise RuntimeError("DEM image type must be CV_16SC1")

        whiteImage = np.full(dem.shape, 255, dtype=np.uint8)
        goalsImage = whiteImage.copy()
        copiashowImage = whiteImage.copy()
        newAlgorithm3()
        miniLaCabrera = cv.resize(image, dem.shape[::-1], interpolation=cv.INTER_LINEAR)
        miniDem2 = demcropped2.astype(np.uint8)
        miniDem2 = cv.applyColorMap(miniDem2, cv.COLORMAP_JET)
        cv.imshow("Real Map", realimage)
        
        while key != 27:
            key = cv.waitKey(0)
            cv.destroyAllWindows()
            if key == ord('1'):            
                cv.namedWindow("Real Map", cv.WINDOW_GUI_NORMAL)

                cv.imshow("Real Map", realimage)
            elif key == ord('2'):
                cv.namedWindow("Sobel Gradient", cv.WINDOW_GUI_NORMAL)
                cv.createTrackbar("Percentage of slope:", "Sobel Gradient", alpha_slider, alpha_slider_max, on_trackbar_alpha)
                on_trackbar_alpha(alpha_slider)
                cv.setMouseCallback("Sobel Gradient", drawCircle2)

            elif key == ord('3'):
                cv.namedWindow("La Cabrera", cv.WINDOW_GUI_NORMAL)
                cv.createTrackbar("Percentage of slope:", "La Cabrera", alpha_slider, alpha_slider_max, on_trackbar_alpha)

                cv.imshow("La Cabrera", miniLaCabrera)
            elif key == ord('4'):
                cv.namedWindow("Gradient Color Map", cv.WINDOW_GUI_NORMAL)

                cv.imshow("Gradient Color Map", miniDem2)
            elif key == ord('5'):
                cv.namedWindow("Goals Map", cv.WINDOW_GUI_NORMAL)

                cv.imshow("Goals Map", goalsImage)
            elif key == ord('s'):
                miniDemresized=cv.resize(miniDem, tuple(int(resize_factor)*i for i in miniDem.shape[::-1]), interpolation=cv.INTER_LINEAR)
                cv.imwrite("/home/david/ros2_ws/src/cv_gdal/maps/map1.pgm", miniDemresized)
                with open("/home/david/ros2_ws/src/cv_gdal/maps/map1.yaml", "w") as yaml_file:
                    yaml_file.write("image: map1.pgm\n")
                    yaml_file.write("resolution: "+str(resolution/int(resize_factor))+"\n")
                    yaml_file.write("origin: ["+str(-x_init*resolution)+","+str(-((miniDem.shape[1]-y_init)*resolution))+",0.00]\n")
                    yaml_file.write("destiny: ["+str(-x_final*resolution)+","+str(-((miniDem.shape[1]-y_final)*resolution))+",0.00]\n")
                    yaml_file.write("map_size: ["+str(miniDem.shape[0]*resolution)+","+str(miniDem.shape[1]*resolution)+",0.00]\n")
                    yaml_file.write("negate: 0\n")
                    yaml_file.write("occupied_thresh: 0.65\n")
                    yaml_file.write("free_thresh: 0.196\n")
                print("map and coordinates saved")
        return

if __name__ == '__main__':
    main()