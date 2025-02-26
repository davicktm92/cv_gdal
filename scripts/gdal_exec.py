#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rasterio
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


origen_z = 0
rowf = 0
rowi = 0
colf = 0
coli = 0
cell_size = 2
Y1 = 0
contador_L = 0
fp2 = None
realimage = None
image = None
dem = None
demcropped = None
demcropped2 = None
miniDem = None
miniDem2 = None
miniLaCabrera = None
goalsImage = None
x_init = 0
y_init = 0
x_final = 0
y_final = 0
alpha_slider = 0
alpha_slider_max = 80
alpha = 0.0

resize_factor = 5

zone_name="canarias"

rosRate=1

class gdal_node(Node):
    def __init__(self):
        super().__init__('gdal_node')
        self.publisher_ = self.create_publisher(Image, 'gdal_image', 10)
        self.timer = self.create_timer(1/rosRate, self.timer_callback)

        self.count_ = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.origin=[]
        self.destiny=[]
        with open(os.path.join(get_package_share_directory('cv_gdal'), 'maps', 'map1.yaml'), 'r') as file:
            map_conf = yaml.safe_load(file)
            self.origin.append(map_conf['origin'])
            self.destiny.append(map_conf['destiny'])
        


    def timer_callback(self):
        global realimage, dir
        msg = Image()
        # Fill in the Image message with the appropriate data
        if realimage is not None:
            realimage_rgb = cv.cvtColor(realimage, cv.COLOR_GRAY2RGB)
            resolution = calculate_resolution(dir)[0]

            #add circles for initial and final points
            self.x_pixel_init = int(-self.origin[0][0]/resolution)
            self.y_pixel_init = realimage.shape[0]-int(-self.origin[0][1]/resolution)
            self.x_pixel_final = int(-self.destiny[0][0]/resolution)
            self.y_pixel_final = realimage.shape[0]-int(-self.destiny[0][1]/resolution)

            cv.circle(realimage_rgb, (self.x_pixel_init, self.y_pixel_init), 5, (255, 0, 0), -1)
            cv.circle(realimage_rgb, (self.x_pixel_final,self.y_pixel_final), 5, (0, 0, 255), -1)
            try:
                t = self.tf_buffer.lookup_transform("map", "robot1/base_link", rclpy.time.Time().to_msg())
               
                # Convert the transform from position in tf to pixels in realimage
                pixel_x = int(t.transform.translation.x / resolution)
                pixel_y = -int(t.transform.translation.y / resolution)
              

                cv.circle(realimage_rgb, (self.x_pixel_init+pixel_x, self.y_pixel_init+pixel_y), 5, (255, 0, 0), -1)

            except TransformException as e:
                self.get_logger().info('Could not get transform: %s' % e)
            
            msg.height, msg.width, _ = realimage_rgb.shape
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = msg.width * 3
            msg.data = realimage_rgb.flatten().tolist()

            self.publisher_.publish(msg)
        else:
            self.get_logger().info('No image to publish')


def newAlgorithm3():
    global demcropped, demcropped2, miniDem, miniDem2, miniLaCabrera, goalsImage, realimage, origen_z
    demcropped = dem
    demcropped2 = np.zeros(demcropped.shape, dtype=np.int16)

    sobel_x = cv.Sobel(demcropped, cv.CV_64F, 1, 0, ksize=3) / (8 * cell_size)
    sobel_y = cv.Sobel(demcropped, cv.CV_64F, 0, 1, ksize=3) / (8 * cell_size)
    magnitude = cv.magnitude(sobel_x, sobel_y) * 100
    np.clip(magnitude, 0, 255, out=magnitude)
    demcropped2 = magnitude.astype(np.uint8)

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

    if event == cv.EVENT_LBUTTONDOWN and contador_L == 0:
        contador_L = 1
        cv.circle(miniLaCabrera, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(goalsImage, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(realimage, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem2, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        x_init = x
        y_init = y
        print("x_init: ", x_init, "y_init: ", y_init)

    elif event == cv.EVENT_LBUTTONDOWN and contador_L == 1:
        contador_L = 0
        cv.circle(miniLaCabrera, (x, y), 3, (0, 0, 255), cv.FILLED, cv.LINE_8, 0)
        cv.circle(realimage, (x, y), 3, (0, 0, 255), cv.FILLED, cv.LINE_8, 0)
        cv.circle(goalsImage, (x, y), 3, (0, 0, 255), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem2, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        cv.circle(miniDem, (x, y), 3, (255, 0, 0), cv.FILLED, cv.LINE_8, 0)
        x_final = x
        y_final = y
        print("x_final: ", x_final, "y_final: ", y_final)


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

def calculate_resolution(geotiff_path):
    with rasterio.open(geotiff_path) as dataset:
        transform = dataset.transform
        resolution_x = transform[0]
        resolution_y = -transform[4]
        return resolution_x, resolution_y


def main(args=None):
    global image, realimage, dem, miniLaCabrera, miniDem2, miniDem, goalsImage, copiashowImage, dir
    rclpy.init(args=args)


    #variables
    key=''

    #image import
    dir=os.path.join(get_package_share_directory('cv_gdal'), 'resources', 'tif', zone_name+'.tif')

    #image and yaml export
    im_dir=os.path.join(get_package_share_directory('cv_gdal'), 'maps', 'map1.pgm')
    yaml_dir=os.path.join(get_package_share_directory('cv_gdal'), 'maps', 'map1.yaml')

    image = cv.imread(dir, cv.IMREAD_LOAD_GDAL | cv.IMREAD_COLOR)
    realimage = cv.imread(dir,cv.IMREAD_UNCHANGED)
    dem = cv.imread(dir, cv.IMREAD_LOAD_GDAL | cv.IMREAD_ANYDEPTH)

    #image processing
    dem = dem.astype(np.int16)
    #realimage =(210-realimage).astype(np.uint8)
    if len(dem.shape) == 3:
        dem=dem[:,:,0]
        realimage=realimage[:,:,0]
    realimage = cv.resize(realimage, dem.shape[::-1], interpolation=cv.INTER_LINEAR)
    max_value = max(realimage.flatten())
    realimage = np.where(realimage < 0, max_value, realimage)
    realimage = cv.normalize(realimage, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)

    if dem.dtype != np.int16:
        raise RuntimeError("DEM image type must be CV_16SC1")

    whiteImage = np.full(dem.shape, 255, dtype=np.uint8)
    goalsImage = whiteImage.copy()
    copiashowImage = whiteImage.copy()

    Welcome_message()

    newAlgorithm3()

    resolution=calculate_resolution(dir)[0]
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
            #cv.imwrite("/home/david/ros2_ws/src/cv_gdal/maps/map1.pgm", miniDemresized)
            #with open("/home/david/ros2_ws/src/cv_gdal/maps/map1.yaml", "w") as yaml_file:
            cv.imwrite(im_dir, miniDemresized)
            with open(yaml_dir, "w") as yaml_file:
                yaml_file.write("image: map1.pgm\n")
                yaml_file.write("resolution: "+str(resolution/int(resize_factor))+"\n")
                yaml_file.write("origin: ["+str(-x_init*resolution)+","+str(-((miniDem.shape[0]-y_init)*resolution))+",0.00]\n")
                yaml_file.write("destiny: ["+str(-x_final*resolution)+","+str(-((miniDem.shape[0]-y_final)*resolution))+",0.00]\n")
                yaml_file.write("map_size: ["+str(miniDem.shape[1]*resolution)+","+str(miniDem.shape[0]*resolution)+",0.00]\n")
                yaml_file.write("zone_name: "+zone_name+"\n")
                yaml_file.write("negate: 0\n")
                yaml_file.write("occupied_thresh: 0.65\n")
                yaml_file.write("free_thresh: 0.196\n")
            print("map and coordinates saved")
            break
    

    gdal_node1 = gdal_node()
    try:
        rclpy.spin(gdal_node1)
    except KeyboardInterrupt:
        pass
    gdal_node1.destroy_node()

if __name__ == '__main__':
    main()