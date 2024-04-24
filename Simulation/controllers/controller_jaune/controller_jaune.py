# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2022b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Taynaud
# janvier 2023

from controller import Gyro
from controller import Lidar
from vehicle import Driver
from delta_angle import *
from filtrage import *
from interpolation_lin import *
from control import control
import time

import numpy as np

#from matplotlib import pyplot as plt

import math

vmin = 5
vmax = 8
d = 2.3

# FILTRAGE

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = basicTimeStep

# Lidar
lidar = Lidar("LidarNouveau")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

gyro = Gyro("TT02_Gyro")
gyro.enable(int(1000/lidar.getFrequency()))
absolute_angle = 0

keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# facteur de bruit
alpha = 0.004

# vitesse en km/h
speed = 0
#maxSpeed = 5

#
points = 1600

# angle de la direction en rad
angle = 0
maxangle = 0.35  # à modifier car nous n'avons pas fait le calcul sans transmission

# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

previous_time = time.time() 

while driver.step() != -1:
    current_time = time.time()
    time_diff = current_time - previous_time
    print(time_diff)
    previous_time = current_time

    speed = driver.getTargetCruisingSpeed()
    donnees_lidar = lidar.getRangeImage() #liste de 1600 mesures d'angles
    donnees_gyro = gyro.getValues()
#    print(donnees_gyro)
    if(not np.isnan(donnees_gyro[2])):
        absolute_angle = (absolute_angle - np.rad2deg(donnees_gyro[2]) * time_diff) % 360
#    print(absolute_angle)
#    print(donnees_gyro[2])
    d = np.asarray(donnees_lidar[1200:] + donnees_lidar[:(points//4)])
    a = np.deg2rad(np.linspace(-90,90,(points//2)))
    #d = np.asarray(donnees_lidar)
    #a = np.deg2rad(np.linspace(0, 360, 1600))
    
    d = np.random.normal(d, alpha * d)
    d = interpolation_lin(d)
    
    # affichage des donnees captees par le lidar
    #x = d * np.cos(a)
    #y = d * np.sin(a)
    #fig, ax = plt.subplots()
    #ax.set_aspect("equal")
    #ax.plot(x, -y, "bo")
    #plt.tight_layout()
    #plt.savefig("image.png", dpi=300)
    #plt.show()

    ##################  LOI DE DIRECTION  ####################
    angle_cible = 0
    angle_cible = np.rad2deg(a[math.floor(np.argmax(d))])
    
    # proportion avec le max steering angle de la voiture
    #p = 0.5/0.35
    p = 1
    
    if abs(angle_cible) < 8:  
        angle_consigne = 0
    elif abs(angle_cible) < 16:
        # angle_consigne = min(3, max(-3, angle_cible))*3.14/180
        angle_consigne = min(5/p, max(-5/p, angle_cible))*3.14/180
    elif abs(angle_cible) < 25:
        # angle_consigne = min(7, max(-7, angle_cible))*3.14/180
        angle_consigne = min(11/p, max(-11/p, angle_cible))*3.14/180
    elif abs(angle_cible) < 33:
        # angle_consigne = min(13, max(-13, angle_cible))*3.14/180
        angle_consigne = min(15/p, max(-15/p, angle_cible))*3.14/180
    elif abs(angle_cible) < 41:
        # angle_consigne = min(19, max(-19, angle_cible))*3.14/180
        angle_consigne = min(22/p, max(-22/p, angle_cible))*3.14/180
    else : 
        # angle_consigne = min(25, max(-25, angle_cible))*3.14/180
        angle_consigne = min(28/p, max(-28/p, angle_cible))*3.14/180

    angle_delta = delta_angle(0,donnees_lidar)*3.14/180
    angle_consigne += angle_delta
    
    ###################  LOI DE VITESSE  #####################
    speed = vmin + (vmax - vmin)*math.exp(-abs(angle_consigne*180/3.14)/10)
    #print(speed);
    ################## FIN CONSIGNE VITESSE ##################
    if speed > vmax:
        speed = vmax
    elif speed < -1 * vmax:
        speed = -1 * vmax
        
    distance_en_face = d[len(d) // 2]
        
    if(distance_en_face < 1.25 and distance_en_face > 0.75):
        aux = (points * 20) // 360
        average_left = np.mean(d[(len(d)//2 - aux):(len(d)//2)])
        average_right = np.mean(d[(len(d)//2):(len(d)//2 + aux)])
        # print(d[(len(d)//2 - aux):(len(d)//2 + aux)])
        #print(average_left, average_right)
        if(average_left / average_right > 1):
            print("WARNING LEFT")
            angle_consigne -= 15 * 3.14 / 180
#        elif(average_left / average_right < 1):
        else:
            print("WARNING RIGHT")
            angle_consigne += 15 * 3.14 / 180
#        if(average_left / average_right > 1.5 or average_left / average_right < 0.5):

        #speed *= 0.75
        driver.setCruisingSpeed(speed)
        driver.setSteeringAngle(angle_consigne) 
        print(f"vitesse : { speed}")
        print(f"distance_en_face : { distance_en_face }")
        print(f"angle_cible : { angle_cible }")
        print(f"angle_consigne : { angle_consigne * 180 / 3.14 }")
        print(f"Gyro reading : { donnees_gyro[2] }")
        print(f"Absolute angle : { absolute_angle }")
        print("="*50)        
    elif(distance_en_face < 0.75):
        speed = -0.5 * vmax 
        angle_consigne = 0
        driver.setCruisingSpeed(speed)
        driver.setSteeringAngle(angle_consigne)
        print("BACKING UP")
        print(f"vitesse : { speed}")
        print(f"distance_en_face : { distance_en_face }")
        print(f"angle_cible : { angle_cible }")
        print(f"angle_consigne : { angle_consigne * 180 / 3.14 }")
        print(f"Gyro reading : { donnees_gyro[2] }")
        print(f"Absolute angle : { absolute_angle }")
        print("="*50)
    else:
        print("NORMAL OPERATION")
        print(f"vitesse : { speed}")
        print(f"distance_en_face : { distance_en_face }")
        print(f"angle_cible : { angle_cible }")
        print(f"angle_consigne : { angle_consigne * 180 / 3.14 }")
        print(f"Gyro reading : { donnees_gyro[2] }")
        print(f"Absolute angle : { absolute_angle }")
        print("="*50)
        driver.setCruisingSpeed(speed)
        driver.setSteeringAngle(angle_consigne)