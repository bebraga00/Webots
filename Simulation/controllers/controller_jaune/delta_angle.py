def delta_angle(angle_consigne, donnees_lidar):
    #test avec cone de 14°
    angle_inf = -7
    angle_sup = 7
    
    for k in range(angle_inf,0):
        if donnees_lidar[49 + angle_consigne + k] < 800/1000: #test à 80cm
            angle_inf = k
    
    for k in range(1,angle_sup):
        if donnees_lidar[49 + angle_consigne + k] < 800/1000:
            angle_sup = k
            break;
            
    delta_angle = angle_sup + angle_inf
    return 2*delta_angle