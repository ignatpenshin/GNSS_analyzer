import struct
import os
from typing import Counter 
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def distance_geoid(lat1: float, lon1: float, lat2: float, lon2: float) -> float: 
    # Degrees to radians
    lat1 = lat1 * np.pi / 180.0
    lon1 = lon1 * np.pi / 180.0
    lat2 = lat2 * np.pi / 180.0
    lon2 = lon2 * np.pi / 180.0

    r = 6378100

    rho1 = r * np.cos(lat1)
    z1 = r * np.sin(lat1)
    x1 = rho1 * np.cos(lon1)
    y1 = rho1 * np.sin(lon1)

    rho2 = r * np.cos(lat2)
    z2 = r * np.sin(lat2)
    x2 = rho2 * np.cos(lon2)
    y2 = rho2 * np.sin(lon2)

    dot = (x1 * x2 + y1 * y2 + z1 * z2)
    cos_theta = dot / (r * r)
    theta = np.arccos(cos_theta)

    return r * theta

def smooth(arr:np.ndarray):
    s = 0
    a = 0.1
    u = np.zeros_like(arr)  
    u[0] = arr[0]
    for i in range(1, arr.shape[0]):
        u[i] = u[i - 1] * (1 - a) + arr[i] * a
    return u
        

def read_GNSS(gnss_path):
    gnss_maxpoints = os.path.getsize(gnss_path) // 32
    with open(gnss_path, 'rb') as f:
        gnss_data = []
        i = 0
        while i < gnss_maxpoints:
            data = struct.unpack('=Qddff',f.read(32))
            if i > 0:
                timestamp_2 = int(data[0] * 1e-9)
                lat_2 = round(data[1], 9)
                lon_2 = round(data[2], 9)
                alt_2 = round(data[3], 2)
                length = distance_geoid(lat_1, lon_1, lat_2, lon_2)
                speed_2 = length/(timestamp_2-timestamp_1) * 3.6 
                # Продумать фильтр по скорости // допилить сюда IMU
            if i == 0:
                timestamp_1 = int(data[0] * 1e-9)
                lat_1 = round(data[1], 9)
                lon_1 = round(data[2], 9)
                alt_1 = round(data[3], 2)
                speed_1 = 0.0
            else: 
                timestamp_1 = timestamp_2
                lat_1 = lat_2
                lon_1 = lon_2
                alt_1 = alt_2
                speed_1 = speed_2
            
            gnss_data.append((timestamp_1, lat_1, lon_1, alt_1, speed_1)) 
            i += 1  
        return gnss_data

def visual_coords(gnss_path):

    data = read_GNSS(gnss_path)
    time, lat, long, altitude, speed = data[0]

    speed_l = []
    time_l = []

    ax1 = plt.subplot(121)
    ax1.set_title('Coords')
    ax1.scatter((np.array(data))[:, 2], (np.array(data))[:, 1], c=(np.array(data))[:, 3], s=0.1)
    loc = ax1.scatter([long], [lat], c="red", s=3)
    ax3 = plt.subplot(122)
    ax3.set_title('Velocity (Km/H)')
    ax3.plot(time_l, speed_l, c="red")

    def update(i, speed_l:list, time_l:list):
        
        #plt.xticks(rotation=45, ha='right')
        #plt.subplots_adjust(bottom=0.20)
        time, lat, long, altitude, speed = data[i]
        speed_l.append(speed)
        time_l.append(time)
        print(len(speed_l))
        speed_l = smooth(np.array(speed_l[-30:]))
        time_l = time_l[-30:]
     
        loc.set_offsets([long, lat])
        ax3.clear()
        ax3.set_ylim([0,50])
        ax3.plot(time_l, speed_l)
        return loc, ax3
    
    anim = FuncAnimation(plt.gcf(), update, fargs=(speed_l,time_l), interval=1, blit=False)
    plt.show()

gnss_path = "gnss.log"
visual_coords(gnss_path)

#rpc spms 
#commlib

#enabler