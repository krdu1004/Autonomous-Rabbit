
import pyzed.sl as sl
from gpsd_reader import GPSDReader
from math import radians, cos, sin, asin, sqrt
import timer 

def DistanceFromCoordinates(lat1, lat2, lon1, lon2):
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
      
    dlon = lon2 - lon1 
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return(c * r * 1609.344)

def gpsProcess(gps_reader, fist_read_gnss, total_pos, lat_prev, lon_prev):
    status, lat_gnss, lon_gnss, speed_gnss = gps_reader.grab() # Get information from GPS 
    if status == sl.ERROR_CODE.SUCCESS:
        print("Received GNSS data: Latitude:", lat_gnss, ", Longitude:", lon_gnss, ", Speed:", speed_gnss)
        if fist_read_gnss: # Initilize first position
            total_pos = 0
            lon_prev = lon_gnss
            lat_prev = lat_gnss
            fist_read_gnss = False
        else:   
            # If speed of the APM is larger than 0.07 m/s calculate distance moved. 
            # Prevents the distance moved to be updated if the car is standing still, The GPS is has an 0.05 m/s accuracy. 
            if speed_gnss >= 0.07: 
                 #Start timer if not yet started
                if timer.start_time is None:
                    timer.start_timer()
                # If one second has passed update position
                elif timer.check_timer(1):
                    total_pos += DistanceFromCoordinates(lat_gnss, lat_prev, lon_gnss, lon_prev) # Update position
                    timer.start_time = None
            # Update previos position     
            lat_prev = lat_gnss
            lon_prev = lon_gnss
    else:
        print("Failed to grab GNSS data.") # Returns None for all parameters
    return fist_read_gnss, total_pos, lat_prev, lon_prev, speed_gnss
    