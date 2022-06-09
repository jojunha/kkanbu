#!/usr/bin/env python3

import rospy
import folium
import webbrowser
import base64
import time
from ublox_msgs.msg import NavPVT
from kkanbu_sensor.msg import object_info

class crack_Map:
    def __init__(self, center, zoom_start):
        self.my_map = folium.Map(location = center, zoom_start = zoom_start)

    def mark(self, latitude, longitude, detection_name, file_name):
        if(detection_name == 'car'):
            folium.CircleMarker([latitude, longitude],
                    radius = 1,
                    color = 'darkblue',
                    popup='car').add_to(self.my_map)
            return
        
        time.sleep(2)
        pic = base64.b64encode(open(file_name,'rb').read()).decode()
        image_tag = '<img src="data:image/jpeg;base64,{}">'.format(pic)
        iframe = folium.IFrame(image_tag, width=500, height=400)
        popup = folium.Popup(iframe, max_width=500)

        folium.Marker([latitude, longitude],
                        popup = popup,
                        icon = folium.Icon(color='black', icon='star'),
                        tooltip='crack').add_to(self.my_map)
    
    def showMap(self):
        #Display the map
        self.my_map.save("crack_map.html")
        webbrowser.open("crack_map.html")


class yolo_Map:
    def __init__(self, center, zoom_start):
        self.my_map = folium.Map(location = center, zoom_start = zoom_start)

    def mark(self, latitude, longitude, detection_name, file_name):
        if(detection_name == 'car'):
            folium.CircleMarker([latitude, longitude],
                    radius = 1,
                    color = 'darkblue',
                    popup='car').add_to(self.my_map)
            return

        time.sleep(2)
        pic = base64.b64encode(open(file_name,'rb').read()).decode()
        image_tag = '<img src="data:image/jpeg;base64,{}">'.format(pic)
        iframe = folium.IFrame(image_tag, width=500, height=400)
        popup = folium.Popup(iframe, max_width=500)

        if(detection_name == 'kickboard'):
            mark_color = 'red'
        elif(detection_name == 'obstacles'):
            mark_color = 'blue'
        elif(detection_name == 'porthole'):
            mark_color = 'green'
        else:
            print("Error !! ")
            return 

        folium.Marker([latitude, longitude],
                        popup = popup,
                        icon = folium.Icon(color=mark_color, icon='star'),
                        tooltip = detection_name).add_to(self.my_map)
    
    def showMap(self):
        #Display the map
        self.my_map.save("yolo_map.html")
        webbrowser.open("yolo_map.html")


latitude = 0
longitude = 0
base = [37.5416,  127.0792]  # konkuk univ.

############## mode ##############
############## mode ##############
############## mode ##############
mode = 'yolo'

if(mode=='crack'): # crack mode
    map = crack_Map(center = base, zoom_start = 18)
    map.showMap()
else: # yolo mode
    map = yolo_Map(center = base, zoom_start = 18)
    map.showMap()


def GnssCB(msg):
    global latitude
    global longitude

    latitude = msg.lat/10000000.0
    longitude = msg.lon/10000000.0

    # rospy.loginfo(latitude)
    # rospy.loginfo(longitude)


def CrackCB(msg):

    print(msg.file_name)
    map.mark(msg.latitude, msg.longitude, msg.class_name, msg.file_name)
    map.showMap()


def YoloCB(msg):
    global latitude
    global longitude
    
    if(latitude == 0):
        return
    else:
        map.mark(latitude, longitude, msg.class_name, msg.file_name)
        map.showMap()

    return 


# latitude = 37.5409
# longitude = 127.0763
# base = [latitude,  longitude]  # konkuk univ.
# map = yolo_Map(center = base, zoom_start = 17)
# map.showMap()

# class_name = ['kickboard', 'obstacle', 'porthole']
# file_name = ['kickboard.png', 'obstacle.png', 'porthole.png']
# for i in range(40):
#     time.sleep(1)
#     latitude += 0.001
#     longitude += 0.001
    # map.mark(latitude, longitude, class_name[i%3], file_name[i%3])
    # map.showMap()


if __name__ == "__main__":
    rospy.init_node('visualization')

    gnss_subscriber = rospy.Subscriber('/ublox_msgs/navpvt', NavPVT, GnssCB)
    seg_subscriber = rospy.Subscriber('/seg_info', object_info, CrackCB)
    yolo_subscriber = rospy.Subscriber('/yolo_info', object_info, YoloCB)


    rospy.loginfo("Ready visualization!!")
    rate = rospy.Rate(0.4)

    while not rospy.is_shutdown():

        if(latitude != 0):
            map.mark(latitude, longitude, 'car', 'car.png')
            map.showMap()
            
        rate.sleep()

    

