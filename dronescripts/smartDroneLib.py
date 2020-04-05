import scanAreaModule
import findObjectModule
from dronekit import LocationGlobalRelative
import os
import smtplib


TYPE = 0
LOCATION = 1
SIZE = 2
COLOR = 3
MAPPING = 4
LEFT = 0
TOP = 1
RIGHT = 2
BOTTOM = 3
FlYING_HEIGHT = 15
DIRECTORY= "/home/pi/Documents/drone/"
IMG_DIRECTORY= "/home/pi/Documents/drone/images"

def _sendMail(msg, to='arye.rules@gmail.com'):
	fromaddr = 'nitsan57@gmail.com'
	toaddrs  = to
	username = 'nitsan57'
	password = 'XXXXX
	server = smtplib.SMTP('smtp.gmail.com:587')
	server.ehlo()
	server.starttls()
	server.login(username,password)
	server.sendmail(fromaddr, toaddrs, msg)
	server.quit()
	


def scanArea(p1, p2):
    if (not os.path.isdir(DIRECTORY)):
	os.mkdir(DIRECTORY)
    if (not os.path.isdir(IMG_DIRECTORY)):
	os.mkdir(IMG_DIRECTORY)

    x, y,z = p1
    x2, y2, z2 = p2
    po1 = LocationGlobalRelative(x, y, z)
    po2 = LocationGlobalRelative(x2, y2, z2)
    po3 = LocationGlobalRelative(x, y2, (z+z2)/2)
    po4 = LocationGlobalRelative(x2 ,y, (z+z2)/2)
    scanAreaModule.scanAreaFunction(po1, po2, po3, po4)

def findObject(p1, p2, object_name, to='arye.rules@gmail.com'):
    if (not os.path.isdir(DIRECTORY)):
	os.mkdir(DIRECTORY)
    if (not os.path.isdir(IMG_DIRECTORY)):
	os.mkdir(IMG_DIRECTORY)
    x, y,z = p1
    x2, y2, z2 = p2
    po1 = LocationGlobalRelative(x, y, z)
    po2 = LocationGlobalRelative(x2, y2, z2)
    po3 = LocationGlobalRelative(x, y2, (z+z2)/2)
    po4 = LocationGlobalRelative(x2 ,y, (z+z2)/2)
    found_object = findObjectModule.findObject(po1, po2, po3, po4, object_name)
    if found_object != False:
        print("found object "+ object_name + " at " + str(found_object[LOCATION]))
        _sendMail("found object "+ object_name + " at " + str(found_object[LOCATION]), to)
    else:
        print("no object on scanned area")
        
