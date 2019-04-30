import smartDroneLib
import os
import read_json
FlYING_HEIGHT = 15

p1 = (31.7766,35.20201,750)
p2 = (31.77592,35.20078,750)



def main():
	"""
	path = "/home/pi/Documents/drone/"
	os.chdir(path)
	os.system("./detectTiny ./images/")
	
	object_list = read_json.text_parsing("/home/pi/Documents/drone/images/35.1968524,31.7777304,15.08,0.22301839292.txt")
	for object_value in object_list:
		if object_value[0] == "person":
			print("there is person!")
			return object_value[0]
return False
"""

smartDroneLib.scanArea(p1, p2)

#smartDroneLib.findObject(p1, p2, "person")

if __name__ == "__main__":
    main()
    
