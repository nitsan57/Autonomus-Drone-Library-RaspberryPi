from pprint import pprint
# import webcolors
# import cv2
TYPE = 0
LOCATION = 1
SIZE = 2
COLOR = 3
MAPPING = 4
LEFT = 0
TOP = 1
RIGHT = 2
BOTTOM = 3
# def closest_colour(requested_colour):
#     min_colours = {}
#     for key, name in webcolors.css3_hex_to_names.items():
#         r_c, g_c, b_c = webcolors.hex_to_rgb(key)
#         rd = (r_c - requested_colour[0]) ** 2
#         gd = (g_c - requested_colour[1]) ** 2
#         bd = (b_c - requested_colour[2]) ** 2
#         min_colours[(rd + gd + bd)] = name
#     return min_colours[min(min_colours.keys())]
#
# def get_colour_name(requested_colour):
#     try:
#         closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
#     except ValueError:
#         closest_name = closest_colour(requested_colour)
#         actual_name = None
#     return actual_name, closest_name

def text_parsing(name):
    with open(name, "r") as ins:
        car_list = []
        for line in ins:
            txt_parser = line.split(",")
            object_dict = dict()
            object_dict[TYPE] = txt_parser[0]
            loc_parser = txt_parser[1].split("_")
            object_dict[LOCATION] = dict()
            object_dict[LOCATION]["lon"] = loc_parser[0]
            object_dict[LOCATION]["lat"] = loc_parser[1]
            object_dict[SIZE] = dict()
            loc_parser = txt_parser[2].split("_")
            object_dict[SIZE]["height"] = float(loc_parser[0])
            object_dict[SIZE]["width"] = float(loc_parser[1])
            color_parser = txt_parser[3].split("_")
            rgb = [float(color_parser[0][1:]),float(color_parser[1]),float(color_parser[2][:-1])]
            # a,b = get_colour_name(rgb)
            # object_dict[COLOR] = b
            mapping_parser = txt_parser[MAPPING].split("_")
            object_dict[MAPPING] = mapping_parser
            car_list.append(object_dict)
    return car_list

# def draw_objects(img,object_list):
#     for object in object_list:
#         left,top,right,bottom = object[MAPPING]
#         cv2.rectangle(img,(float(top),float(left)),(float(bottom),float(right)),(0,255,0),3)
#



#img_path = "52608580_380819525799548_1907478680771231744_n.jpg"
# txt_path = "31.222223333.3333150.24.txt"

#object_list = text_parsing(txt_path )
#print(object_list)
#img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

