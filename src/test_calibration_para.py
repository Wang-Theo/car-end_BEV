from nuscenes.nuscenes import NuScenes
import json

rootpath='/data/sets/nuscenes'

nusc = NuScenes(version='v1.0-mini', dataroot=rootpath, verbose=True)

my_scene = nusc.scene[0]

my_calibration = nusc.calibrated_sensor
my_sensor = nusc.sensor



print("\n"+"calibrated_sensor size: "+str(len(my_calibration)))

print("\n===================================================")

list = []
json_list = []
for i in range(12):
    print(i)
    print(my_calibration[i])
    for j in range(len(my_sensor)):
        if my_calibration[j]["sensor_token"] == my_sensor[i]["token"]:
            print(my_sensor[j]["channel"])
            list.append(str(my_sensor[j]["channel"]))
            list.append(str(my_calibration[i]["translation"]))
            list.append(str(my_calibration[i]["rotation"]))
            list.append(str(my_calibration[i]["camera_intrinsic"]))
            a = {
                "name":str(my_sensor[j]["channel"]),
                "translation":str(my_calibration[i]["translation"]),
                "rotation":str(my_calibration[i]["rotation"]),
                "camera_intrinsic":str(my_calibration[i]["camera_intrinsic"])
            }
            json_list.append(a)
            break

print("\n===================================================")
file = open("cali_param.txt",'w+',encoding='utf-8')
for i in range(len(list)):
    if i%4 == 0 and i != 0:
        print("\n")
        file.write("\n")
    print(list[i])
    file.write(str(list[i])+" ")

f_write = open('cali_param.json', 'w')
json.dump(json_list,f_write)