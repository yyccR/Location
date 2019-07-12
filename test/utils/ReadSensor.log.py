#!/usr/bin/python
# -*- coding:utf-8 -*-

import csv

origin_sensors_data = []
line = {}
outputs = []


def writeCsv(origin_sensors_data, outputs):
    sensor_file_name = "origin_sensors_data_" + str(origin_sensors_data[0]['gps3d'][6]) + ".csv"
    outputs_file_name = "Location_ouputs_" + str(origin_sensors_data[0]['gps3d'][6]) + ".csv"
    sensor_file = "D:\\worksheet\\clion\\Location\\test\\data\\sensor_log\\" + sensor_file_name
    outputs_file = "D:\\worksheet\\clion\\Location\\test\\data\\sensor_log\\" + outputs_file_name
    with open(sensor_file, "w", newline="") as sensor_csv_file:
        write = csv.writer(sensor_csv_file)
        all=[]
        for i in range(len(origin_sensors_data)):
            if origin_sensors_data[i] != {} :
                data = []
                data.extend(origin_sensors_data[i]["acc3d"])
                data.extend(origin_sensors_data[i]["g3d"])
                data.extend(origin_sensors_data[i]["gyro3d"])
                data.extend(origin_sensors_data[i]["mag3d"])
                ornt = origin_sensors_data[i]["ori3d"]
                new_ornt = [ornt[2], ornt[0], ornt[1]]
                data.extend(new_ornt)
                # gps(lng,lat,alt,accuracy,speed,bearing,t)
                gps = origin_sensors_data[i]["gps3d"]
                new_gps = [gps[1], gps[0], gps[2], 0.0, gps[4], gps[3], gps[5], 0.0, 0.0, 0.0, gps[6]]
                data.extend(new_gps)
                data.extend(origin_sensors_data[i]["way2d"])
                all.append(data)
        print("Done! see files: \n" + str(sensor_file) + " size: " + str(len(all))
              + "\n and \n" + str(outputs_file))
        write.writerows(all)
    with open(outputs_file, "w", newline="") as outputs_csv_file:
        write = csv.writer(outputs_csv_file)
        write.writerows(outputs)


with open("C:\\Users\\yangcheng\\Desktop\\Log\\sensor_1558684326000.log") as f:
    data = f.readlines()
    for i in range(len(data)):
        if data[i].find("Go") != -1 or i == len(data) - 1:
            if origin_sensors_data != [] and outputs != []:
                writeCsv(origin_sensors_data, outputs)
                origin_sensors_data = []
                outputs = []
        else:
            if data[i].find("---") != -1 and data[i].find(">") == -1:
                origin_sensors_data.append(line)
                line = {}
            else:
                if data[i].find("Output") != -1:
                    output_line = list(map(lambda x: float(x), data[i].split("=")[1].split(",")))
                    outputs.append(output_line)
                else:
                    if data[i].strip() != "" and data[i].find("=") != -1:
                        key_values = data[i].split("=")
                        line[key_values[0].strip()] = list(map(lambda x: float(x), key_values[1].strip().split(",")))
