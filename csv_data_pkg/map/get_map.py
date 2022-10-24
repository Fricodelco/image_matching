#!/usr/bin/python3

import urllib.request
from PIL import Image, ImageOps, ImageDraw
import csv
import math
import argparse
from pathlib import Path

class COLOR():
    __colors = [
        (255,0,0),
        (0,255,0),
        (0,0,255),
        (255,255,0),
        (255,0,255),
        (255,255,0)
    ]
    @staticmethod
    def get_color(index):
        return COLOR.__colors[index]

class MapGenerator():
    def __init__(self) -> None:
        self.map_width = 600
        self.map_height = 450
        self.map_height_cup = 50
        
        self.lat_step = 0.001606
        lon_step = 0.0007
    
        self.lon_step = lon_step*(1-self.map_height_cup/self.map_height)

        self.first_csv_file_path = "/home/parallels/copa5/video/200_csi_19.csv"
        self.second_csv_file_path = "/home/parallels/copa5/video/200_csi_19_bad.csv"

    def generate_map(self, center:list, lat_lon_datas:list, max_min: list)->Image:
        
        lat_image_count = math.ceil((max(max_min[0][0])-min(max_min[0][1]))/self.lat_step)+1
        lon_image_count = math.ceil((max(max_min[1][0])-min(max_min[1][1]))/self.lon_step)+1
        
        lat_start = center[0] - (lat_image_count/2+0.5)*self.lat_step
        lon_start = center[1] + (lon_image_count/2+0.5)*self.lon_step
        
        map_img = Image.new('RGB', (self.map_width*lat_image_count, (self.map_height-self.map_height_cup)*lon_image_count))
        
        # points_str = self.convert_lat_lon_to_srt(lat_lon_data)
        
        for i in range(lat_image_count):
            for j in range(lon_image_count):
                url = 'https://static-maps.yandex.ru/1.x/?ll={0},{1}&l=sat&z=19&size={2},{3}'.format(
                    lat_start+self.lat_step*(i+1),
                    lon_start-self.lon_step*(j+1),
                    self.map_width,
                    self.map_height,
                    # points_str,
                    # str(lat_center)+','+str(lon_center)
                )
                res = urllib.request.urlopen(url)
                img = ImageOps.crop(Image.open(res), (0,0,0,self.map_height_cup))
                map_img.paste(img,(self.map_width*i, (self.map_height-self.map_height_cup)*j))
        
        px_path_points = self.convert_lat_lon_to_px(lat_image_count, lon_image_count, center, lat_lon_datas)
        self.draw_path_on_map(map_img, px_path_points)

        return map_img
    
    def convert_lat_lon_to_srt(self, lat_lon_data):
        result_srt = ''
        for i in range(len(lat_lon_data[0])):
            result_srt+='{0},{1},'.format(lat_lon_data[0][i], lat_lon_data[1][i])
        return result_srt[:-1]

    def convert_lat_lon_to_px(self, lat_image_count: int, lon_image_count: int, center: list, lat_lon_datas) -> list:
        
        lat_range_d = lat_image_count*self.lat_step
        lon_range_d = lon_image_count*self.lon_step
        
        width_im = self.map_width*lat_image_count
        heigth_im = (self.map_height - self.map_height_cup)*lon_image_count
        
        lat_px_d = width_im/lat_range_d
        lon_px_d = heigth_im/lon_range_d
        
        left_d = center[0] - lat_range_d/2
        top_d = center[1] - lon_range_d/2
        points = []
        for j, lat_lon_data in enumerate(lat_lon_datas):
            points.append([])
            for i in range(len(lat_lon_data[0])):
                points[j].append((
                        (lat_lon_data[0][i]-left_d)*lat_px_d,
                        heigth_im-(lat_lon_data[1][i]-top_d)*lon_px_d+self.map_height_cup/2
                    )
                )

        return points

    def draw_path_on_map(self, img, px_path_points):
        draw = ImageDraw.Draw(img)
        for i, px_path in enumerate(px_path_points):
            draw.line(px_path, fill=COLOR.get_color(i), width=4)


def read_csv(file_name):
    lat_lon_data = [[],[]]
    with open(file_name, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')
        if(len(spamreader.__next__())==1):
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
            spamreader.__next__()
        for i,line in enumerate(spamreader):
            lat_lon_data[0].append(float(line[2]))
            lat_lon_data[1].append(float(line[1]))
    
    return lat_lon_data

def get_geo_data(file_names: list):
    lat_lon_datas = []
    for file_name in file_names:
        lat_lon_datas.append(read_csv(file_name))
    
    max_min_lats = [[],[]]
    max_min_lons = [[],[]]
    
    for data in lat_lon_datas:
        max_min_lats[0].append(max(data[0]))
        max_min_lats[1].append(min(data[0]))
    
        max_min_lons[0].append(max(data[1]))
        max_min_lons[1].append(min(data[1]))

    lat_center = (max(max_min_lats[0])+min(max_min_lats[1]))/2
    lon_center = (max(max_min_lons[0])+min(max_min_lons[1]))/2
    
    return [lat_center, lon_center], lat_lon_datas,[max_min_lats, max_min_lons]
    
def parse_args()->list:
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--logs", help="list of logs names", nargs='+')
    
    args = parser.parse_args()

    return [] if args.logs is None else args.logs 

def main():
    mapGenerator = MapGenerator()
    file_names = parse_args()
    if len(file_names)==0:
        if(Path(mapGenerator.first_csv_file_path).is_file()):
            file_names.append(mapGenerator.first_csv_file_path)
        else:
            print("Cant find file with path '{0}'".format(mapGenerator.first_csv_file_path))
        if(Path(mapGenerator.second_csv_file_path).is_file()):
            file_names.append(mapGenerator.second_csv_file_path)
        else:
            print("Cant find file with path '{0}'".format(mapGenerator.second_csv_file_path))
        if len(file_names) !=2:
            return
    
    center, lat_lon_datas, max_min = get_geo_data(file_names)

    try:
        map_img = mapGenerator.generate_map(center, lat_lon_datas, max_min)
    except IOError:
        print("Could not generate the image - try adjusting the zoom level and checking your coordinates")
    else:
        map_img.save('map.png')
        print("The map has successfully been created")


if __name__ == '__main__':  main()