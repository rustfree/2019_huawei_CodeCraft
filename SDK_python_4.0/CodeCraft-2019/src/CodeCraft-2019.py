# coding=utf-8
import logging
import sys
import re
import time 
import mygraph 
import mymap
import mydataprocess

def filter_word(str):
    return str[0] != '#'

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')


def main():
    if len(sys.argv) != 6:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    preset_answer_path = sys.argv[4]
    answer_path = sys.argv[5]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("preset_answer_path is %s" % (preset_answer_path))
    logging.info("answer_path is %s" % (answer_path))
#length of vector
    len_vec_road = 7
    len_vec_car = 7
    len_vec_cross = 5

# to read input file
    # get info_car, info_road, info_cross

    start1 = time.time()
    with open(car_path, 'r') as f1:
        list_car = f1.readlines()
        list_car = filter(filter_word,list_car)
        list_car = list(list_car)
        p1 = re.compile(r'[(](.*?)[)]', re.S) #least match
        info_car = []
        for item in list_car:
            str_info =  re.findall(p1, item) 
            str_pro = str_info[0].replace(' ', '')         
            str_pro = str_pro.split(',')
            for i in range(len_vec_car):
                str_pro[i] = int(str_pro[i])
            info_car.append(str_pro)
        f1.close()

    with open(road_path, 'r') as f1:
        list_road = f1.readlines()
        list_road = filter(filter_word,list_road)
        list_road = list(list_road)
        p1 = re.compile(r'[(](.*?)[)]', re.S) #least match
        info_road = []
        for item in list_road:
            str_info =  re.findall(p1, item) 
            str_pro = str_info[0].replace(' ', '')         
            str_pro = str_pro.split(',')
            for i in range(len_vec_road):
                str_pro[i] = int(str_pro[i])
            info_road.append(str_pro)
        f1.close()

    with open(cross_path, 'r') as f1:
        list_cross = f1.readlines()
        list_cross = filter(filter_word,list_cross)
        list_cross = list(list_cross)
        p1 = re.compile(r'[(](.*?)[)]', re.S) #least match
        info_cross = []
        for item in list_cross:
            str_info =  re.findall(p1, item) 
            str_pro = str_info[0].replace(' ', '')         
            str_pro = str_pro.split(',')
            for i in range(len_vec_cross):
                str_pro[i] = int(str_pro[i])
            info_cross.append(str_pro)
        f1.close()
    
    #分为预置和非预置
    info_car_preset = []
    info_car_normal = []
    for car_item in info_car:
        if car_item[6] == 1:
            info_car_preset.append(car_item)
        else:
            info_car_normal.append(car_item)


    road_dict = mymap.read_file(road_path)
    cross_dict =mymap.read_file(cross_path)
    

    new_info_car,new_info_cross,new_info_road = mydataprocess.mapprocess(info_car_normal,info_cross,info_road,road_dict,cross_dict)#有序映射

    result = mydataprocess.get_car_path(new_info_car,new_info_road,new_info_cross)
    #print('result',result)
    
       
# to write output file
    
    with open(answer_path, 'w') as f1:
        for item in result:
            f1.write(str(item)+'\n')
        f1.close()
    
    end1 = time.time()
    print('run time:',end1-start1)#运行时间
    
if __name__ == "__main__":
    main()

