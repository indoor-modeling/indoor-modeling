"""
将c++程序生成的floor文件格式转换，
    floor文件格式：
    ================================
    #begin
        p1
        p2
        p3
    #end
    #begin
        p1
        p2
    #end
    ...
    ================================

    新格式：
    ================================
    p1 p2 p3...pn
    p1 p2 p3...pn
    ...
    ================================
"""
import numpy as np


def getRoomBound(in_floor_path, output_path):
    floors = []
    with open(in_floor_path, 'r', encoding='utf-8') as f:
        data = f.readlines()
        data = np.array([x.strip('\n') for x in data])
        begin_idx = np.argwhere(data == '#begin').reshape(-1)
        end_idx = np.argwhere(data == '#end').reshape(-1)
        for i in range(begin_idx.size):
            if begin_idx[i] + 1 != end_idx[i]:
                wall = []
                for j in range(begin_idx[i] + 1, end_idx[i]):
                    wall += data[j].split(' ')
                wall = " ".join(wall)
                # wall = [float(i) for i in wall]
                floors.append(wall)

    with open(output_path, 'w', encoding='utf-8') as f:
        for one_room in floors:
            f.writelines(one_room)
            f.write('\n')


if __name__ == '__main__':
    # in_path = './data/exp1/floor.txt'
    # out_path = './data/exp1/area5full.txt'

    # in_path = './data/area4/floor.txt'
    # out_path = './data/area4/area4.txt'

    in_path = './data/area3/floor.txt'
    out_path = './data/area3/area3.txt'
    getRoomBound(in_path, out_path)
