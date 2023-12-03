"""
读取txt文件并将其拉伸为三维ply文件
"""
import pyvista as pv
import numpy as np

# 设置绘图模式为文档模式，即白色底
pv.set_plot_theme("document")


# 将房间边界转换为mesh文件
def polygon23D(points, isExtrude, show_cap):
    remove_last_points = points[0:-3]
    x = remove_last_points[::3].reshape(-1, 1)
    y = remove_last_points[1::3].reshape(-1, 1)
    z = np.zeros((len(x), 1))

    height = remove_last_points[2]

    poly = np.hstack((x, y, z))
    poly_connectivity = [x.shape[0]] + [i for i in range(x.shape[0])]

    # Create a PolyData object from the points array and triangle
    polygon = pv.PolyData(poly, poly_connectivity).triangulate()

    if isExtrude is True:
        # Extrude the polygon along the z-axis to create a 3D model
        mesh = polygon.extrude([0, 0, height], capping=show_cap)
    else:
        mesh = polygon
    return mesh


def genModel(path_roomBound, path_ori_point_cloud=None, path_save=None, isExtrude=True, show_cap=True, show_edge=True):
    """
    :param path_roomBound: 房间边界文件txt
    :param path_ori_point_cloud: 房间原型的点云文件ply，如果有则生成两个画面，否则仅有mesh画面
    :param path_save: mesh文件存储路径
    :param show_cap: 是否为mesh生成上下底面
    :param show_edge: 是否显示线框模型
    :return:
    """
    # 读取文件
    with open(path_roomBound, 'r') as f:
        data = f.readlines()
        data = [(x.strip('\n').split(' ')) for x in data]
        new_data = []
        for i in data:
            ps = []
            for j in i:
                if j != '':
                    ps.append(float(j))
            new_data.append(np.array(ps))

    # mesh文件列表
    meshes = []
    # 颜色字典
    color_map = {
        'aliceblue': '#F0F8FF',
        'antiquewhite': '#FAEBD7',
        'aqua': '#00FFFF',
        'aquamarine': '#7FFFD4',
        'azure': '#F0FFFF',
        'beige': '#F5F5DC',
        'bisque': '#FFE4C4',
        'black': '#000000',
        'blanchedalmond': '#FFEBCD',
        'blue': '#0000FF',
        'blueviolet': '#8A2BE2',
        'brown': '#A52A2A',
        'burlywood': '#DEB887',
        'cadetblue': '#5F9EA0',
        'chartreuse': '#7FFF00',
        'chocolate': '#D2691E',
        'coral': '#FF7F50',
        'cornflowerblue': '#6495ED',
        'cornsilk': '#FFF8DC',
        'crimson': '#DC143C',
        'cyan': '#00FFFF',
        'darkblue': '#00008B',
        'darkcyan': '#008B8B',
        'darkgoldenrod': '#B8860B',
        'darkgray': '#A9A9A9',
        'darkgreen': '#006400',
        'darkkhaki': '#BDB76B',
        'darkmagenta': '#8B008B',
        'darkolivegreen': '#556B2F',
        'darkorange': '#FF8C00',
        'darkorchid': '#9932CC',
        'darkred': '#8B0000',
        'darksalmon': '#E9967A',
        'darkseagreen': '#8FBC8F',
        'darkslateblue': '#483D8B',
        'darkslategray': '#2F4F4F',
        'darkturquoise': '#00CED1',
        'darkviolet': '#9400D3',
        'deeppink': '#FF1493',
        'deepskyblue': '#00BFFF',
        'dimgray': '#696969',
        'dodgerblue': '#1E90FF',
        'firebrick': '#B22222',
        'floralwhite': '#FFFAF0',
        'forestgreen': '#228B22',
        'fuchsia': '#FF00FF',
        'gainsboro': '#DCDCDC',
        'ghostwhite': '#F8F8FF',
        'gold': '#FFD700',
        'goldenrod': '#DAA520',
        'gray': '#808080',
        'green': '#008000',
        'greenyellow': '#ADFF2F',
        'honeydew': '#F0FFF0',
        'hotpink': '#FF69B4',
        'indianred': '#CD5C5C',
        'indigo': '#4B0082',
        'ivory': '#FFFFF0',
        'khaki': '#F0E68C',
        'lavender': '#E6E6FA',
        'lavenderblush': '#FFF0F5',
        'lawngreen': '#7CFC00',
        'lemonchiffon': '#FFFACD',
        'lightblue': '#ADD8E6',
        'lightcoral': '#F08080',
        'lightcyan': '#E0FFFF',
        'lightgoldenrodyellow': '#FAFAD2',
        'lightgreen': '#90EE90',
        'lightgray': '#D3D3D3',
        'lightpink': '#FFB6C1',
        'lightsalmon': '#FFA07A',
        'lightseagreen': '#20B2AA',
        'lightskyblue': '#87CEFA',
        'lightslategray': '#778899',
        'lightsteelblue': '#B0C4DE',
        'lightyellow': '#FFFFE0',
        'lime': '#00FF00',
        'limegreen': '#32CD32',
        'linen': '#FAF0E6',
        'magenta': '#FF00FF',
        'maroon': '#800000',
        'mediumaquamarine': '#66CDAA',
        'mediumblue': '#0000CD',
        'mediumorchid': '#BA55D3',
        'mediumpurple': '#9370DB',
        'mediumseagreen': '#3CB371',
        'mediumslateblue': '#7B68EE',
        'mediumspringgreen': '#00FA9A',
        'mediumturquoise': '#48D1CC',
        'mediumvioletred': '#C71585',
        'midnightblue': '#191970',
        'mintcream': '#F5FFFA',
        'mistyrose': '#FFE4E1',
        'moccasin': '#FFE4B5',
        'navajowhite': '#FFDEAD',
        'navy': '#000080',
        'oldlace': '#FDF5E6',
        'olive': '#808000',
        'olivedrab': '#6B8E23',
        'orange': '#FFA500',
        'orangered': '#FF4500',
        'orchid': '#DA70D6',
        'palegoldenrod': '#EEE8AA',
        'palegreen': '#98FB98',
        'paleturquoise': '#AFEEEE',
        'palevioletred': '#DB7093',
        'papayawhip': '#FFEFD5',
        'peachpuff': '#FFDAB9',
        'peru': '#CD853F',
        'pink': '#FFC0CB',
        'plum': '#DDA0DD',
        'powderblue': '#B0E0E6',
        'purple': '#800080',
        'red': '#FF0000',
        'rosybrown': '#BC8F8F',
        'royalblue': '#4169E1',
        'saddlebrown': '#8B4513',
        'salmon': '#FA8072',
        'sandybrown': '#FAA460',
        'seagreen': '#2E8B57',
        'seashell': '#FFF5EE',
        'sienna': '#A0522D',
        'silver': '#C0C0C0',
        'skyblue': '#87CEEB',
        'slateblue': '#6A5ACD',
        'slategray': '#708090',
        'snow': '#FFFAFA',
        'springgreen': '#00FF7F',
        'steelblue': '#4682B4',
        'tan': '#D2B48C',
        'teal': '#008080',
        'thistle': '#D8BFD8',
        'tomato': '#FF6347',
        'turquoise': '#40E0D0',
        'violet': '#EE82EE',
        'wheat': '#F5DEB3',
        'white': '#FFFFFF',
        'whitesmoke': '#F5F5F5',
        'yellow': '#FFFF00',
        'yellowgreen': '#9ACD32'}

    if path_ori_point_cloud is None:
        # 创建画布
        p = pv.Plotter()
        p.renderer.set_color_cycler(list(color_map.values()))
        for i, room in enumerate(new_data):
            mesh = polygon23D(room, isExtrude, show_cap)
            if path_save is not None:
                mesh.save(path_save + str(i) + '_.ply')
            meshes.append(mesh)
            p.add_mesh(mesh, show_edges=show_edge)
        p.show()
    else:
        p = pv.Plotter(shape=(1, 2))
        area_6 = pv.read(path_ori_point_cloud)

        p.subplot(0, 0)
        p.add_points(area_6, scalars=area_6['RGB'], rgba=True)

        p.subplot(0, 1)
        p.renderer.set_color_cycler(list(color_map.values()))
        for i, room in enumerate(new_data):
            mesh = polygon23D(room, isExtrude, show_cap)
            if path_save is not None:
                mesh.save(path_save + str(i) + '_.ply')
            meshes.append(mesh)
            p.add_mesh(mesh, show_edges=show_edge)
        p.show()


if __name__ == '__main__':
    # roomBound = './data/exp1/area5full_with_height.txt'
    # path_point_cloud = './data/exp1/area5_full.ply'
    # path_save_mesh = './data/exp1/res/'

    # roomBound = './data/area4/area4full_with_height.txt'
    # path_point_cloud = './data/area4/Area_4_sub.ply'
    # path_save_mesh = './data/area4/res/'

    # roomBound = './data/area6/Area_6_with_height.txt'

    roomBound = './data/area3/area3full_with_height.txt'
    path_save_mesh = './data/area3/res/'

    genModel(roomBound, isExtrude=True, show_cap=True, path_save=path_save_mesh)
