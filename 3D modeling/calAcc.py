"""
计算 室内外cell分割的精度: TP/FP/FN
"""

from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
import numpy as np


def IoU(matrix):
    Intersection = np.diag(matrix)
    Union = np.sum(matrix, axis=1) + np.sum(matrix, axis=0) - Intersection
    iou = Intersection / Union
    return iou

# 计算cell单元的标签:TP/FP/FN
def calLabel(truth, preds):
    """
    :param truth:
    :param preds:
    :return:
    获取每个被划分为室内的TP：1，FP：2，FN：3
    室外label=0
    """
    t = truth.reshape(-1)
    p = preds.reshape(-1)
    idx_TP = np.argwhere((t == 1) & (p == 1))
    idx_FP = np.argwhere((t == 0) & (p == 1))
    idx_FN = np.argwhere((t == 1) & (p == 0))

    print(idx_TP.shape[0] / (idx_TP.shape[0] + idx_FP.shape[0]))
    print(idx_TP.shape[0] / (idx_TP.shape[0] + idx_FN.shape[0]))
    label = np.zeros_like(t)
    label[idx_TP] = 1
    label[idx_FP] = 2
    label[idx_FN] = 3

    return label


# 计算室内外面积比
def calAreaPer(in_area, in_gt):
    idx_area = np.concatenate((in_gt, in_area), axis=1)
    idx_indoor = np.argwhere(idx_area[:, 0] == 1)
    sum_indoor_area = np.sum(in_area[idx_indoor])
    print(sum_indoor_area / (np.sum(in_area) - sum_indoor_area))


if __name__ == '__main__':
    gt = np.loadtxt('./data/area3/calAcc/gt.txt').reshape(-1, 1)
    pred = np.loadtxt('./data/area3/calAcc/pred.txt').reshape(-1, 1)
    area = np.loadtxt('./data/area3/calAcc/area.txt').reshape(-1, 1)

    # gt = np.loadtxt('./data/area6/calAcc/gt.txt').reshape(-1, 1)
    # pred = np.loadtxt('./data/area6/calAcc/pred.txt').reshape(-1, 1)
    # area = np.loadtxt('./data/area6/calAcc/area.txt').reshape(-1, 1)


    # gt = np.loadtxt('./data/exp1/calAcc/gt.txt').reshape(-1, 1)
    # pred = np.loadtxt('./data/exp1/calAcc/pred.txt').reshape(-1, 1)
    # area = np.loadtxt('./data/exp1/calAcc/area.txt').reshape(-1, 1)

    # np.savetxt('./data/exp1/calAcc/plot_labels.txt', labels)

    # gt = np.loadtxt('./data/area4/calAcc/gt.txt').reshape(-1, 1)
    # pred = np.loadtxt('./data/area4/calAcc/pred.txt').reshape(-1, 1)
    # area = np.loadtxt('./data/area4/calAcc/area.txt').reshape(-1, 1)

    labels = calLabel(gt, pred).astype(np.int32)

    np.savetxt('./data/area3/calAcc/plot_labels.txt', labels)

    m = confusion_matrix(gt, pred)
    print(classification_report(gt, pred, digits=4))
    print(IoU(m))
