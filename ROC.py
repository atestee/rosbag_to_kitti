import numpy as np
import os

from matplotlib import pyplot as plt


def get_scores(dir):
    scores = []
    for file in os.listdir(dir):
        filename = os.path.join(dir, file)
        f = open(filename, 'r')
        label = f.read().split(' ')
        scores.append(float(label[-1]))
    return np.array(scores)

def get_gt_classes(dir):
    gt_classes = []
    p_cnt = 0
    n_cnt = 0
    for file in os.listdir(dir):
        filename = os.path.join(dir, file)
        f = open(filename, 'r')
        labels = f.read().split('\n')
        if len(labels) > 2:
            label = labels[1].split(' ')[0]
        else:
            label = labels[0].split(' ')[0]

        if label == 'rescue_randy':
            p_cnt += 1
            gt_classes.append(1)
        else:
            n_cnt += 1
            gt_classes.append(0)
    return (np.array(gt_classes), p_cnt, n_cnt)


def normalize_scores(scores):
    min = np.min(scores)
    if min < 0:
        scores = np.add(scores, np.abs(min))
    max = np.max(scores)
    scores = np.divide(scores, max)
    return scores


def trapezoid_area(x1, x2, y1, y2):
    base = np.abs(x1 - x2)
    height = (y1 + y2) / 2
    return base * height


def get_roc(scores, groundtruth, p, n):
    truth = [x for _, x in sorted(zip(scores, groundtruth), reverse=True)]
    scores = sorted(scores, reverse=True)
    FP = TP = 0
    FP_prev = TP_prev = 0
    AUC = 0
    R_x = []
    R_y = []
    f_prev = - float("inf")
    i = 0
    while (i < p + n):
        if (scores[i] != f_prev):
            AUC += trapezoid_area(FP, FP_prev, TP, TP_prev)
            R_x.append(FP / n)
            R_y.append(TP / p)
            f_prev = scores[i]
            FP_prev = FP
            TP_prev = TP
        if (truth[i] == 1):
            TP += 1
        else:
            FP += 1
        i += 1

    R_x.append(FP / n)
    R_y.append(TP / p)
    R_x.append(1)
    R_y.append(1)

    AUC += trapezoid_area(n, FP_prev, n, TP_prev)
    AUC /= (p * n)

    return [R_x, R_y], AUC




if __name__ == '__main__':
    scores = get_scores('/media/atestee/Verbatim/rosdata/test/data')
    (gt_classes, p, n) = get_gt_classes('/media/atestee/Verbatim/rosdata/test/label_2')
    normalize_scores(scores)
    ROC, AUC = get_roc(scores, gt_classes, p, n)

    print("AUC: " + str(AUC))

    plt.figure(1)
    plt.plot(ROC[0], ROC[1])
    plt.plot([0, 1], [0, 1])

    plt.savefig('ROC-test.png')
    plt.show()



