
import csv
from decimal import *
import math
from math import *
import sys
import numpy
from numpy import *
from argparse import ArgumentParser

def main():

    # Ground Truth and Measurement Path
    parser = ArgumentParser()
    parser.add_argument("-g", "--ground-truth", nargs='+', help="the ground truth csv file you want to evaluation", dest="gt", default="default")
    parser.add_argument("-r", "--result", nargs='+', help="your result csv file you want to evaluation", dest="result", default="default")
    parser.add_argument("-s", "--sigma", nargs='+', help="the covariance of the ground truth (translation, yaw, pitch, roll)", dest="sigma", default=[0.1, 0.0017, 0.0017, 0.0017])
    args = parser.parse_args()

    gt_list = []
    meas_list = []
    for num in range(len(args.gt)):
        gt_list += read_csv(args.gt[num])
        meas_list += read_csv(args.result[num])

    print ("\n***Calculation Starts***\n")
    score, yaw_score, pitch_score, roll_score = calscore(gt_list, meas_list, args)
    print ("\nNumber of your frames: %d" %(len(meas_list)))
    print ("Total score: %s %s %s %s" %(score, yaw_score, pitch_score, roll_score))
    print ("final score: %s\n" %(0.4*score + 0.4* yaw_score + 0.1*pitch_score + 0.1*roll_score))
    print ("***Calculation Finished***")

def min_angle(gt, meas):
    _list = [abs(gt-2*math.pi - meas), abs(gt - meas), abs(gt + 2 * math.pi - meas)]
    return min(_list)

def calscore(_gt, _meas, args):

    _dist_score = []
    _yaw_score = []
    _pitch_score = []
    _roll_score = []
    _total_yaw_score = 0
    _total_dist_score = 0
    _total_pitch_score = 0
    _total_roll_score = 0

    for i in range(len(_gt)):
        Notfound = True
        for m in range(len(_meas)):
            meas = _meas[m]
            if( abs(_gt[i][0] - meas[0]) < 1E-6):
                gt = numpy.array(_gt[i][1:3])
                mm = numpy.array(meas[1:3])
                dist = numpy.linalg.norm(gt-mm)
                _dist_score.append(grading(dist, float(args.sigma[0])))
                _yaw_score.append(grading(min_angle(_gt[i][4], meas[4]), float(args.sigma[1])))
                _pitch_score.append(grading(min_angle(_gt[i][5], meas[5]), float(args.sigma[2])))
                _roll_score.append(grading(min_angle(_gt[i][6], meas[6]), float(args.sigma[3])))
                Notfound = False
        if Notfound:
            _dist_score.append(0)
            _yaw_score.append(0)
            _pitch_score.append(0)
            _roll_score.append(0)
    _total_dist_score = sqrt(mean(square(_dist_score)))
    _total_yaw_score = sqrt(mean(square(_yaw_score)))
    _total_pitch_score = sqrt(mean(square(_pitch_score)))
    _total_roll_score = sqrt(mean(square(_roll_score)))

    return _total_dist_score, _total_yaw_score, _total_pitch_score, _total_roll_score

def grading(distance, sigma):

    if distance <= sigma :
        return 1
    elif distance > sigma and distance <= 2 * sigma:
        return 0.7
    elif distance > 2 * sigma and distance <= 3 * sigma:
        return 0.3
    else:
        return 0



def read_csv(_csv_file):

    _list = []

    f= open(_csv_file, 'r')
    data = csv.reader(f)

    for row in data:
        if row:
            _list.append(row)

    f.close()

    _list = check_data_type(_list)

    return _list


def check_data_type(_list):

    for m in range(len(_list)):
        _list[m][0]=float(_list[m][0])
        _list[m][1]=float(_list[m][1])
        _list[m][2]=float(_list[m][2])
        _list[m][3]=float(_list[m][3])
        _list[m][4]=float(_list[m][4])
        _list[m][5]=float(_list[m][5])
        _list[m][6]=float(_list[m][6])

    return _list

if __name__ == '__main__':
    main()
