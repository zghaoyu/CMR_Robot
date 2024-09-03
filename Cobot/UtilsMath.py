import math
import copy

def calcListAvg(listVal):
    lenList = len(listVal)
    avg = 0.0
    if lenList > 0:
        for val in listVal:
            avg = avg + float(val / lenList)
    return avg

def vectorCrossAngle(v1, v2):  # v1 = [x1,y1,x2,y2], v2 = [x3,y3,x4,y4]
    dx1 = v1[2] - v1[0]
    dy1 = v1[3] - v1[1]
    dx2 = v2[2] - v2[0]
    dy2 = v2[3] - v2[1]
    angle1 = math.atan2(dy1, dx1)
    angle1 = round(angle1 * 180 / math.pi, 3)
    # print(angle1)
    angle2 = math.atan2(dy2, dx2)
    angle2 = round(angle2 * 180 / math.pi, 3)
    # print(angle2)
    if angle1 * angle2 >= 0:
        included_angle = abs(angle1 - angle2)
    else:
        included_angle = abs(angle1) + abs(angle2)
        if included_angle > 180:
            included_angle = 360 - included_angle
    return round(included_angle,3)

def getDictVal(dictData, key, defaultVal):
    try:
        val = dictData[key]
    except Exception as e:
        val = defaultVal
    return val

def getLineStatisticParas(line):
    minX = line[0][0]; maxX = minX; minY = line[0][1]; maxY = minY
    lenPts = len(line)
    hist_x = {}; hist_y = {}
    for i in range(lenPts):
        if minX > line[i][0]: minX = line[i][0]
        if maxX < line[i][0]: maxX = line[i][0]
        if minY > line[i][1]: minY = line[i][1]
        if maxY < line[i][1]: maxY = line[i][1]
        hist_x[line[i][0]] = getDictVal(hist_x, line[i][0], 0) + 1
        hist_y[line[i][1]] = getDictVal(hist_y, line[i][1], 0) + 1

    maxHistX = [-1,0]; maxHistY = [-1,0]
    for k in hist_x.keys():
        if maxHistX[1] < hist_x[k]: maxHistX = [k,hist_x[k]] # x appears times
    for k in hist_y.keys():
        if maxHistY[1] < hist_y[k]: maxHistY = [k,hist_y[k]] # y appears times

    sumX_1 = 0; sumX_2 = 0; sumY_1 = 0; sumY_2 = 0
    for i in range(lenPts):
        if maxHistX[0] == line[i][0] + 1: sumX_1 = sumX_1 + 1
        if maxHistX[0] == line[i][0] - 1: sumX_2 = sumX_2 + 1
        if maxHistY[0] == line[i][1] + 1: sumY_1 = sumY_1 + 1
        if maxHistY[0] == line[i][1] - 1: sumY_2 = sumY_2 + 1
    if sumX_1 > sumX_2:
        maxHistX.append(-1)
        maxHistX.append(sumX_1)
    else:
        maxHistX.append(1)
        maxHistX.append(sumX_2)
    if sumY_1 > sumY_2:
        maxHistY.append(-1)
        maxHistY.append(sumY_1)
    else:
        maxHistY.append(1)
        maxHistY.append(sumY_2)

    return [minX, maxX, minY, maxY], maxHistX, maxHistY

def calcPointToLineDistance(linePts, point, lineParas = []):
    pointToLineD = 4080
    if not linePts is None:
        lenPts = len(linePts)
        if lenPts > 1:
            lineStatisticParas = []
            if len(lineParas) == 3:
                slope = lineParas[0]; offset = lineParas[1]
            else:
                startIdx = 0; stopIdx = lenPts - 1
                if lenPts > 10:
                    startIdx = int(lenPts*0.1); stopIdx = int(lenPts*0.9)
                slope, offset, rsq, lineStatisticParas = lineFitting(linePts, startIdx, stopIdx)
                if rsq < 0.5:
                    slope, offset, rsq, lineStatisticParas = lineFitting(linePts, startIdx, stopIdx, True)
            if slope != 0:
                verticalLineSlope = -1 / slope
                verticalLineOffset = point[1] - verticalLineSlope * point[0]
                crossP_x = (offset - verticalLineOffset) / (verticalLineSlope - slope)
                crossP_y = (offset*verticalLineSlope-verticalLineOffset*slope)/(verticalLineSlope-slope)
                pointToLineD = math.sqrt(math.pow(crossP_x-point[0],2)+math.pow(crossP_y-point[1],2))
            else:
                if len(lineStatisticParas) == 3:
                    xyRng = lineStatisticParas[0]; xHist = lineStatisticParas[1]; yHist = lineStatisticParas[2]
                else:
                    xyRng, xHist, yHist = getLineStatisticParas(linePts)
                if(abs(xyRng[3] - xyRng[2]) < abs(xyRng[1] - xyRng[0])):
                    pointToLineD = abs(point[1] - yHist[0]) # parallel to x axis
                else:
                    pointToLineD = abs(point[0] - xHist[0]) # parallel to y axis
    return pointToLineD

def calTwoPointsDistance(p1, p2):
    d1 = abs(p1[0] - p2[0])
    d2 = abs(p1[1]-p2[1])
    d = round(math.sqrt(math.pow(d1,2) + math.pow(d2,2)),3)
    return d

def calTwoLinesCrossPoint(line_xyPoints_1, line_xyPoints_2):
    crossPoint = []
    len_1 = len(line_xyPoints_1); len_2 = len(line_xyPoints_2)
    if len_1 > 1 and len_2 > 1:
        slope_1, offset_1, rsq_1, lineStatisticParas_1 = lineFitting(line_xyPoints_1, int(len_1*0.1), int(len_1*0.9), True)
        slope_2, offset_2, rsq_2, lineStatisticParas_2 = lineFitting(line_xyPoints_2, int(len_2*0.1), int(len_2*0.9), True)
        if slope_1 != 0 and slope_2 != 0 and round(slope_2 - slope_1, 3) != 0:
            crossP_x = (offset_1 - offset_2) / (slope_2 - slope_1)
            crossP_y = (offset_1 * slope_2 - offset_2 * slope_1) / (slope_2 - slope_1)
            crossAngle = vectorCrossAngle([crossP_x,crossP_y,crossP_x+10,slope_1*(crossP_x+10)+offset_1],[crossP_x,crossP_y,crossP_x+10,slope_2*(crossP_x+10)+offset_2])
            crossPoint = [crossP_x, crossP_y, round(crossAngle)]

        if slope_1 != 0 and slope_2 == 0:
            xyRng = lineStatisticParas_2[0]
            if (abs(xyRng[3] - xyRng[2]) < abs(xyRng[1] - xyRng[0])):
                # parallel to x axis
                crossP_y = line_xyPoints_2[0][1]
                crossP_x = (crossP_y - offset_1) / slope_1
                crossAngle = vectorCrossAngle([crossP_x, crossP_y, crossP_x + 10, slope_1 * (crossP_x + 10) + offset_1],
                                              [crossP_x, crossP_y, crossP_x + 10, crossP_y])
            else:
                # parallel to y axis
                crossP_x = line_xyPoints_2[0][0]
                crossP_y = crossP_x * slope_1 + offset_1
                crossAngle = vectorCrossAngle([crossP_x, crossP_y, crossP_x + 10, slope_1 * (crossP_x + 10) + offset_1],
                                              [crossP_x, crossP_y, crossP_x, crossP_y + 10])
            crossPoint = [crossP_x, crossP_y, round(crossAngle)]

        if slope_1 == 0 and slope_2 != 0:
            xyRng = lineStatisticParas_1[0]
            if (abs(xyRng[3] - xyRng[2]) < abs(xyRng[1] - xyRng[0])):
                # parallel to x axis
                crossP_y = line_xyPoints_1[0][1]
                crossP_x = (crossP_y - offset_2) / slope_2
                crossAngle = vectorCrossAngle([crossP_x, crossP_y, crossP_x + 10, crossP_y],
                                              [crossP_x, crossP_y, crossP_x + 10, slope_2 * (crossP_x + 10) + offset_2])
            else:
                # parallel to y axis
                crossP_x = line_xyPoints_1[0][0]
                crossP_y = crossP_x * slope_2 + offset_2
                crossAngle = vectorCrossAngle([crossP_x, crossP_y, crossP_x, crossP_y + 10],
                                              [crossP_x, crossP_y, crossP_x + 10, slope_2 * (crossP_x + 10) + offset_2])
            crossPoint = [crossP_x, crossP_y, round(crossAngle)]

        if slope_1 == 0 and slope_2 == 0:
            xyRng = lineStatisticParas_1[0]
            if (abs(xyRng[3] - xyRng[2]) < abs(xyRng[1] - xyRng[0])):
                # line_1 is parallel to x axis
                xyRng = lineStatisticParas_2[0]
                if (abs(xyRng[3] - xyRng[2]) > abs(xyRng[1] - xyRng[0])):
                    # line_2 is parallel to y axis
                    crossP_y = line_xyPoints_1[0][1]
                    crossP_x = line_xyPoints_2[0][0]
                    crossPoint = [crossP_x, crossP_y, 90]
            else:
                # line_1 is parallel to y axis
                xyRng = lineStatisticParas_2[0]
                if (abs(xyRng[3] - xyRng[2]) < abs(xyRng[1] - xyRng[0])):
                    # line_2 is parallel to x axis
                    crossP_y = line_xyPoints_2[0][1]
                    crossP_x = line_xyPoints_1[0][0]
                    crossPoint = [crossP_x, crossP_y, 90]

    return crossPoint

def adjustLine(linePoints):
    xyPoints = copy.deepcopy(linePoints)
    lenPts = len(xyPoints)
    xyRng, xHist, yHist = getLineStatisticParas(xyPoints)
    if (xyRng[1] - xyRng[0] <= 3 and (xHist[1] + xHist[3])*2 >= lenPts):
        for i in range(lenPts):
            xyPoints[i][0] = xHist[0]
    if (xyRng[3] - xyRng[2] <= 3 and (yHist[1] + yHist[3])*2 >= lenPts):
        for i in range(lenPts):
            xyPoints[i][1] = yHist[0]
    return xyPoints, [xyRng, xHist, yHist]

def lineFitting(linePoints, startIdx = -1, stopIdx = -1, adjLine = False):
    slope = 0.0; offset = 0.0; rsq = 0.0
    lineStatisticParas = []
    if not linePoints is None:
        if adjLine:
            xyPoints, lineStatisticParas = adjustLine(linePoints)
        else:
            xyPoints = linePoints

        if startIdx >= 0 and (stopIdx - startIdx) > 1:
            stopIdx = stopIdx + 1
            lenPts = stopIdx - startIdx
        else:
            lenPts = len(xyPoints)
            startIdx = 0
            stopIdx = lenPts

        if lenPts > 1:
            xmean = 0.0; ymean = 0.0
            for i in range(startIdx, stopIdx):
                xmean = xmean + float(xyPoints[i][0] / lenPts)
                ymean = ymean + float(xyPoints[i][1] / lenPts)
            xmean = round(xmean, 7)
            ymean = round(ymean, 7)

            sumx2 = 0.0; sumy2 = 0.0; sumxy = 0.0
            for i in range(startIdx, stopIdx):
                sumx2 = sumx2 + (xyPoints[i][0] - xmean)*(xyPoints[i][0] - xmean)
                sumy2 = sumy2 + (xyPoints[i][1] - ymean) * (xyPoints[i][1] - ymean)
                sumxy = sumxy + (xyPoints[i][0] - xmean) * (xyPoints[i][1] - ymean)
            sumx2 = round(sumx2, 7)
            sumy2 = round(sumy2, 7)

            if sumx2 != 0: slope = round(sumxy / sumx2, 5)
            if sumxy == 0 and sumx2 == 0:
                offset = round(ymean - xmean, 5)
            else:
                offset = round(ymean - slope * xmean, 5)

            if sumx2 != 0 and sumy2 != 0:
                rsq = round(sumxy*sumxy / (sumx2*sumy2), 5)
            else:
                rsq = 1.0

    return slope, offset, rsq, lineStatisticParas


# add by Hui Zhi 2022/8/3
def cal_ratio(a, b):
    return a/b if a < b else b/a
