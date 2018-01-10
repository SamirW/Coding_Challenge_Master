# import random
# import numpy as np
# import pylab as pl
# from matplotlib import collections as mc

"""
Collision check code adapted from 
http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
"""

def is_collision(line_seg1, line_seg2):
    """
    Checks for a collision between line segments p1(x1, y1) -> q1(x2, y2)
    and p2(x3, y3) -> q2(x4, y4)
    """

    def on_segment(p1, p2, p3):
        if (p2[0] <= max(p1[0], p3[0])) & (p2[0] >= min(p1[0], p3[0])) & (p2[1] <= max(p1[1], p3[1])) & (p2[1] >= min(p1[1], p3[1])):
           return True
        return False


    def orientation(p1, p2, p3):
        val = ((p2[1] - p1[1]) * (p3[0] - p2[0])) - ((p2[0] - p1[0]) * (p3[1] - p2[1]))

        if val == 0:
            return 0
        elif val > 0:
            return 1
        elif val < 0:
            return 2

    p1, q1 = line_seg1[0], line_seg1[1]
    p2, q2 = line_seg2[0], line_seg2[1]

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if (o1 != o2) & (o3 != o4):
        return True

    if (o1 == 0 & on_segment(p1, p2, q1)):
        return True
 
    if (o2 == 0 & on_segment(p1, q2, q1)):
        return True

    if (o3 == 0 & on_segment(p2, p1, q2)):
        return True
 
    if (o4 == 0 & on_segment(p2, q1, q2)):
        return True

    return False


#########################################
# Below - code for debugging            #
# and testing collision check code #    #
#########################################

# mylist = [(random.randint(1, 100), random.randint(1, 100)) for k in range(1000*4)]

# p1 = (49, 35)
# q1 = (30, 21)
# p2 = (89, 34)
# q2 = (44, 69)

# print is_collision(p1, q1, p2, q2)

# for k in range(len(mylist)/4):
#     p1 = mylist[4*k]
#     q1 = mylist[4*k+1]
#     p2 = mylist[4*k+2]
#     q2 = mylist[4*k+3]

#     if not is_collision(p1, q1, p2, q2):
#         lines = [[p1, q1], [p2,q2]]

#         print [p1, q1]
#         print [p2, q2]
        
#         lc = mc.LineCollection(lines)
#         fig, ax = pl.subplots()
#         ax.add_collection(lc)
#         ax.autoscale()
#         ax.margins(0.1)
#         pl.show()