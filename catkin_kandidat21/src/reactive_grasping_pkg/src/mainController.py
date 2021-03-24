#! /usr/bin/env python
import numpy


class MainController:

    """constructor with printmessage for debugging"""
    def __init__(self):
        print('MainController constructor is called')

    """method converts degree to radian"""
    def degree_to_radian(self, degree):
        return numpy.deg2rad(degree)

    """method convert radian to degree"""
    def radian_to_degree(self, radian):
        return numpy.rad2deg(radian)

    if __name__ == '__main__':
        None


"""simple testing of methods in MainController"""
mc1 = MainController()
print('mc1.degree_to_radian()', mc1.degree_to_radian(10))
print('mc2.radian_to_degree()', mc1.radian_to_degree(1))


