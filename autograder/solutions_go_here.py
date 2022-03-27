#!/usr/bin/env python

"""
This is an optional submission for the lab5 written portion autograder. 
This is so you can check to make sure that your understanding is 
correct before you get a headstart on the coding. You will not get 
credit for submitting to this autograder. It's just for your testing. 
This pset will be manually graded
"""
import numpy as np

def answer_to_1i():
    """
    Return your answer to 1i in a python list, [x, y, theta]
    """
    return [((2*(3**(1./2)))+1)/20., (3**(1./2)-2)/20., (np.pi)/60.]

def answer_to_1ii():
    """
    Return your answer to 1ii in a python list, [x, y, theta]
    """
    return [((4*np.sqrt(3) - 2)/40) + 3, ((2*np.sqrt(3) + 4)/40) + 4, (21*np.pi)/60]

def answer_to_2():
    """
    Return your answers to 2 in a python list for the values z=0,3,5,8,10
    Each value should be a float
    """
    return [.032, .0234, .0179, .0919, .712]

