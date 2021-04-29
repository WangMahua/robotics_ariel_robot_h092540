#! /usr/bin/env python3
# coding=UTF-8
# This Python file uses the following encoding: utf-8

# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import numpy as np
import xlrd
from numpy.linalg import inv
from scipy.spatial.transform import Rotation
STEP = 0.1
G = 9.8

def quaternion_mult(a,b):
    c=np.array( [a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])
    return c

def norm(a):
    return ((a[0]**2)+(a[1]**2)+(a[2]**2)+(a[3]**2))**0.5

def norm_3(a):
    return ((a[0]**2)+(a[1]**2)+(a[2]**2))**0.5

def conjugate(a):
    a_c=np.array([a[0],-a[1],-a[2],-a[3]])
    
    return a_c/norm(a)

def descent(q,s):
    # q=np.array(q[0],-q[1],-q[2],-q[3])
    gravity=np.array([0,0,0,-G])
    gradient = np.array([-2*G*(norm(q)**(-2))*(q[1]*q[3]-q[0]*q[2])-s[1],
                        -2*G*(norm(q)**(-2))*(q[0]*q[1]+q[2]*q[3])-s[2],
                        -2*G*(0.5-(norm(q)**(-2))*(q[1]**2)+(q[2]**2))-s[3]])
    Jacobian = np.array([(   -2*q[2],    2*q[3],   -2*q[0],  2*q[1] ),
                        (     2*q[1],    2*q[0],   2*q[3],  2*q[2]  ),
                        (         0,   -4*q[1],   -4*q[2],      0)  ])
    J=-G*(norm(q)**(-2))*Jacobian
    J = np.transpose(J)
    
    print(Jacobian)
    print(gradient)
    a=np.matmul(J,gradient)

    print("a:"+str(a/norm(a)))
    print(STEP*a/norm(a))
    q_n = q - STEP*a/norm(a)
    
    print("q:"+str(q))
    print("q_n:"+str(q_n))
    return q_n,gradient
    


def main():

    data = xlrd.open_workbook('HW5-2.xls')
    for n in range(len(data.sheet_names())):
        table = data.sheets()[n]


    ax=np.copy(table.col_values(0)[1:])
    ay=np.copy(table.col_values(1)[1:])
    az=np.copy(table.col_values(2)[1:])



    flag=0
    data=open("output.txt",'w+') 
    data.write("q\t:w\ti:\tj:\tk:\n")
    for i in range(len(ax)):
        sensor=np.array([0,ax[i],ay[i],az[i]])
        q = np.array([10.,0.,0.,0.])
        q_new = np.array([0.,0.,1.,0.])
        gradient = np.array([1.,-1.,1.,-1.])
        while norm(q_new-q)>=0.001:
            print(i)
            print(sensor)
            q=q_new
            q_new,gradient = descent(q,sensor)

        
        data.write(str(i)+"\t:"+str(q_new[0])+"\t+"+str(q_new[1])+"i\t+"+str(q_new[2])+"j\t+"+str(q_new[3])+"k\t"+"\n")

    




    

    
    
if __name__ == '__main__':
    main()