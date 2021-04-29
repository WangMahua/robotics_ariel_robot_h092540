#! /usr/bin/env python3
# coding=UTF-8
# This Python file uses the following encoding: utf-8

# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import numpy as np
import xlrd
from numpy.linalg import inv


def main():

    data = xlrd.open_workbook('HW5-1.xls')
    for n in range(len(data.sheet_names())):
        table = data.sheets()[n]

    input1=np.copy(table.col_values(0)[1:])
    input2=np.copy(table.col_values(1)[1:])
    output=np.copy(table.col_values(2)[1:])
    A=np.array([input1,input2]) #A should be A_t
    A_t=np.transpose(A)
    x=np.matmul(A, A_t) 
    x_inv=inv(x)
    x=np.matmul(x_inv,A)
    x=np.matmul(x,output)
    print(x)
    
    
    
if __name__ == '__main__':
    main()
