#!/usr/bin/env python

import numpy as np
import math
from numpy import *


p0_x = 2
p0_y = 2

p1_x = 0.5
p1_y = 0.5

R = np.matrix([[p0_x*p1_x, p0_y*p1_x],
               [p0_x*p1_y, p0_y*p1_y]])

print(R)



