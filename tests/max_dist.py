
from dis import dis
from math import dist
from this import d
import numpy as np

if __name__ == '__main__':
   
    dists = []
    with open("tests/distances.txt") as f:
        for line in f:
            dists.append(float(line.strip()))
    
    dists = np.array(dists)
    dists[763508] = 0
    dists[860719] = 0

    print("\n--------------")
    print("MAX DIST:", np.max(dists))
    print("--------------")
