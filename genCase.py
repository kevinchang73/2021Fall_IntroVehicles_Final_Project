import numpy as np
import sys

length = int(sys.argv[1])
file_name = sys.argv[2]
out_str = []

#out_str.append(str(float(10/(length-1)))+'\n')
#out_str.append(str(float(15/(length-1)))+'\n')
out_str.append(str(float(10/5))+'\n')
out_str.append(str(float(15/5))+'\n')
out_str.append(str(1.0)+'\n')
out_str.append(str(2.0)+'\n')
out_str.append(str(6)+'\n')
out_str.append(str(5)+'\n')
out_str.append(str(2)+'\n')
out_str.append(str(length)+'\n')
at = 0
for i in range(0, length):
    out_str.append(str(i)+' ')
    out_str.append(str(np.random.randint(0,5)) + ' ')
    out_str.append(str(np.random.randint(0,2)) + ' ')
    at = at + np.random.exponential(1)
    out_str.append(str(at) + '\n')

out = open(file_name, 'w')
out.writelines(out_str)
out.close()
