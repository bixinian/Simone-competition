##Visualization for AVP sample
##Change AVP_LOG in AVPLog.hpp to 1 to enable data logging
##51World

import matplotlib.pyplot as plt
import numpy as np
import csv

def ReadCSV(filename):
    ifile = open(filename, 'r')
    reader = csv.reader(ifile, delimiter=",")
    data = []
    for row in reader:
        row = [None if v is '' else float(v) for v in row]
        data.append(row)
    ifile.close()
    return data


plt.close('all')

#Enter the directory of your file(s)
files = ['F:/AVP.csv']

#Plot data
fig, ax = plt.subplots()
plt.title('AVP Sample', fontsize = 16)
plt.xlabel('x')
plt.xlabel('y')
legends = ['Parking space', 'Control points', 'Planned reverse trajectory', 'Planned forward trajectory', 'Planned leaving trajectory', 'Actual trajectory']
colors = ['black', 'red', 'blue', 'darkblue', 'maroon', 'lime']

for i in range(len(files)):
    data = ReadCSV(files[i])
    A = np.array(data)
    for j in range(A.shape[0]):
        row = A[j]
        x = [0 for index in range(int((len(row) - 1) / 2))]
        y = [0 for index in range(int((len(row) - 1) / 2))]
        for k in range(len(x)):
            x[k] = row[2 * k]
            y[k] = row[2 * k + 1]
        if j == 1:
            for t in range(len(x)):
                plt.plot(x[t], y[t], marker = 'o', fillstyle = 'none', color = colors[j], label = legends[j] if t==1 else '_nolegend_')
        else:
            plt.plot(x, y, color = colors[j], label = legends[j])
            
plt.legend()

plt.show()