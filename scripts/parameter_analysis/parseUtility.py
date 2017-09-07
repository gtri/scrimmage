import os
import matplotlib.pyplot as plt


def GetUtility(fileName):
    f = open(fileName, 'r')
    # Get last line
    for line in f:
        pass
    lastLine = line.split(',')

    utility = lastLine[-1]

    return utility

dir = '/home/kbowers6/swarm-log/pw-p2/'
files = os.listdir(dir)

sumUtility = 0
for file in files:
    utility = GetUtility(dir + file + '/summary.csv')
    sumUtility += float(utility)
    # print utility

avgUtility = sumUtility / len(files)
print 'Average Utility:', avgUtility

# example plot
plt.plot([0, .1, .2, .3, .5, 1, 2, 3], [0.35, 0.55, .76, .85, .97, 1, 1, 1.08], 'ro')
plt.xlabel('Priority Weight')
plt.ylabel('Utility')
plt.show()