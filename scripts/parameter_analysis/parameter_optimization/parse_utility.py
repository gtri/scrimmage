import os
import matplotlib.pyplot as plt
import pandas as pd


def GetUtility(fileName, columnName):
    df = pd.read_csv(fileName)
    utility = df[columnName].mean()
    return utility

def GetAverageUtility(directory, columnName='Utility'):
    files = os.listdir(directory)

    sumUtility = 0
    for file in files:
        utility = GetUtility(directory + '/' + file + '/summary.csv', columnName)
        sumUtility += utility

    avgUtility = sumUtility / len(files)
    return avgUtility

if __name__ == "__main__":
    dir = '/home/kbowers6/swarm-log/pw-p2'
    print(GetAverageUtility(dir))

    # example plot
    plt.plot([0, .1, .2, .3, .5, 1, 2, 3], [0.35, 0.55, .76, .85, .97, 1, 1, 1.08], 'ro')
    plt.xlabel('Priority Weight')
    plt.ylabel('Utility')
    plt.show()
