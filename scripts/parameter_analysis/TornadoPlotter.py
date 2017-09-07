import numpy as np
from matplotlib import pyplot as plt

def TornadoPlotter(base, lows, highs, labels, title='TornadoPlot', barWidth=0.8, whitespaceEdgeBuffer=0.1):
    # Plot the bars, one by one
    # Sort by the delta range
    for row_counter, (low, high) in enumerate(sorted(zip(lows, highs),key=lambda x: abs(x[1]-x[0]))):
        # The width of the 'low' and 'high' pieces
        low_width = abs(base - low)
        high_width = abs(high - base)

        # Each bar is a "broken" horizontal bar chart
        if low < high:
            plt.broken_barh(
                [(low, low_width), (base, high_width)],
                (row_counter - barWidth / 2, barWidth),
                facecolors=['blue', 'red'],  # Try different colors if you like
                edgecolors=['black', 'black'],
                linewidth=1, )
        else:
            plt.broken_barh(
                [(high, high_width), (base, low_width)],
                (row_counter - barWidth / 2, barWidth),
                facecolors=['red', 'blue'],  # Try different colors if you like
                edgecolors=['black', 'black'],
                linewidth=1, )

    # Draw a vertical line down the middle
    plt.axvline(base, color='black')

    # Position the x-axis on the top, hide all the other spines (=axis lines)
    axes = plt.gca()  # (gca = get current axes)
    axes.spines['left'].set_visible(False)
    axes.spines['right'].set_visible(False)
    axes.spines['bottom'].set_visible(False)
    axes.xaxis.set_ticks_position('top')
    axes.xaxis.set_label_position('top')

    # Make the y-axis display the variables
    plt.yticks(range(len(labels)), labels)

    # Set the portion of the x- and y-axes to show
    span = max(abs(x - base) for x in lows + highs)
    plt.xlim(base - (span * (1 + whitespaceEdgeBuffer)),
             base + (span * (1 + whitespaceEdgeBuffer)))
    plt.ylim(-1, len(labels))
    plt.xlabel(title, fontsize=20)


    plt.show()


if __name__ == "__main__":
    
    labels = [
        'apple',
        'juice',
        'orange',
        'peach',
        'gum',
        'stones',
        'bags',
        'lamps',
    ]

    base = 3000

    lows = [
        base + 246 / 2,
        base - 1633 / 2,
        base - 500 / 2,
        base - 150 / 2,
        base - 35 / 2,
        base - 36 / 2,
        base - 43 / 2,
        base - 37 / 2,
    ]

    highs =  [
        base - 246 / 3,
        base + 300 / 3,
        base + 500 / 3,
        base + 150 / 3,
        base + 35 / 3,
        base + 36 / 3,
        base + 43 / 3,
        base + 37 / 3,
    ]

    TornadoPlotter(base, lows, highs, labels)