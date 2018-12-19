import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class Point():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.s = 1
        self.c = 1

class AnimatedScatter(object):

    points = []


    """An animated scatter plot using matplotlib.animations.FuncAnimation."""
    def __init__(self, numpoints=50):
        self.numpoints = numpoints
        self.stream = self.data_stream()

        # Setup the figure and axes...
        self.fig, self.ax = plt.subplots()
        # Then setup FuncAnimation.
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=5, 
                                           init_func=self.setup_plot, blit=True)

        self.points = [
            Point(0,0),
            Point(10,10)
        ]

    def setup_plot(self):
        """Initial drawing of the scatter plot."""
        x, y, s, c = next(self.stream)
        self.scat = self.ax.scatter(x, y, c=c, s=s, animated=True)
        self.ax.axis([-10, 10, -10, 10])

        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,

    def data_stream(self):



        data = [[p.x, p.y, p.s, p.c] for p in self.points]
        print(data)
        x, y = data[:2, :]
        s, c = data[2:, :]

        while True:
            xy += 0# * (np.random.random((2, self.numpoints)) - 0.5)
            s += 0 #0.05# * (np.random.random(self.numpoints) - 0.5)
            c += 0 #.02# * (np.random.random(self.numpoints) - 0.5)
            yield data

    def update(self, i):
        """Update the scatter plot."""
        data = next(self.stream)

        # Set x and y data...
        self.scat.set_offsets(data[:2, :])
        # Set sizes...
        self.scat._sizes = 300 * abs(data[2])**1.5 + 100
        # Set colors..
        self.scat.set_array(data[3])

        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,

    def show(self):
        plt.show()

if __name__ == '__main__':
    a = AnimatedScatter()
    a.show()