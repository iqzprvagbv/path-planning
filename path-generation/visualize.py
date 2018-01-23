# Creates visualizations

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns

def draw_spline(spline,canvas):
    points = []
        for t in range(1000):
            points.append(self.eval(t/1000.0))

        x,y = zip(*points)
        canvas.plot(x,y)
