# Matplotlib

## Layers
1. Backend Layer: 
    - FigureCanvas: Defines the **area** on which the figure is drawn.
    - Renderer: The tool to **draw** on FigureCanvas.
    - Event: Handles user inputs from keyboard and mouse.
2. Artist Layer: `Artist` is the object that knows how to use a `Renderer` to paint on canvas. The `Artist` handles all the high level constructs like representing and laying out the figure, text, and lines. `Figure` is the main Artist object that holds everything together. 
    - Composite: Figure, Axes
    - Primitive: Line, Circle, Text
3. Scripting Layer: Scripting layer is the matplotlib.pyplot interface. Scripting layer automates the process of putting everthing together. Thus, it is easier to use than the Artist layer.

## Components
All are `Artist`, `Figure` create and possess `Axes`, `Axes` create and possess other primitive `Artist`.
1. `Figure`: The whole figure, keeps track of all the child `Axes`, can access them by `fig.axes[i]`.
2. [Axes](https://matplotlib.org/stable/tutorials/intermediate/artists.html): An `Axes` is a group of 'special' Artist (titles, figure legends, colorbars, etc) attached to a `Figure` that contains a region for plotting data (represent an individual plot), and usually includes two `Axis` (three in 3D case).
    - with many helper methods to create `Artist` instance and add them to the relevant `Axes`. These instances can be removed by `.remove()`. 不要通过`Axes.lines`, `Axes.patches`直接添加，因为还需要自动设置一些属性，通过`add_line`, `add_patch`来添加。
    ```python
    ax = fig.add_axes([left, bottom, width, height]) # create Axes at arbitrary position of Figure
    line, = ax.plot(x, y, color='blue')
    line = ax.lines[0] # the same as previous
    line.remove()
    ```
    [helper method](http://inshallah.againxx.cn/matplotlib_axes_helper_methods.png) <br>
    [axes attributes](http://inshallah.againxx.cn/Screenshot\ from\ 2022-10-19\ 12-24-45.png) <br>
3. `Axis`: Exact axis which provides ticks and tick labels to provide scales for data in `Axes`.
4. `Artist`: Everything visible on the Figure is an Artist (even Figure, Axes, and Axis objects). This includes `Text`, `Line2D`, `collections`, `Patch` objects. 
When the Figure is rendered, all of the Artists are drawn to the canvas. Most Artists are tied to an Axes; such an Artist cannot be shared by multiple Axes, or moved from one to another.
```python
fig = plt.figure()  # an empty figure with no Axes
ax1, ax2 = fig.subplots(2, 1)
fig, ax = plt.subplots()  # a figure with a single Axes
fig, axs = plt.subplots(2, 2)  # a figure with a 2x2 grid of Axes
axs[0, 0].plot(data)
fig, (ax1, ax2) = plt.subplots(2, 1)  # a figure with a 2x1 grid of Axes
```

## OO-style vs pyplot style usage
There are essentially two ways to use Matplotlib:
- Explicitly create Figures and Axes, and call methods on them (the "object-oriented (OO) style").
```python
x = np.linspace(0, 2, 100)  # Sample data.
# Note that even in the OO-style, we use `.pyplot.figure` to create the Figure.
fig, axs = plt.subplots(1, 2, figsize=(5, 2.7), layout='constrained')
xdata = np.arange(len(data1))  # make an ordinal for this
data = 10**data1
axs[0].plot(xdata, data)
axs[1].set_yscale('log')
axs[1].plot(xdata, data);
```

- Rely on pyplot to implicitly create and manage the Figures and Axes, and use pyplot functions for plotting.
```python
x = np.linspace(0, 2, 100)  # Sample data.
plt.figure(figsize=(5, 2.7), layout='constrained')
plt.plot(x, x, label='linear')  # Plot some data on the (implicit) axes.
plt.xlabel('x label')
plt.ylabel('y label')
plt.title("Simple Plot")
plt.legend();
```

## Characters
### rotated x label
```python
plt.rcParams.update({'figure.autolayout': True})
fig, ax = plt.subplots()
ax.barh(group_names, group_data)
labels = ax.get_xticklabels()
plt.setp(labels, rotation=45, horizontalalignment='right')
```

### mathematical expressions
```python
ax.set_title(r'$\sigma_i=15$')
```

### annotations
add some annotation as specific place.
```python
ax.annotate('local max', xy=(2, 1), xytext=(3, 1.5),
            arrowprops=dict(facecolor='black', shrink=0.05))
```
