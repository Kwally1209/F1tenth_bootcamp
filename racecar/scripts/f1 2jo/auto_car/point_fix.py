from bokeh.models.widgets import Button
from bokeh.plotting import figure, curdoc
from bokeh.models import ColumnDataSource, PointDrawTool
from bokeh.layouts import layout
import numpy as np

ref = np.genfromtxt('./map_colored-centerline.csv', delimiter=',', skip_header = 1)
source = ColumnDataSource(data=dict(x=ref[:,0], y=ref[:,1]))

print("1")


original_length = len(source.data['x'])

p = figure(x_range=(min(ref[:,0])-1, max(ref[:,0])+1), y_range=(min(ref[:,1])-1, max(ref[:,1])+1),
           width=400, height=400, tools=[], title='Drag the points')
renderer = p.scatter(x='x', y='y', source=source, color='blue', alpha=0.5, size=10)
print("2")
def callback(attr, old, new):
    if len(new) > original_length:
        new.pop()
        source.data = dict(x=new)

source.on_change('data', callback)
print("3")
draw_tool = PointDrawTool(renderers=[renderer], empty_value='black')
p.add_tools(draw_tool)
p.toolbar.active_tap = draw_tool

# button widget
button = Button(label="Save", button_type="success")

# button callback function
def save_data():
    df = source.to_df()
    df.to_csv('new_points.csv', index=False)

button.on_click(save_data)

# add to layout and display
curdoc().add_root(layout([p, button]))
print("5")