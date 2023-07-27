import yaml
from bokeh.models.widgets import Button
from bokeh.plotting import figure, curdoc
from bokeh.models import ColumnDataSource, PointDrawTool, Image
from bokeh.layouts import layout
import numpy as np
from PIL import Image

# Load YAML file
yaml_file_path = './map0614.yaml'  # Replace with the path to your YAML file
with open(yaml_file_path, 'r') as f:
    yaml_data = yaml.safe_load(f)

image_path = './map0614.pgm'  # Path to the background image
resolution = yaml_data['resolution']  # Resolution of the image
origin = yaml_data['origin']  # Origin coordinates [x, y, z]

# Convert PGM to PNG
pgm_image_path = image_path  # Assume the image path is the same as the one in the YAML file
png_image_path = './map0614.png'  # Output PNG image path
pgm_image = Image.open(pgm_image_path)
pgm_image.save(png_image_path)

ref = np.genfromtxt('./map_colored_0614-centerline.csv', delimiter=',', skip_header=1)
source = ColumnDataSource(data=dict(x=ref[:, 0], y=ref[:, 1]))

original_length = len(source.data['x'])

# Calculate the scaling factor and adjusted origin based on resolution
scaling_factor =  resolution
adjusted_origin = [origin[0] - (min(ref[:, 0]) * scaling_factor),
                   origin[1] - (min(ref[:, 1]) * scaling_factor)]

p = figure(x_range=(min(ref[:, 0])-1, max(ref[:, 0])+1), y_range=(min(ref[:, 1])-1, max(ref[:, 1])+1),
           width=400, height=400, tools=[], title='Drag the points')

# Load the PNG image as the background, flip vertically, and adjust position and scaling
image = Image.open(png_image_path)
image = image.transpose(Image.FLIP_TOP_BOTTOM)
# p.image(image=[np.array(image)], x=adjusted_origin[0], y=adjusted_origin[1],
#         dw=(max(ref[:, 0]) - min(ref[:, 0])) * scaling_factor,
#         dh=(max(ref[:, 1]) - min(ref[:, 1])) * scaling_factor, alpha=0.5)

p.image(image=[np.array(image)], x=adjusted_origin[0], y=adjusted_origin[1],
        dw=len(np.array(image)[0,:])*scaling_factor, dh=len(np.array(image))*scaling_factor, alpha=0.5)



renderer = p.scatter(x='x', y='y', source=source, color='blue', alpha=0.5, size=10)


def callback(attr, old, new):
    if len(new) > original_length:
        new.pop()
        source.data = dict(x=new)


source.on_change('data', callback)

draw_tool = PointDrawTool(renderers=[renderer], empty_value='black')
p.add_tools(draw_tool)
p.toolbar.active_tap = draw_tool

# Button widget
button = Button(label="Save", button_type="success")


# Button callback function
def save_data():
    df = source.to_df()
    df.to_csv('new_points.csv', index=False)


button.on_click(save_data)

# Add to layout and display
curdoc().add_root(layout([p, button]))