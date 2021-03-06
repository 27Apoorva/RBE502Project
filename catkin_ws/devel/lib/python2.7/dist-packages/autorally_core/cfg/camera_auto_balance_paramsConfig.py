## *********************************************************
## 
## File autogenerated for the autorally_core package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 235, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 280, 'description': 'Top left vertex X position of ROI', 'max': 1279, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'roi_x_top_left', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 280, 'description': 'Top left vertex Y position of ROI', 'max': 1023, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'roi_y_top_left', 'edit_method': '', 'default': 500, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 280, 'description': 'Bottom right vertex X position of ROI', 'max': 1280, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'roi_x_bottom_right', 'edit_method': '', 'default': 1280, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 280, 'description': 'Bottom right vertex Y position of ROI', 'max': 1024, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'roi_y_bottom_right', 'edit_method': '', 'default': 1000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 280, 'description': 'Setpoint for exposure calibration', 'max': 255.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'msvGrayReference', 'edit_method': '', 'default': 120.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 280, 'description': 'Parameter that controls how fast shutter can change', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'kShutter', 'edit_method': '', 'default': 0.0005, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 280, 'description': 'Parameter that controls how fast gain can change', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'kGain', 'edit_method': '', 'default': 0.0005, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 280, 'description': 'Enable/Disable display of ROI and histogram of ROI', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'show_roi_and_hist', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

