from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['octomap_based_spatial_mapper_control_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/octomap_based_spatial_mapper_control_gui']
)

setup(**d)