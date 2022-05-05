from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['naive_spatial_mapper_control_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/naive_spatial_mapper_control_gui']
)

setup(**d)