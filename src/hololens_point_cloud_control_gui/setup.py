from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['hololens_point_cloud_control_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/hololens_point_cloud_control_gui']
)

setup(**d)