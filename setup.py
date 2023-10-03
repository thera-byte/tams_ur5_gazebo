from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tams_ur5_gazebo_lib'],
    package_dir={'': 'scripts'}
)

setup(**d)