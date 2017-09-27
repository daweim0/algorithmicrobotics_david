## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# Expose the modules so they can used by other packages
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['hampy'],
    package_dir={'': 'include'},
)

setup(**setup_args)
