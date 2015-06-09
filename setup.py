from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['aisoy_playground']
d['package_dir'] = {'': 'src'}

setup(**d)