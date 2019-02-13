from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_sdvoice'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_sdvoice.py']
)

setup(**d)
