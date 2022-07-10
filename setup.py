from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['husky_line_coverage'],
    package_dir={'': 'src'}
)
print("Running Setting")
setup(**d)