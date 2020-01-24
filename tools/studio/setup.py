from setuptools import setup, find_packages

from cvra_studio import __version__

setup(
    name="cvra_studio",
    version=__version__,
    description="Introspection tool used for debugging our robot at CVRA",
    author="Club Vaudois de Robotique Autonome",
    author_email="info@cvra.ch",
    url="http://cvra.ch",
    license="BSD",
    packages=find_packages(exclude=["contrib", "docs", "tests*"]),
    install_requires=["docopt", "PyQt5", "pyserial", "pyqtgraph", "numpy", "uavcan",],
    entry_points={"console_scripts": ["cvra=cvra_studio.cli:main",],},
)
