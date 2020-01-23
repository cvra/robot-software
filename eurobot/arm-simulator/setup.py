from setuptools import setup, find_packages

# read the contents of the README
from os import path

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="arm_simulator",
    version=0.1,
    description="Simulating robot arms using Hamiltonian dynamics & automatic differentiation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Club Vaudois de Robotique Autonome",
    author_email="info@cvra.ch",
    url="http://cvra.ch",
    license="BSD",
    packages=find_packages(exclude=["contrib", "docs", "tests*"]),
    install_requires=["autograd",],
)
