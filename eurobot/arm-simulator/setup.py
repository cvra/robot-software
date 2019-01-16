from setuptools import setup, find_packages

setup(
    name='arm_simulator',
    version=0.1,
    description='Simulating robot arms using Hamiltonian dynamics & automatic differentiation',
    author='Club Vaudois de Robotique Autonome',
    author_email='info@cvra.ch',
    url='http://cvra.ch',
    license='BSD',
    packages=find_packages(exclude=['contrib', 'docs', 'tests*']),
    install_requires=[
        'autograd',
    ],
)
