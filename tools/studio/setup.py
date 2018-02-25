from setuptools import setup

setup(
    name='cvra_studio',
    version='0.1.0',
    description='Introspection tool used for debugging our robot at CVRA',
    author='Club Vaudois de Robotique Autonome',
    author_email='info@cvra.ch',
    url='http://cvra.ch',
    license='BSD',
    packages=['cvra_studio'],
    install_requires=[
        'PyQt5',
        'pyserial',
        'pyqtgraph',
        'numpy',
        'uavcan',
    ],
)
