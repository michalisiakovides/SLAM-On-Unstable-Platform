import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rollandpitch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michalis',
    maintainer_email='michalis.iacovides2002@gmail.com',
    description='Dissertation Project.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waveforms = rollandpitch.waveforms:main',
        
        ],
    },
)
