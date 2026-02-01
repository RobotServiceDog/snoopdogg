from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'snoopy_gait_provider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ammar-V',
    maintainer_email='ammarmust4@gmail.com',
    description='This package contains a gait scheduler for a quadruped.',
    license='TODO: License declaration',
    extras_require={
        'test': [
        ],
    },
    entry_points={
        'console_scripts': [
            'trot_gait_node = snoopy_gait_provider.trot_gait_node:main'
        ],
    },
)
