from glob import glob

from setuptools import find_packages, setup

package_name = 'tukamukai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ratarara',
    maintainer_email='kh147ksyky359@gmail.com',
    description='CRANE-X7 と RealSense でテトリミノ型ピースをケースに詰めるパズルを解く',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
