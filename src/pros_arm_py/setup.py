from setuptools import find_packages, setup
from glob import glob

package_name = 'pros_arm_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch',  glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylin',
    maintainer_email='kylingithubdev@gmail.com',
    description='This is a package to create a framework for development in PAIA.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard = pros_arm_py.keyboard:main',
            'random = pros_arm_py.random:main',
            'arm_reader = pros_arm_py.arm_reader:main',
            'arm_writer = pros_arm_py.arm_writer:main',
            'mock = pros_arm_py.mock:main',
            
        ],
    },
)
