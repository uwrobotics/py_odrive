from setuptools import setup

package_name = 'py_odrive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuchen Lin',
    maintainer_email='y29lin@uwaterloo.ca',
    description='Odrive Motor Controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_controller_node = py_odrive.main:main',
        ],
    },
)
