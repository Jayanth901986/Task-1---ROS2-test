from setuptools import find_packages, setup

package_name = 'image_conversion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/image_conversion_with_dummy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayanth',
    maintainer_email='jk0721585@gmail.com',
    description='ROS2 node for converting camera images to grayscale or color',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_conversion = image_conversion.image_conversion:main'
        ],
    },
)

