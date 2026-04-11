from setuptools import find_packages, setup

package_name = 'conveyor_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamedhussien',
    maintainer_email='mohamedhussien@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'perception_node = conveyor_tracking.perception_node:main',
        'conductor_node = conveyor_tracking.conductor_node:main'
    ],
},
)
