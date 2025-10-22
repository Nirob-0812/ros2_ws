from setuptools import find_packages, setup

package_name = 'house_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/slam_mapping.launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirob0812',
    maintainer_email='nirob0812@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
