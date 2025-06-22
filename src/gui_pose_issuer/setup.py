from setuptools import find_packages, setup

package_name = 'gui_pose_issuer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui_pose_issuer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam.scheele@outlook.com',
    description='GUI interface for commanding robot poses by clicking on a clock',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_pose_issuer_node = gui_pose_issuer.gui_pose_issuer_node:main',
        ],
    },
)
