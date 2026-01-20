from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pac_mouse_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Install URDF/Xacro Files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # 3. Install World Files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        
        # 4. Install Rviz Files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # 5. Install Config Files (ADD THIS LINE HERE!)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giginu',
    maintainer_email='damian.cutajar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorer = pac_mouse_pkg.explorer:main',
            'smart_mouse = pac_mouse_pkg.smart_mouse:main',
            'smart_mouse_hew = pac_mouse_pkg.smart_mouse_hew:main',
        ],
    },
)
