from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pac_mouse_pkg'

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
    ]

for root, dirs, files in os.walk('models'):
    if files:
        # Where to put it in the install folder
        install_dir = os.path.join('share', package_name, root)
        # Where it is now
        source_files = [os.path.join(root, f) for f in files]
        # Add to the list
        data_files.append((install_dir, source_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giginu',
    maintainer_email='damian.cutajar@gmail.com',
    description='Autonomous Pac-Mouse simulation with Doraemon Cat and Cheese collection.',
    license='Apache-2.0',
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
            'cat_brain = pac_mouse_pkg.cat_brain:main',
            'teleop = pac_mouse_pkg.teleop:main',
        ],
    },
)
