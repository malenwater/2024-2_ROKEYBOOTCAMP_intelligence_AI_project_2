from setuptools import find_packages, setup
import glob
import os
package_name = 'military_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob(os.path.join('resource', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunwolee',
    maintainer_email='128200788+malenwater@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'AMRmainServer = military_ai.detect_sprint.CentralAMRControllerServerClass:main',
            'PCmainServer = military_ai.User_UI.CentralPCControllerServerClass:main',
            'MoveArg = military_ai.AMR_sprint.FindRCImgNode:main',
            'Movecmd = military_ai.AMR_sprint.tur_cmd_vel:main',
            'Nav = military_ai.AMR_sprint.nav:main',
        ],
    },
)
