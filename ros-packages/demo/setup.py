from setuptools import setup
import os
from glob import glob
package_name = 'demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjw',
    maintainer_email='kojowelbeck2021@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_client = demo.action_client:main',
            'client_manager = demo.client_manager:main',
            'error_handler = demo.error_handler:main',
            'visual_tool = demo.visual_tool:main'


        ],
    },
)
