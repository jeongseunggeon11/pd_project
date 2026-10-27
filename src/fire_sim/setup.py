from glob import glob

from setuptools import setup

package_name = 'fire_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='blueciki@gmail.com',
    description='Fire model, fire generator and fire manager for the simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fire_generator = fire_sim.fire_generator:main',
            'fire_manager = fire_sim.fire_manager:main',
        ],
    },
)
