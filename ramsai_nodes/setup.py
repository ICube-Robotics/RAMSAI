from setuptools import find_packages, setup

package_name = 'ramsai_nodes'

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
    maintainer='iris',
    maintainer_email='laurent.barbe@unistra.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viscotec_velocity_observer = ramsai_nodes.viscotec_velocity_observer:main',
            'fake_vm_publisher = ramsai_nodes.fake_vm_publisher:main',
        ],
    },
)
