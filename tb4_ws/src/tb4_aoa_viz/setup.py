from setuptools import setup

package_name = 'tb4_aoa_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['tb4_aoa_viz/aoa_marker.launch.py',
                                   'tb4_aoa_viz/aoa_tx2rx.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='AOA visualization markers in RViz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aoa_marker_node = tb4_aoa_viz.aoa_marker_node:main',
            'aoa_tx2rx_node = tb4_aoa_viz.aoa_tx2rx_node:main',
        ],
    },
)
