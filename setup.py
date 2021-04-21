from setuptools import setup

package_name = 'visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drew',
    maintainer_email='drew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'talker = visualizer.publisher_member_function:main',
            'vis = visualizer.vis_subscriber:main',
            'PT_vis = visualizer.PT_visualizer:main',
            'VD_vis = visualizer.VD_visualizer:main',
            'AS_vis = visualizer.AS_visualizer:main',            
        ],
    },
)
