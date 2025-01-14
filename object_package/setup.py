from setuptools import find_packages, setup


package_name = 'object_package'

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
    maintainer='mitsu',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'camera_listener = my_package.camera_listener:main',
            #'image_display = my_package.image_display:main',
            'object_detection_node = object_package.object_detection_node:main',
            
        ],
    },
)

