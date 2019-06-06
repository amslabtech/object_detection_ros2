from setuptools import setup
 
package_name = 'control'
 
setup(
    name=package_name,
    version='0.0.0',
    packages=['object_detection','yolo3'],
    package_dir={'yolo3': 'object_detection/yolo3'},
    py_modules=[
        'image_publisher',
        'object_detection_publisher',
        'agent'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='user',
    author_email="user@todo.todo",
    maintainer='user',
    maintainer_email="user@todo.todo",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = agent:main',
            'image_publisher = image_publisher:main',
            'object_detection_publisher = object_detection_publisher:main',
        ],
    },
)
