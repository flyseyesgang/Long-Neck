# from setuptools import find_packages, setup

# package_name = 'drink_memory'

# setup(
#     name=package_name,
#     version='0.1.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=[
#         'setuptools',
#         'roboticstoolbox-python',
#         'spatialmath-python',
#     ],
#     zip_safe=True,
#     maintainer='parallels',
#     maintainer_email='jorian.p.mitchell@outlook.com',
#     description='Decoupled search logic and UR3 motion nodes for drink retrieval',
#     license='Apache-2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             # Runs the MemoryDecisionNode (search logic)
#             'memory_decision = drink_memory.drink_memory_node:main_memory',
#             # Runs the UR3TrajectoryPublisher (motion node)
#             'ur3_trajectory  = drink_memory.drink_memory_node:main_ur3',
#         ],
#     },
# )


from setuptools import find_packages, setup

package_name = 'drink_memory'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],  # no more roboticstoolbox or spatialmath here
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='jorian.p.mitchell@outlook.com',
    description='Search logic node for drink retrieval with pseudo-move stub',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'memory_decision = drink_memory.drink_memory_node:main',
        ],
    },
)

# from setuptools import find_packages, setup

# package_name = 'drink_memory'

# setup(
#     name=package_name,
#     version='0.1.0',  # bumped to semantic versioning
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         # we’ve removed the ament_index resource file entirely,
#         # since it’s not needed for this Python‐only package
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='parallels',
#     maintainer_email='jorian.p.mitchell@outlook.com',
#     description='Decoupled search logic and UR3 motion nodes for drink retrieval',
#     license='Apache-2.0',  # specify a real OSI-approved license
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             # Launch the MemoryDecisionNode:
#             # ros2 run drink_memory memory_decision
#             'memory_decision = drink_memory_node:main',
#             # Launch the UR3TrajectoryPublisher:
#             # ros2 run drink_memory ur3_trajectory
#             'ur3_trajectory  = drink_memory_node:main',
#         ],
#     },
# )


# from setuptools import find_packages, setup

# package_name = 'drink_memory'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         # ('share/ament_index/resource_index/packages',
#         #     ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='parallels',
#     maintainer_email='jorian.p.mitchell@outlook.com',
#     description='Memory / decision logic for drink‑retrieval robot',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'drink_memory_node = drink_memory.drink_memory_node:main'
#         ],
#     },
# )
