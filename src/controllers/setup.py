from setuptools import find_packages, setup

package_name = 'controllers'

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
    maintainer='labpc1',
    maintainer_email='labpc1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tfbl_controller = controllers.tfbl_controller_node:main",
            "fblqp_controller = controllers.fblqp_controller_node:main",
            "fblqp_controller_PP = controllers.fblqp_controller_PP_node:main",
            "dfl_qp_tracking = controllers.dfl_qp_tracking_node:main",
            "std_dfl_tracking = controllers.std_dfl_tracking_node:main"
        ],
    },
)
