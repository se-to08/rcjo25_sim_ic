from setuptools import find_packages, setup

package_name = 'rcjo25_sim_interactivecleanup'
submodules = 'rcjo25_sim_interactivecleanup/element' #追加

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
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'direction_of_face = rcjo25_sim_interactivecleanup.element.direction_of_face:main',
            'person_position_tracker = rcjo25_sim_interactivecleanup.element.person_position_tracker:main',
            'person_centering_controller = rcjo25_sim_interactivecleanup.element.person_centering_controller:main',
            'pointing_direction_estimator = rcjo25_sim_interactivecleanup.element.pointing_direction_estimator:main',
            'tf_listener = rcjo25_sim_interactivecleanup.element.tf_listener:main',
            'tf_listener2 = rcjo25_sim_interactivecleanup.element.tf_listener2:main',
            'tf_listener2_for_debug = rcjo25_sim_interactivecleanup.element.tf_listener2_for_debug:main',
            'person_region_estimator = rcjo25_sim_interactivecleanup.element.person_region_estimator:main',
            'task_common_send_ready2 = rcjo25_sim_interactivecleanup.element.task_common_send_ready2:main',
            'message_receive_for_avatar = rcjo25_sim_interactivecleanup.element.message_receive_for_avatar:main',
            'receive = rcjo25_sim_interactivecleanup.receive:main'
        ],
    },
)
