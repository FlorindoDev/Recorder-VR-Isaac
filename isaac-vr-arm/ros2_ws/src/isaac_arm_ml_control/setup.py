from setuptools import find_packages, setup

package_name = 'isaac_arm_ml_control'

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
    maintainer='francesco',
    maintainer_email='francesco@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    include_package_data=True,  # necessario
    package_data={
        # dice a setuptools di includere model_weights.pth
        'isaac_arm_ml_control': ['model_weights.pth'],
    },
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_test=isaac_arm_ml_control.publisher_test:main',
            'publisher_ml=isaac_arm_ml_control.panda_mlnode:main',
            'iiwa_pub_ml=isaac_arm_ml_control.iiwa_mlnode:main',
            'vrdatapub=isaac_arm_ml_control.vrdatapub:main',
            'realtimepub=isaac_arm_ml_control.realtimepub:main',
            'realtimepub_claude=isaac_arm_ml_control.realtimepub_claude:main',
            'manual_joint_command=isaac_arm_ml_control.manual_joint_command:main'
        ],
    },
)
