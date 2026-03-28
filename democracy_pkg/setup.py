from setuptools import find_packages, setup

package_name = 'democracy_pkg'

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
    maintainer='byte_knight',
    maintainer_email='rsolyzia@outlook.com',
    description='Bipedal robot package',
    license='MIT Liscence',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'activate = democracy_pkg.activate:main',
            'voice_service = democracy_pkg.voice_service:main',
            'voice_recog = democracy_pkg.voice_recog_sys:main',
            'servos = democracy_pkg.servos_sys:main',
            'imu = democracy_pkg.imu:main',
            'stepper = democracy_pkg.stepper_sys:main',
        ],
    },
)
