from setuptools import find_packages, setup

package_name = 'ltts_vision'

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
    maintainer='o_oji',
    maintainer_email='svsuresh.embedded@gmail.com',
    description='Color Detection Package',
    license='MIT License',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = ltts_vision.color_detector:main',
        ],
    },
)
