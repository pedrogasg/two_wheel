from setuptools import setup

package_name = 'two_wheel_python'

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
    maintainer='rbs',
    maintainer_email='pedrogas_g@hotmail.com',
    description='Two wheel learning robot',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'two_wheel_python = two_wheel_python.two_wheel_python:main'
        ],
    },
)
