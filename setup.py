from setuptools import setup,find_packages
import os

package_name = 'commanded_stt'
curr_dir = os.path.abspath(os.path.dirname(__file__))
wakeword_dir = os.path.join(curr_dir,'wake_word_engines')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,["wake_word_engines/{}".format(item) for item in os.listdir(wakeword_dir)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='dporfirio@wisc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener_node = commanded_stt.listener_node:main'
        ],
    },
)
