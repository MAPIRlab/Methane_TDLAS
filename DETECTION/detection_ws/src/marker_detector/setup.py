from setuptools import find_packages, setup
from setuptools.command.install import install
import subprocess
import os
import stat

package_name = 'marker_detector'


class CustomInstall(install):
    def run(self):
        # Install the wheel first
        subprocess.check_call(['pip', 'install', './vmbpy-1.1.0-py3-none-linux_x86_64.whl'])
        
        # Run the standard install
        install.run(self)
        
        # Make the .so files executable
        install_dir = os.path.join(self.install_lib, 'marker_detector', 'scripts')
        for lib in ['libslabhiddevice.so.1.0', 'libslabhidtosmbus.so.1.0']:
            lib_path = os.path.join(install_dir, lib)
            if os.path.exists(lib_path):
                st = os.stat(lib_path)
                os.chmod(lib_path, st.st_mode | stat.S_IEXEC)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/marker_detector/scripts', [
            'marker_detector/scripts/libslabhiddevice.so.1.0',
            'marker_detector/scripts/libslabhidtosmbus.so.1.0'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    include_package_data=True, 
    package_data={
        'marker_detector': [
            'scripts/*',
            'scripts/*.so*',
            'scripts/*.dll',
            'scripts/libslabhiddevice.so.1.0',
            'scripts/libslabhidtosmbus.so.1.0',
            'scripts/SLABHIDtoSMBus.dll',
            'scripts/SLABHIDDevice.dll'
        ]
    },
    maintainer='aromsan',
    maintainer_email='alerosan16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'marker_detector = marker_detector.camera_node:main'
        ],
    },
    cmdclass={
    	'install': CustomInstall,
    }
)
