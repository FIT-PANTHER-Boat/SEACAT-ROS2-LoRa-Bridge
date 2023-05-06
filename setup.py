from setuptools import setup

package_name = 'ros2_lora'

setup(
	name=package_name,
	version='0.0.1',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages', ['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	author='hikari',
	author_email='asaad2022@my.fit.edu',
	keywords=['ROS2', 'LoRa'],
	classifiers=[
		'Intended Audience :: Developers',
		'License :: OSI Approved :: Apache Sofware License',
		'Programming Language :: Python',
		'Programming Language :: Python :: 3',
		'Programming Language :: Python :: 3.6',
		'Programming Language :: Python :: 3.7',
		'Programming Language :: Python :: 3.8',
	],
	description='ROS2 package for LoRa communication.',
	license='Apache License 2.0',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'ros2_lora = ros2_lora.ros2_lora:main',
		],
	},

)
