from setuptools import setup

package_name = 'opcua_client'

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
    maintainer='rosi',
    maintainer_email='sshaba9406@outlook.com',
    description='OPCUA Client Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
        'console_scripts': [
                'opcua_client = opcua_client.opcua_client_node:main',
        ],
},
)
