from setuptools import setup

package_name = 'telemetry'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/rtklib.py']),
    ],
    package_data={'': ['*.conf']},
    include_package_data=True,
    install_requires=['setuptools', "python-dotenv"],
    zip_safe=True,
    maintainer='Eden Attenborough',
    maintainer_email='gae19jtu@uea.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtkrcv = telemetry.rtkrcv:main'
        ],
    },
)
