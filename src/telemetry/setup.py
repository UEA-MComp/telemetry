from setuptools import setup

package_name = 'telemetry'
serversidelibs_path = "../../../server-dockerfiles/server-side"

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [serversidelibs_path+'/database.py', serversidelibs_path+'/models.py']),
    ],
    package_data={'': ['*.conf']},
    include_package_data=True,
    install_requires=['setuptools', "python-dotenv", "pynmeagps", "PyMySQL"],
    zip_safe=True,
    maintainer='Eden Attenborough',
    maintainer_email='gae19jtu@uea.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry = telemetry.telemetry:main',
            'nmea_logger = telemetry.nmea_logger:main'
        ],
    },
)
