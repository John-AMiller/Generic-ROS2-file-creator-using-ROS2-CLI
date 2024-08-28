from setuptools import setup

package_name = 'cli_plugin'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_node = cli_plugin.create_node:main',
        ],
        'ros2cli.command': [
            'create = my_cli_plugin.command.create:CreateCommand',
        ],
    },
)
