import os
import setuptools

# For version number interpretation see: https://semver.org
VERSION = '1.0.0'

with open('requirements.txt') as f:
    required = f.read().splitlines()
    setuptools.setup(name='FrontierExploration',
                    version=VERSION,
                    description='',
                    url='',
                    author='',
                    author_email='',
                    license='Private',
                    packages=setuptools.find_packages(where="src"),
                    package_dir={"": "src"},
                    entry_points={
                        'console_scripts': ['fesh=helper.main:app'],
                    },
                    include_package_data=True,
                    zip_safe=False,
                    install_requires=required)