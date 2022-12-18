from setuptools import setup, find_packages

# To use a consistent encoding
from codecs import open
from os import path

# The directory containing this file
HERE = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(HERE, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# This call to setup() does all the work
setup(
    name="rachelspurepursuit",
    version="0.0.1",
    description="Vehicle Dynamics Simulation - Pure Pursuit",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://www.youtube.com/watch?v=4IO1zxsY4wU/",
    author="Rachel",
    author_email="shtainrc@gmail.com",
    license="MIT",
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Operating System :: OS Independent"
    ],
    packages=find_packages,
    include_package_data=True,
    install_requires=["numpy","pandas","matplotlib"]
)
