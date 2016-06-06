#!/usr/bin/env python
#
# Author: Logan Gunthorpe <logang@deltatee.com>
# Copyright (c) Deltatee Enterprises Ltd. 2015, All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.

# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library.

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(name='pyft232',
      version='0.5',
      description="""Python bindings to d2xx and libftdi to access FT232 chips with
                     the same interface as pyserial. Using this method gives easy access
                     to the additional features on the chip like CBUS GPIO.""",
      long_description=open('README.md', 'rt').read(),
      author='Logan Gunthorpe',
      author_email='logang@deltatee.com',
      packages=['ft232'],
      install_requires=[
        'pyusb >= 0.4',
        'pyserial >= 2.5',
       ]

     )
