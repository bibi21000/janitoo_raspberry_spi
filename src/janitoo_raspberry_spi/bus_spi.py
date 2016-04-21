# -*- coding: utf-8 -*-
"""The Raspberry i2c bus

"""

__license__ = """
    This file is part of Janitoo.

    Janitoo is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Janitoo is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Janitoo. If not, see <http://www.gnu.org/licenses/>.

"""
__author__ = 'Sébastien GALLET aka bibi21000'
__email__ = 'bibi21000@gmail.com'
__copyright__ = "Copyright © 2013-2014-2015-2016 Sébastien GALLET aka bibi21000"

# Set default logging handler to avoid "No handler found" warnings.
import logging
logger = logging.getLogger(__name__)
import os, sys
import threading
import time
import datetime
import socket

from janitoo.thread import JNTBusThread
from janitoo.bus import JNTBus
from janitoo.component import JNTComponent
from janitoo.thread import BaseThread
from janitoo.options import get_option_autostart

import Adafruit_GPIO.SPI as SPI

##############################################################
#Check that we are in sync with the official command classes
#Must be implemented for non-regression
from janitoo.classes import COMMAND_DESC

COMMAND_CAMERA_PREVIEW = 0x2200
COMMAND_CAMERA_PHOTO = 0x2201
COMMAND_CAMERA_VIDEO = 0x2202
COMMAND_CAMERA_STREAM = 0x2203

assert(COMMAND_DESC[COMMAND_CAMERA_PREVIEW] == 'COMMAND_CAMERA_PREVIEW')
assert(COMMAND_DESC[COMMAND_CAMERA_PHOTO] == 'COMMAND_CAMERA_PHOTO')
assert(COMMAND_DESC[COMMAND_CAMERA_VIDEO] == 'COMMAND_CAMERA_VIDEO')
assert(COMMAND_DESC[COMMAND_CAMERA_STREAM] == 'COMMAND_CAMERA_STREAM')
##############################################################

class SPIBus(JNTBus):
    """A pseudo-bus to handle the Raspberry SPI Bus
    """

    def __init__(self, **kwargs):
        """
        :param int bus_id: the SMBus id (see Raspberry Pi documentation)
        :param kwargs: parameters transmitted to :py:class:`smbus.SMBus` initializer
        """
        try:
            os.system('modprobe spi-bcm2708')
        except :
            log.exception("[%s] - Can't load spi-* kernel modules", self.__class__.__name__)
        JNTBus.__init__(self, **kwargs)
        self._spi_lock = threading.Lock()
        self._ada_spi = SPI
        """ The shared ADAFruit SPI bus """
        self.load_extensions(self.oid)
        self.export_attrs('spi_acquire', self.spi_acquire)
        self.export_attrs('spi_release', self.spi_release)

    def spi_acquire(self):
        """Get a lock on the bus"""
        self._spi_lock.acquire()

    def spi_release(self):
        """Release a lock on the bus"""
        self._spi_lock.release()
