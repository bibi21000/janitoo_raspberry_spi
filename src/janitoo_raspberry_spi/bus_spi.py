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

import Adafruit_GPIO as GPIO
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

OID = 'rpispi'

class SPIBus(JNTBus):
    """A pseudo-bus to handle the Raspberry SPI Bus
    """

    def __init__(self, **kwargs):
        """
        :param int bus_id: the SMBus id (see Raspberry Pi documentation)
        :param kwargs: parameters transmitted to :py:class:`smbus.SMBus` initializer
        """
        JNTBus.__init__(self, **kwargs)
        self._spi_lock = threading.Lock()
        try:
            self._ada_spi = SPI
            self._ada_gpio = GPIO.get_platform_gpio()
        except :
            logger.exception("[%s] - Can't get GPIO", self.__class__.__name__)
        self.load_extensions(self.oid)
        self.export_attrs('_ada_spi', self._ada_spi)
        self.export_attrs('_ada_gpio', self._ada_spi)
        self.export_attrs('spi_acquire', self.spi_acquire)
        self.export_attrs('spi_release', self.spi_release)
        self.export_attrs('get_spi_device', self.get_spi_device)
        self.export_attrs('get_spi_device_pin', self.get_spi_device_pin)

    def spi_acquire(self, blocking=True):
        """Get a lock on the bus"""
        self._spi_lock.acquire(blocking)

    def spi_release(self):
        """Release a lock on the bus"""
        self._spi_lock.release()

    def get_spi_device(self, num):
        """Return a device to use with adafruit bus"""
        raise RuntimeError("Must be overloaded by descendant")

    def get_spi_device_pin(self, num):
        """Return the CS pin corresponding to an hardawre SPI device"""
        if num==0:
            #map spi_device to pin number. On a pi2 0 ->18
            dc_pin = 18
        elif num==1:
            #map spi_device to pin number. On a pi2 1 ->17 ?
            dc_pin = 17
        else:
            dc_pin = num

def extend_hardware( self ):
    #You must choose either software or hardware bus
    uuid="%s_port"%OID
    self.values[uuid] = self.value_factory['config_byte'](options=self.options, uuid=uuid,
        node_uuid=self.uuid,
        help='The SPI hardware port',
        label='port',
        default=0,
    )

    self._spih_start = self.start
    def start(mqttc, trigger_thread_reload_cb=None):
        """Start the bus"""
        logger.debug("[%s] - Start the bus %s", self.__class__.__name__, self.oid )
        self.spi_acquire()
        try:
            os.system('modprobe spi-bcm2835')
        except :
            logger.exception("[%s] - Can't load spi-* kernel modules", self.__class__.__name__)
        finally:
            self.spi_release()
        return self._spih_start(mqttc, trigger_thread_reload_cb=trigger_thread_reload_cb)
    self.start = start

    def get_spi_device(device, max_speed_hz=4000000):
        """Return a device to use with adafruit bus"""
        self.spi_acquire()
        try:
            return SPI.SpiDev(self.values["%s_port"%OID].data, device, max_speed_hz=max_speed_hz)
        except:
            logger.exception('[%s] - Exception when getting device', self.__class__.__name__)
        finally:
            self.spi_release()
    self.get_spi_device = get_spi_device

def extend_software( self ):
    #You must choose either software or hardware bus
    uuid="%s_pin_mosi"%OID
    self.values[uuid] = self.value_factory['config_byte'](options=self.options, uuid=uuid,
        node_uuid=self.uuid,
        help='The SPI MOSI pin',
        label='pin_mosi',
        default=23,
    )
    uuid="%s_pin_miso"%OID
    self.values[uuid] = self.value_factory['config_byte'](options=self.options, uuid=uuid,
        node_uuid=self.uuid,
        help='The SPI MISO pin',
        label='pin_miso',
        default=24,
    )
    uuid="%s_pin_clk"%OID
    self.values[uuid] = self.value_factory['config_byte'](options=self.options, uuid=uuid,
        node_uuid=self.uuid,
        help='The SPI CLK pin',
        label='pin_clk',
        default=25,
    )

    def get_spi_device(device, max_speed_hz=4000000):
        """Return a device to use with adafruit bus"""
        self.spi_acquire()
        try:
            return SPI.BitBang(self._ada_gpio, self.values["%s_pin_clk"%OID].data,
                    self.values["%s_pin_mosi"%OID].data,
                    self.values["%s_pin_miso"%OID].data)
        except:
            logger.exception('[%s] - Exception when getting device', self.__class__.__name__)
        finally:
            self.spi_release()
    self.get_spi_device = get_spi_device
