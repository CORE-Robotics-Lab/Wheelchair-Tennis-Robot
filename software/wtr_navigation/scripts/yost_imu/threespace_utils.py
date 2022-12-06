#!/usr/bin/env python2.7

""" This module is a utility module used in the ThreeSpace API.
    
    The ThreeSpace Utils module is a collection of functions, structures, and
    static variables to be use exclusivly with the ThreeSpace API module to find
    available ThreeSpace devices on the host system and information on them.
    This module can be used with a system running Python 2.5 and newer
    (including Python 3.x).
"""

__authors__ = [
    '"Chris George" <cgeorge@yeitechnology.com>',
    '"Dan Morrison" <dmorrison@yeitechnology.com>',
]

import os
import sys
import subprocess
import collections
import serial
import multiprocessing
import time

### Globals ###
TSS_FIND_BTL =          0x00000001
TSS_FIND_USB =          0x00000002
TSS_FIND_DNG =          0x00000004
TSS_FIND_WL =           0x00000008
TSS_FIND_EM =           0x00000010
TSS_FIND_DL =           0x00000020
TSS_FIND_BT =           0x00000040
TSS_FIND_LX =           0x00000080
TSS_FIND_MBT =          0x00000100
TSS_FIND_MWL =          0x00000200
TSS_FIND_NANO =         0x00000400
TSS_FIND_UNKNOWN =      0x80000000
TSS_FIND_ALL_KNOWN =    0x7fffffff
TSS_FIND_ALL =          0xffffffff

### Private ###
__version_firmware = (
    time.strptime("01Jan2000", "%d%b%Y"),
    time.strptime("25Apr2013", "%d%b%Y"),
    time.strptime("21Jun2013", "%d%b%Y"),
    time.strptime("08Aug2013", "%d%b%Y")
)

### Structures ###
ComInfo = collections.namedtuple(
    'ComInfo', (
        'com_port',
        'friendly_name',
        'dev_type'
    )
)

SensorInfo = collections.namedtuple(
    'SensorInfo', (
        'friendly_name',
        'dev_type',
        'dev_serial',
        'dev_fw_ver',
        'dev_hw_ver',
        'in_bootloader'
    )
)

ComPortListing = collections.namedtuple(
    'ComPortListing', (
        'known_ports',
        'unknown_ports'
    )
)

### Functions ###
if sys.version_info >= (3, 0):
    def convertString(string):
        return string.decode('utf-8')
else:
    def convertString(string):
        return string


def pyTryPort(port_name, conn):
    try:
        tmp_port = serial.Serial(port_name, timeout=0.2, writeTimeout=0.2, baudrate=115200)
    except:
        conn.send(False)
        return
    tmp_port.close()
    conn.send(True)


def tryPort(port_name, use_subprocess=False):
    if use_subprocess:
        ## Subprocess version of tryport
        # TryPort attempts to connect to the port repeatedly and releases it
        # order to make it available to the python script
        # in (yucky CDCC driver work around)
        startup_info = None
        program_name = "\\try_port\\try_port.exe"
        file_path = os.path.abspath(__file__)
        if os.name == 'nt':
            startup_info = subprocess.STARTUPINFO()
            startup_info.dwFlags |= subprocess._subprocess.STARTF_USESHOWWINDOW
        
        try:
            # We assume the directory resolved by __file__ will
            # get us the directory for try_port.exe
            last_slash_idx = file_path.rfind("\\")
            try_ports_path = (file_path[:last_slash_idx] + program_name)
            call_success = subprocess.call([try_ports_path, port_name], startupinfo=startup_info)
        
        except Exception as ex:
            if global_file_path is None:
                print(ex)
                return None
            try:
                # We will try to use the cached current working directory
                # instead for try_port.exe
                try_ports_path = (global_file_path + program_name)
                call_success = subprocess.call([try_ports_path, port_name], startupinfo=startup_info)
            except Exception as ex:
                print(ex)
                return None
        
        if call_success != 0:
           return None

    else:
        ## Multiprocessing version of tryport
        parent_conn, child_conn = multiprocessing.Pipe()
        tmp_process = multiprocessing.Process(target=pyTryPort, args=(port_name, child_conn))
        tmp_process.start()
        make_port = parent_conn.recv()
        tmp_process.join()
        if not make_port:
            return None
    return True


def checkSoftwareVersionFromPort(serial_port):
    # Figure out whether the current hardware is on "old" or "new" firmware
    compatibility = 0
    serial_port.write(bytearray((0xf7, 0xdf, 0xdf)))
    response = convertString(serial_port.read(9))
    
    if len(response) == 0:
        # Very Old firmware version
        raise Exception("Either device on( %s ) is not a 3-Space Sensor or the firmware is out of date for this API and recommend updating to latest firmware." % serial_port.name)
    elif response[:3] == b"TSS":
        # Old firmware version remainder
        serial_port.read(9)
        raise Exception("Firmware for device on ( %s ) is out of date for this API. Recommend updating to latest firmware." % serial_port.name)
    else:
        # Hour-minute remainder
        serial_port.read(3)
        
        sensor_firmware = time.strptime(response, "%d%b%Y")
        
        for i in reversed(range(len(__version_firmware))):
            if sensor_firmware >= __version_firmware[i]:
                compatibility = i
                break
    if compatibility == 0:
        raise Exception("Firmware for device on ( %s ) is out of date for this API. Recommend updating to latest firmware." % serial_port.name)
    return compatibility