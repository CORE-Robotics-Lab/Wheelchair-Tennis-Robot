#!/usr/bin/env python3
from __future__ import print_function

""" This module is an API module for ThreeSpace devices.
    
    The ThreeSpace API module is a collection of classes, functions, structures,
    and static variables use exclusivly for ThreeSpace devices. This module can
    be used with a system running Python 2.5 and newer (including Python 3.x).
"""

__version__ = "2.0.2.3"

__authors__ = [
    '"Chris George" <cgeorge@yeitechnology.com>',
    '"Dan Morrison" <dmorrison@yeitechnology.com>',
]

import threading
import sys
import serial
import struct
import collections
import traceback
import time
import os

# chose an implementation, depending on os
if os.name == 'nt':  # sys.platform == 'win32':
    from win32_threespace_utils import *
else:
    from threespace_utils import *

    print("WARNING: No additional utils are loaded!!!!!!")

### Globals ###
global_file_path = os.getcwd()
global_error = None
global_counter = 0
global_donglist = {}
global_sensorlist = {}
global_broadcaster = None

TSS_TIMESTAMP_SENSOR = 0
TSS_TIMESTAMP_SYSTEM = 1
TSS_TIMESTAMP_NONE = 2

TSS_JOYSTICK = 0
TSS_MOUSE = 2

TSS_BUTTON_LEFT = 0
TSS_BUTTON_RIGHT = 1

### Private ###
_baudrate = 115200
_allowed_baudrates = [1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200, 230400, 460800, 921600]
_wireless_retries = 5

### Functions ###
if sys.version_info >= (3, 0):
    def makeWriteArray(startbyte, index_byte=None, command_byte=None, data=None):
        rtn_array = bytearray((startbyte,))
        if index_byte is not None:
            rtn_array.append(index_byte)
        if command_byte is not None:
            rtn_array.append(command_byte)
        if data is not None:
            rtn_array += data
        rtn_array.append((sum(rtn_array) - startbyte) % 256)  # checksum
        _hexDump(rtn_array)
        return rtn_array
else:
    def makeWriteArray(startbyte, index_byte=None, command_byte=None, data=None):
        rtn_array = chr(startbyte)
        if index_byte is not None:
            rtn_array += chr(index_byte)
        if command_byte is not None:
            rtn_array += chr(command_byte)
        if data is not None:
            rtn_array += data
        rtn_array += chr((sum(bytearray(rtn_array)) - startbyte) % 256)  # checksum
        _hexDump(rtn_array)
        return rtn_array


def _hexDump(serial_string, header='i'):
    if "-d_hex" in sys.argv:
        ba = bytearray(serial_string)
        print('{0}('.format(header), end='')
        for i in range(len(ba)):
            if i == len(ba) - 1:
                print('0x{0:02x}'.format(ba[i]), end='')
            else:
                print('0x{0:02x},'.format(ba[i]), end='')
        print(')')


def _print(string):
    if "-d" in sys.argv:
        print(string)


def _echoCallback(sensor, state):
    _print('{0}:{1}'.format(sensor, state))


def _generateProtocolHeader(success_failure=False,
                            timestamp=False,
                            command_echo=False,
                            checksum=False,
                            logical_id=False,
                            serial_number=False,
                            data_length=False):
    byte = 0
    struct_str = '>'
    idx_list = []
    if success_failure:
        byte += 0x1
        struct_str += '?'
        idx_list.append(0)
    if timestamp:
        byte += 0x2
        struct_str += 'I'
        idx_list.append(1)
    if command_echo:
        byte += 0x4
        struct_str += 'B'
        idx_list.append(2)
    if checksum:
        byte += 0x8
        struct_str += 'B'
        idx_list.append(3)
    if logical_id:
        byte += 0x10
        struct_str += 'B'
        idx_list.append(4)
    if serial_number:
        byte += 0x20
        struct_str += 'I'
        idx_list.append(5)
    if data_length:
        byte += 0x40
        struct_str += 'B'
        idx_list.append(6)
    return (byte, struct.Struct(struct_str), idx_list)


def _generateSensorClass(sensor_inst, serial_port, allowed_device_types):
    sensor_inst.compatibility = checkSoftwareVersionFromPort(serial_port)
    sensor_inst.port_name = serial_port.name
    sensor_inst.serial_port_settings = serial_port.getSettingsDict()
    sensor_inst.serial_port = serial_port

    hardware_version = convertString(sensor_inst.f7WriteRead('getHardwareVersionString'))
    dev_type = hardware_version[4:-8].strip()
    if dev_type not in allowed_device_types:
        raise Exception("This is a %s device, not one of these devices %s!" % (dev_type, allowed_device_types))

    sensor_inst.device_type = dev_type

    serial_number = sensor_inst.f7WriteRead('getSerialNumber')
    sensor_inst.serial_number = serial_number

    if dev_type == "DNG":
        if serial_number in global_donglist:
            rtn_inst = global_donglist[serial_number]
            rtn_inst.close()
            rtn_inst.compatibility = sensor_inst.compatibility
            rtn_inst.port_name = serial_port.name
            rtn_inst.serial_port_settings = serial_port.getSettingsDict()
            rtn_inst.serial_port = serial_port
            return rtn_inst

        global_donglist[serial_number] = sensor_inst
    else:
        if serial_number in global_sensorlist:
            rtn_inst = global_sensorlist[serial_number]
            rtn_inst.close()
            rtn_inst.compatibility = sensor_inst.compatibility
            rtn_inst.port_name = serial_port.name
            rtn_inst.serial_port_settings = serial_port.getSettingsDict()
            rtn_inst.serial_port = serial_port
            if "BT" in dev_type:
                rtn_inst.serial_port.timeout = 1.5
                rtn_inst.serial_port.writeTimeout = 1.5
            if "WL" in dev_type:
                rtn_inst.switchToWiredMode()
            return rtn_inst

        if "BT" in dev_type:
            sensor_inst.serial_port.timeout = 1.5
            sensor_inst.serial_port.writeTimeout = 1.5
        elif "WL" in dev_type:
            sensor_inst.switchToWiredMode()

        global_sensorlist[serial_number] = sensor_inst

    return sensor_inst


def parseAxisDirections(axis_byte):
    axis_order_num = axis_byte & 7
    if axis_order_num == 0:
        axis_order = "XYZ"
    elif axis_order_num == 1:
        axis_order = "XZY"
    elif axis_order_num == 2:
        axis_order = "YXZ"
    elif axis_order_num == 3:
        axis_order = "YZX"
    elif axis_order_num == 4:
        axis_order = "ZXY"
    elif axis_order_num == 5:
        axis_order = "ZYX"
    else:
        raise ValueError
    neg_x = neg_y = neg_z = False
    if (axis_byte & 32) > 0:
        neg_x = True
    if (axis_byte & 16) > 0:
        neg_y = True
    if (axis_byte & 8) > 0:
        neg_z = True
    return axis_order, neg_x, neg_y, neg_z


def generateAxisDirections(axis_order, neg_x=False, neg_y=False, neg_z=False):
    axis_order = axis_order.upper()
    if axis_order == "XYZ":
        axis_byte = 0
    elif axis_order == "XZY":
        axis_byte = 1
    elif axis_order == "YXZ":
        axis_byte = 2
    elif axis_order == "YZX":
        axis_byte = 3
    elif axis_order == "ZXY":
        axis_byte = 4
    elif axis_order == "ZYX":
        axis_byte = 5
    else:
        raise ValueError
    if neg_x:
        axis_byte = axis_byte | 32
    if neg_y:
        axis_byte = axis_byte | 16
    if neg_z:
        axis_byte = axis_byte | 8
    return axis_byte


def getSystemWirelessRetries():
    return _wireless_retries


def setSystemWirelessRetries(retries):
    global _wireless_retries
    _wireless_retries = retries


def getDefaultCreateDeviceBaudRate():
    return _baudrate


def setDefaultCreateDeviceBaudRate(new_baudrate):
    global _baudrate
    if new_baudrate in _allowed_baudrates:
        _baudrate = new_baudrate


def padProtocolHeader69(header_data, sys_timestamp):
    fail_byte, cmd_echo, data_size = header_data
    return (fail_byte, sys_timestamp, cmd_echo, None, None, None, data_size)


def padProtocolHeader71(header_data):
    fail_byte, timestamp, cmd_echo, data_size = header_data
    return (fail_byte, timestamp, cmd_echo, None, None, None, data_size)


def padProtocolHeader85(header_data, sys_timestamp):
    fail_byte, cmd_echo, rtn_log_id, data_size = header_data
    return (fail_byte, sys_timestamp, cmd_echo, None, rtn_log_id, None, data_size)


def padProtocolHeader87(header_data):
    fail_byte, timestamp, cmd_echo, rtn_log_id, data_size = header_data
    return (fail_byte, timestamp, cmd_echo, None, rtn_log_id, None, data_size)


### Classes ###
class Broadcaster(object):
    def __init__(self):
        self.retries = 10

    def setRetries(self, retries=10):
        self.retries = retries

    def sequentialWriteRead(self, command, input_list=None, filter=None):
        if filter is None:
            filter = list(global_sensorlist.values())
        val_list = {}
        for i in range(self.retries):
            for sensor in reversed(filter):
                packet = sensor.writeRead(command, input_list)
                if packet[0]:  # fail_byte
                    continue
                val_list[sensor.serial_number] = packet
                filter.remove(sensor)
            if not filter:
                break
            # _print("##Attempt: {0} complete".format(i))
        else:
            # _print("sensor failed to succeed")
            for sensor in filter:
                val_list[sensor.serial_number] = (True, None, None)
        return val_list

    def writeRead(self, command, input_list=None, filter=None):
        q = TSCommandQueue()
        if filter is None:
            filter = list(global_sensorlist.values())
        for sensor in filter:
            q.queueWriteRead(sensor, sensor.serial_number, self.retries, command, input_list)
        return q.proccessQueue()

    def _broadcastMethod(self, filter, method, default=None, *args):
        # _print(filter)
        if filter is None:
            filter = list(global_sensorlist.values())
        val_list = {}
        for i in range(self.retries):
            for sensor in reversed(filter):
                packet = getattr(sensor, method)(*args)
                if packet is default:  # fail_byte
                    continue
                val_list[sensor.serial_number] = packet
                filter.remove(sensor)
            if not filter:
                break
            # _print("##Attempt: {0} complete".format(i))
        else:
            # _print("sensor failed to succeed")
            for sensor in filter:
                val_list[sensor.serial_number] = default
        return val_list

    def broadcastMethod(self, method, default=None, args=[], filter=None, callback_func=None):
        q = TSCommandQueue()
        if filter is None:
            filter = list(global_sensorlist.values())
        for sensor in filter:
            q.queueMethod(getattr(sensor, method),
                          sensor,
                          self.retries,
                          default,
                          args,
                          callback_func)
        return q.proccessQueue()

    def setStreamingSlots(self, slot0='null',
                          slot1='null',
                          slot2='null',
                          slot3='null',
                          slot4='null',
                          slot5='null',
                          slot6='null',
                          slot7='null',
                          filter=None,
                          callback_func=None):
        args = (slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7)
        return self.broadcastMethod('setStreamingSlots', False, args, filter, callback_func)

    def getStreamingSlots(self, filter=None, callback_func=None):
        return self.broadcastMethod('getStreamingSlots', None, [], filter, callback_func)

    def startStreaming(self, record_data=False, filter=None, callback_func=None):
        return self.broadcastMethod('startStreaming', False, [record_data], filter, callback_func)

    def stopStreaming(self, filter=None, callback_func=None):
        return self.broadcastMethod('stopStreaming', False, [], filter, callback_func)

    def setStreamingTiming(self, interval, duration, delay, delay_offset, filter=None, callback_func=None):
        if filter is None:
            filter = list(global_sensorlist.values())
        else:
            filter = list(filter)
        val_list = {}
        for sensor in reversed(filter):
            success = False
            for i in range(self.retries):
                if sensor.setStreamingTiming(interval, duration, delay):
                    if callback_func is not None:
                        callback_func(sensor, True)
                    success = True
                    break
                # _print("##Attempt: {0} complete".format(i))
                if callback_func is not None:
                    callback_func(sensor, False)
            else:
                # _print("sensor failed to succeed")
                pass
            val_list[sensor] = success
            filter.remove(sensor)
            delay += delay_offset
        return val_list

    def startRecordingData(self, filter=None, callback_func=None):
        if filter is None:
            filter = list(global_sensorlist.values())
        for sensor in filter:
            sensor.record_data = True
            if callback_func is not None:
                callback_func(sensor, True)

    def stopRecordingData(self, filter=None, callback_func=None):
        if filter is None:
            filter = list(global_sensorlist.values())
        for sensor in filter:
            sensor.record_data = False
            if callback_func is not None:
                callback_func(sensor, True)

    def debugPrint(self, broadcast_dict):
        for sensor, data in broadcast_dict.items():
            _print('Sensor {0:08X}: {1}'.format(sensor, data))


class TSCommandQueue(object):
    def __init__(self):
        self.queue = []
        self.return_dict = {}

    def queueWriteRead(self, sensor, rtn_key, retries, command, input_list=None):
        self.queue.append(("queueWriteRead", sensor, (self.return_dict, rtn_key, retries, command, input_list)))

    def queueMethod(self, method_obj, rtn_key, retries, default=None, input_list=None, callback_func=None):
        self.queue.append(("queueMethod", (method_obj, rtn_key, retries, default, input_list, callback_func)))

    def _queueMethod(self, method_obj, rtn_key, retries, default=None, input_list=None, callback_func=None):
        try:
            for i in range(retries):
                packet = method_obj(*input_list)
                if packet is default:  # fail_byte
                    if callback_func is not None:
                        callback_func(rtn_key, False)
                    continue
                if callback_func is not None:
                    callback_func(rtn_key, True)
                self.return_dict[rtn_key] = packet
                break

            else:
                self.return_dict[rtn_key] = default
        except(KeyboardInterrupt):
            print('\n! Received keyboard interrupt, quitting threads.\n')
            raise KeyboardInterrupt  # fix bug where a thread eats the interupt

    def createThreads(self):
        thread_queue = []
        for item in self.queue:
            if item[0] == "queueWriteRead":
                thread_queue.append(item[1].queueWriteRead(*item[2]))
            elif item[0] == "queueMethod":
                qThread = threading.Thread(target=self._queueMethod, args=item[1])
                thread_queue.append(qThread)
        return thread_queue

    def proccessQueue(self, clear_queue=False):
        thread_queue = self.createThreads()
        [qThread.start() for qThread in thread_queue]
        [qThread.join() for qThread in thread_queue]
        if clear_queue:
            self.queue = []
        return self.return_dict


# Base class should not be used directly
class _TSBase(object):
    command_dict = {
        'checkLongCommands': (0x19, 1, '>B', 0, None, 1),
        'startStreaming': (0x55, 0, None, 0, None, 1),
        'stopStreaming': (0x56, 0, None, 0, None, 1),
        'updateCurrentTimestamp': (0x5f, 0, None, 4, '>I', 1),
        'setLEDMode': (0xc4, 0, None, 1, '>B', 1),
        'getLEDMode': (0xc8, 1, '>B', 0, None, 1),
        '_setWiredResponseHeaderBitfield': (0xdd, 0, None, 4, '>I', 1),
        '_getWiredResponseHeaderBitfield': (0xde, 4, '>I', 0, None, 1),
        'getFirmwareVersionString': (0xdf, 12, '>12s', 0, None, 1),
        'commitSettings': (0xe1, 0, None, 0, None, 1),
        'softwareReset': (0xe2, 0, None, 0, None, 1),
        'getHardwareVersionString': (0xe6, 32, '>32s', 0, None, 1),
        'getSerialNumber': (0xed, 4, '>I', 0, None, 1),
        'setLEDColor': (0xee, 0, None, 12, '>fff', 1),
        'getLEDColor': (0xef, 12, '>fff', 0, None, 1),
        'setJoystickAndMousePresentRemoved': (0xfd, 0, None, 2, '>BB', 1),
        'getJoystickAndMousePresentRemoved': (0xfe, 2, '>B', 0, None, 1),
        'null': (0xff, 0, None, 0, None, 1)
    }

    def __init__(self, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        self.protocol_args = {'success_failure': True,
                              'timestamp': True,
                              'command_echo': True,
                              'data_length': True}
        if timestamp_mode != TSS_TIMESTAMP_SENSOR:
            self.protocol_args['timestamp'] = False
        self.timestamp_mode = timestamp_mode
        self.baudrate = baudrate
        reinit = False
        try:  # if this is set the class had been there before
            check = self.stream_parse
            reinit = True
            # _print("sensor reinit!!!")
        except:
            self._setupBaseVariables()
        self._setupProtocolHeader(**self.protocol_args)
        self._setupThreadedReadLoop()
        if reinit:
            if self.stream_timing is not None:
                self.setStreamingTiming(*self.stream_timing)
            if self.stream_slot_cmds is not None:
                self.setStreamingSlots(*self.stream_slot_cmds)

    def _setupBaseVariables(self):
        self.serial_number_hex = '{0:08X}'.format(self.serial_number)
        self.stream_timing = None
        self.stream_parse = None
        self.stream_slot_cmds = ['null'] * 8
        self.stream_last_data = None
        self.stream_data = []
        self.record_data = False
        self.data_loop = False

    def _setupProtocolHeader(self, success_failure=False,
                             timestamp=False,
                             command_echo=False,
                             checksum=False,
                             logical_id=False,
                             serial_number=False,
                             data_length=False):
        protocol_header = _generateProtocolHeader(success_failure,
                                                  timestamp,
                                                  command_echo,
                                                  checksum,
                                                  logical_id,
                                                  serial_number,
                                                  data_length)
        protocol_byte, self.header_parse, self.header_idx_lst = protocol_header
        d_header = self.f7WriteRead('_getWiredResponseHeaderBitfield')
        if d_header != protocol_byte:
            self.f7WriteRead('_setWiredResponseHeaderBitfield', protocol_byte)
            d_header = self.f7WriteRead('_getWiredResponseHeaderBitfield')
        if d_header != protocol_byte:
            print("!!!!!fail d_header={0}, protocol_header_byte={1}".format(d_header, protocol_byte))
            raise Exception

    def _setupThreadedReadLoop(self):
        self.read_lock = threading.Condition(threading.Lock())
        self.read_queue = collections.deque()
        self.read_dict = {}
        self.data_loop = True
        self.read_thread = threading.Thread(target=self._dataReadLoop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def __repr__(self):
        return "<YEI3Space {0}:{1}>".format(self.device_type, self.serial_number_hex)

    def __str__(self):
        return self.__repr__()

    def close(self):
        self.data_loop = False
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        self.read_thread.join()

    def reconnect(self):
        self.close()
        if not tryPort(self.port_name):
            _print("tryport fail")
        try:
            serial_port = serial.Serial(self.port_name, baudrate=self.baudrate, timeout=0.5, writeTimeout=0.5)
            serial_port.applySettingsDict(self.serial_port_settings)
            self.serial_port = serial_port
        except:
            traceback.print_exc()
            return False
        self._setupProtocolHeader(**self.protocol_args)
        self._setupThreadedReadLoop()
        if self.stream_timing is not None:
            self.setStreamingTiming(*self.stream_timing)
        if self.stream_slot_cmds is not None:
            self.setStreamingSlots(*self.stream_slot_cmds)
        return True

    # Wired Old Protocol WriteRead
    def f7WriteRead(self, command, input_list=None):
        command_args = self.command_dict[command]
        cmd_byte, out_len, out_struct, in_len, in_struct, compatibility = command_args
        packed_data = None
        if in_struct:
            if type(input_list) in (list, tuple):
                packed_data = struct.pack(in_struct, *input_list)
            else:
                packed_data = struct.pack(in_struct, input_list)
        write_array = makeWriteArray(0xf7, None, cmd_byte, packed_data)
        self.serial_port.write(write_array)
        if out_struct:
            output_data = self.serial_port.read(out_len)
            rtn_list = struct.unpack(out_struct, output_data)
            if len(rtn_list) != 1:
                return rtn_list
            return rtn_list[0]

    # requires the dataloop, do not call
    # Wired New Protocol WriteRead
    def f9WriteRead(self, command, input_list=None):
        global global_counter
        command_args = self.command_dict[command]
        cmd_byte, out_len, out_struct, in_len, in_struct, compatibility = command_args
        if self.compatibility < compatibility:
            raise Exception("Firmware for device on ( %s ) is out of date for this function. Recommend updating to latest firmware." % self.serial_port.name)
        packed_data = None
        if in_struct:
            if type(input_list) in (list, tuple):
                packed_data = struct.pack(in_struct, *input_list)
            else:
                packed_data = struct.pack(in_struct, input_list)
        write_array = makeWriteArray(0xf9, None, cmd_byte, packed_data)
        self.read_lock.acquire()
        uid = global_counter
        global_counter += 1
        try:
            self.serial_port.write(write_array)  # release in reader thread
        except serial.SerialTimeoutException:
            self.read_lock.release()
            self.serial_port.close()
            # _print("SerialTimeoutException!!!!")
            # !!!!!Reconnect
            return (True, None, None)
        except ValueError:
            try:
                # _print("trying to open it back up!!!!")
                self.serial_port.open()
                # _print("aaand open!!!!")
            except serial.SerialException:
                self.read_lock.release()
            # _print("SerialTimeoutException!!!!")
            # !!!!!Reconnect
            return (True, None, None)
        queue_packet = (uid, cmd_byte)
        timeout_time = 0.5 + (len(self.read_queue) * 0.150)  # timeout increases as queue gets larger
        self.read_queue.append(queue_packet)
        #start_time = time.clock() + timeout_time
        start_time = time.perf_counter() + timeout_time
        read_data = None
        while (timeout_time > 0):
            self.read_lock.wait(timeout_time)
            read_data = self.read_dict.get(uid, None)

            if read_data is not None:
                break
            timeout_time = start_time - time.perf_counter() #time.clock()
            # _print("Still waiting {0} {1} {2}".format(uid, command, timeout_time))
        else:
            # _print("Operation timed out!!!!")
            try:
                self.read_queue.remove(queue_packet)
            except:
                traceback.print_exc()
            self.read_lock.release()
            return (True, None, None)
        self.read_lock.release()
        del self.read_dict[uid]
        header_list, output_data = read_data
        fail_byte, timestamp, cmd_echo, ck_sum, rtn_log_id, sn, data_size = header_list
        if cmd_echo != cmd_byte:
            # _print("!!!!!!!!cmd_echo!=cmd_byte!!!!!")
            # _print('cmd_echo= 0x{0:02x} cmd_byte= 0x{1:02x}'.format(cmd_echo, cmd_byte))
            return (True, timestamp, None)
        rtn_list = None
        if not fail_byte:
            if out_struct:
                rtn_list = struct.unpack(out_struct, output_data)
                if len(rtn_list) == 1:
                    rtn_list = rtn_list[0]
        else:
            # _print("fail_byte!!!!triggered")
            pass
        return (fail_byte, timestamp, rtn_list)

    writeRead = f9WriteRead

    def isConnected(self, try_reconnect=False):
        try:
            serial = self.getSerialNumber()
            if serial is not None:
                return True
        except:
            pass
        return False

    ## generated functions USB and WL_ and DNG and EM_ and DL_ and BT_
    ##  85(0x55)
    def stopStreaming(self):
        fail_byte, t_stamp, data = self.writeRead('stopStreaming')
        return not fail_byte

    ##  86(0x56)
    def startStreaming(self):
        fail_byte, t_stamp, data = self.writeRead('startStreaming')
        return not fail_byte

    ##  95(0x5f)
    def updateCurrentTimestamp(self, time, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('updateCurrentTimestamp', time)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 196(0xc4)
    def setLEDMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setLEDMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 200(0xc8)
    def getLEDMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getLEDMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 223(0xdf)
    def getFirmwareVersionString(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getFirmwareVersionString')
        data = convertString(data)
        if timestamp:
            return (data, t_stamp)
        return data

    ## 225(0xe1)
    def commitSettings(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('commitSettings')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 230(0xe6)
    def getHardwareVersionString(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getHardwareVersionString')
        data = convertString(data)
        if timestamp:
            return (data, t_stamp)
        return data

    ## 237(0xed)
    def getSerialNumber(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getSerialNumber')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 238(0xee)
    def setLEDColor(self, rgb, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setLEDColor', rgb)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 239(0xef)
    def getLEDColor(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getLEDColor')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 253(0xfd)
    def setJoystickAndMousePresentRemoved(self, joystick, mouse, timestamp=False):
        arg_list = (joystick, mouse)
        fail_byte, t_stamp, data = self.writeRead('setJoystickAndMousePresentRemoved', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 254(0xfe)
    def getJoystickAndMousePresentRemoved(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getJoystickAndMousePresentRemoved')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions USB and WL_ and DNG and EM_ and DL_ and BT_


class _TSSensor(_TSBase):
    command_dict = _TSBase.command_dict.copy()
    command_dict.update({
        'getTaredOrientationAsQuaternion': (0x0, 16, '>4f', 0, None, 1),
        'getTaredOrientationAsEulerAngles': (0x1, 12, '>fff', 0, None, 1),
        'getTaredOrientationAsRotationMatrix': (0x2, 36, '>9f', 0, None, 1),
        'getTaredOrientationAsAxisAngle': (0x3, 16, '>4f', 0, None, 1),
        'getTaredOrientationAsTwoVector': (0x4, 24, '>6f', 0, None, 1),
        'getDifferenceQuaternion': (0x5, 16, '>4f', 0, None, 1),
        'getUntaredOrientationAsQuaternion': (0x6, 16, '>4f', 0, None, 1),
        'getUntaredOrientationAsEulerAngles': (0x7, 12, '>fff', 0, None, 1),
        'getUntaredOrientationAsRotationMatrix': (0x8, 36, '>9f', 0, None, 1),
        'getUntaredOrientationAsAxisAngle': (0x9, 16, '>4f', 0, None, 1),
        'getUntaredOrientationAsTwoVector': (0xa, 24, '>6f', 0, None, 1),
        'getTaredTwoVectorInSensorFrame': (0xb, 24, '>6f', 0, None, 1),
        'getUntaredTwoVectorInSensorFrame': (0xc, 24, '>6f', 0, None, 1),
        'setEulerAngleDecompositionOrder': (0x10, 0, None, 1, '>B', 1),
        'setMagnetoresistiveThreshold': (0x11, 0, None, 16, '>fIff', 3),
        'setAccelerometerResistanceThreshold': (0x12, 0, None, 8, '>fI', 3),
        'offsetWithCurrentOrientation': (0x13, 0, None, 0, None, 3),
        'resetBaseOffset': (0x14, 0, None, 0, None, 3),
        'offsetWithQuaternion': (0x15, 0, None, 16, '>4f', 3),
        'setBaseOffsetWithCurrentOrientation': (0x16, 0, None, 0, None, 3),
        'getAllNormalizedComponentSensorData': (0x20, 36, '>9f', 0, None, 1),
        'getNormalizedGyroRate': (0x21, 12, '>fff', 0, None, 1),
        'getNormalizedAccelerometerVector': (0x22, 12, '>fff', 0, None, 1),
        'getNormalizedCompassVector': (0x23, 12, '>fff', 0, None, 1),
        'getAllCorrectedComponentSensorData': (0x25, 36, '>9f', 0, None, 1),
        'getCorrectedGyroRate': (0x26, 12, '>fff', 0, None, 1),
        'getCorrectedAccelerometerVector': (0x27, 12, '>fff', 0, None, 1),
        'getCorrectedCompassVector': (0x28, 12, '>fff', 0, None, 1),
        'getCorrectedLinearAccelerationInGlobalSpace': (0x29, 12, '>fff', 0, None, 1),
        'getTemperatureC': (0x2b, 4, '>f', 0, None, 1),
        'getTemperatureF': (0x2c, 4, '>f', 0, None, 1),
        'getConfidenceFactor': (0x2d, 4, '>f', 0, None, 1),
        'getAllRawComponentSensorData': (0x40, 36, '>9f', 0, None, 1),
        'getRawGyroscopeRate': (0x41, 12, '>fff', 0, None, 1),
        'getRawAccelerometerData': (0x42, 12, '>fff', 0, None, 1),
        'getRawCompassData': (0x43, 12, '>fff', 0, None, 1),
        '_setStreamingSlots': (0x50, 0, None, 8, '>8B', 1),
        '_getStreamingSlots': (0x51, 8, '>8B', 0, None, 1),
        '_setStreamingTiming': (0x52, 0, None, 12, '>III', 1),
        '_getStreamingTiming': (0x53, 12, '>III', 0, None, 1),
        '_getStreamingBatch': (0x54, 0, None, 0, None, 1),
        'tareWithCurrentOrientation': (0x60, 0, None, 0, None, 1),
        'tareWithQuaternion': (0x61, 0, None, 16, '>4f', 1),
        'tareWithRotationMatrix': (0x62, 0, None, 36, '>9f', 1),
        'setStaticAccelerometerTrustValue': (0x63, 0, None, 4, '>f', 2),
        'setConfidenceAccelerometerTrustValues': (0x64, 0, None, 8, '>ff', 2),
        'setStaticCompassTrustValue': (0x65, 0, None, 4, '>f', 2),
        'setConfidenceCompassTrustValues': (0x66, 0, None, 8, '>ff', 2),
        'setDesiredUpdateRate': (0x67, 0, None, 4, '>I', 1),
        'setReferenceVectorMode': (0x69, 0, None, 1, '>B', 1),
        'setOversampleRate': (0x6a, 0, None, 1, '>B', 1),
        'setGyroscopeEnabled': (0x6b, 0, None, 1, '>B', 1),
        'setAccelerometerEnabled': (0x6c, 0, None, 1, '>B', 1),
        'setCompassEnabled': (0x6d, 0, None, 1, '>B', 1),
        'setAxisDirections': (0x74, 0, None, 1, '>B', 1),
        'setRunningAveragePercent': (0x75, 0, None, 4, '>f', 1),
        'setCompassReferenceVector': (0x76, 0, None, 12, '>fff', 1),
        'setAccelerometerReferenceVector': (0x77, 0, None, 12, '>fff', 1),
        'resetKalmanFilter': (0x78, 0, None, 0, None, 1),
        'setAccelerometerRange': (0x79, 0, None, 1, '>B', 1),
        'setFilterMode': (0x7b, 0, None, 1, '>B', 1),
        'setRunningAverageMode': (0x7c, 0, None, 1, '>B', 1),
        'setGyroscopeRange': (0x7d, 0, None, 1, '>B', 1),
        'setCompassRange': (0x7e, 0, None, 1, '>B', 1),
        'getTareAsQuaternion': (0x80, 16, '>4f', 0, None, 1),
        'getTareAsRotationMatrix': (0x81, 36, '>9f', 0, None, 1),
        'getAccelerometerTrustValues': (0x82, 8, '>ff', 0, None, 2),
        'getCompassTrustValues': (0x83, 8, '>ff', 0, None, 2),
        'getCurrentUpdateRate': (0x84, 4, '>I', 0, None, 1),
        'getCompassReferenceVector': (0x85, 12, '>fff', 0, None, 1),
        'getAccelerometerReferenceVector': (0x86, 12, '>fff', 0, None, 1),
        'getGyroscopeEnabledState': (0x8c, 1, '>B', 0, None, 1),
        'getAccelerometerEnabledState': (0x8d, 1, '>B', 0, None, 1),
        'getCompassEnabledState': (0x8e, 1, '>B', 0, None, 1),
        'getAxisDirections': (0x8f, 1, '>B', 0, None, 1),
        'getOversampleRate': (0x90, 1, '>B', 0, None, 1),
        'getRunningAveragePercent': (0x91, 4, '>f', 0, None, 1),
        'getDesiredUpdateRate': (0x92, 4, '>I', 0, None, 1),
        'getAccelerometerRange': (0x94, 1, '>B', 0, None, 1),
        'getFilterMode': (0x98, 1, '>B', 0, None, 1),
        'getRunningAverageMode': (0x99, 1, '>B', 0, None, 1),
        'getGyroscopeRange': (0x9a, 1, '>B', 0, None, 1),
        'getCompassRange': (0x9b, 1, '>B', 0, None, 1),
        'getEulerAngleDecompositionOrder': (0x9c, 1, '>B', 0, None, 1),
        'getMagnetoresistiveThreshold': (0x9d, 16, '>fIff', 0, None, 3),
        'getAccelerometerResistanceThreshold': (0x9e, 8, '>fI', 0, None, 3),
        'getOffsetOrientationAsQuaternion': (0x9f, 16, '>4f', 0, None, 3),
        'setCompassCalibrationCoefficients': (0xa0, 0, None, 48, '>12f', 1),
        'setAccelerometerCalibrationCoefficients': (0xa1, 0, None, 48, '>12f', 1),
        'getCompassCalibrationCoefficients': (0xa2, 48, '>12f', 0, None, 1),
        'getAccelerometerCalibrationCoefficients': (0xa3, 48, '>12f', 0, None, 1),
        'getGyroscopeCalibrationCoefficients': (0xa4, 48, '>12f', 0, None, 1),
        'beginGyroscopeAutoCalibration': (0xa5, 0, None, 0, None, 1),
        'setGyroscopeCalibrationCoefficients': (0xa6, 0, None, 48, '>12f', 1),
        'setCalibrationMode': (0xa9, 0, None, 1, '>B', 1),
        'getCalibrationMode': (0xaa, 1, '>B', 0, None, 1),
        'setOrthoCalibrationDataPointFromCurrentOrientation': (0xab, 0, None, 0, None, 1),
        'setOrthoCalibrationDataPointFromVector': (0xac, 0, None, 14, '>BBfff', 1),
        'getOrthoCalibrationDataPoint': (0xad, 12, '>fff', 2, '>BB', 1),
        'performOrthoCalibration': (0xae, 0, None, 0, None, 1),
        'clearOrthoCalibrationData': (0xaf, 0, None, 0, None, 1),
        'setSleepMode': (0xe3, 0, None, 1, '>B', 1),
        'getSleepMode': (0xe4, 1, '>B', 0, None, 1),
        'setJoystickEnabled': (0xf0, 0, None, 1, '>B', 1),
        'setMouseEnabled': (0xf1, 0, None, 1, '>B', 1),
        'getJoystickEnabled': (0xf2, 1, '>B', 0, None, 1),
        'getMouseEnabled': (0xf3, 1, '>B', 0, None, 1),
        'setControlMode': (0xf4, 0, None, 3, '>BBB', 1),
        'setControlData': (0xf5, 0, None, 7, '>BBBf', 1),
        'getControlMode': (0xf6, 1, '>B', 2, '>BB', 1),
        'getControlData': (0xf7, 4, '>f', 3, '>BBB', 1),
        'setMouseAbsoluteRelativeMode': (0xfb, 0, None, 1, '>B', 1),
        'getMouseAbsoluteRelativeMode': (0xfc, 1, '>B', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["!BASE"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                return _generateSensorClass(new_inst, serial_port, _TSSensor._device_types)
        _print('Error serial port was not made')

    def __init__(self, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        self.protocol_args = {'success_failure': True,
                              'timestamp': True,
                              'command_echo': True,
                              'data_length': True}
        if timestamp_mode != TSS_TIMESTAMP_SENSOR:
            self.protocol_args['timestamp'] = False
        self.timestamp_mode = timestamp_mode
        self.baudrate = baudrate
        reinit = False
        try:  # if this is set the class had been there before
            check = self.stream_parse
            reinit = True
            # _print("sensor reinit!!!")
        except:
            self._setupBaseVariables()
            self.callback_func = None
        self._setupProtocolHeader(**self.protocol_args)
        self._setupThreadedReadLoop()
        self.latest_lock = threading.Condition(threading.Lock())
        self.new_data = False
        if reinit:
            if self.stream_timing is not None:
                self.setStreamingTiming(*self.stream_timing)
            if self.stream_slot_cmds is not None:
                self.setStreamingSlots(*self.stream_slot_cmds)

    def _queueWriteRead(self, rtn_dict, rtn_key, retries, command, input_list=None):
        try:
            for i in range(retries):
                packet = self.writeRead(command, input_list)
                if packet[0]:
                    # _print("##Attempt: {0} complete".format(i))
                    time.sleep(0.1)
                    continue
                rtn_dict[rtn_key] = packet
                break
            else:
                # _print("sensor failed to succeed")
                rtn_dict[rtn_key] = (True, None, None)
        except(KeyboardInterrupt):
            print('\n! Received keyboard interrupt, quitting threads.\n')
            raise KeyboardInterrupt  # fix bug where a thread eats the interupt

    def queueWriteRead(self, rtn_dict, rtn_key, retries, command, input_list=None):
        return threading.Thread(target=self._queueWriteRead, args=(rtn_dict, rtn_key, retries, command, input_list))

    def _generateStreamParse(self):
        stream_string = '>'
        if self.stream_slot_cmds is None:
            self.getStreamingSlots()
        for slot_cmd in self.stream_slot_cmds:
            if slot_cmd != 'null':
                out_struct = self.command_dict[slot_cmd][2]
                stream_string += out_struct[1:]  # stripping the >
        self.stream_parse = struct.Struct(stream_string)
        # Set streaming batch command
        self.command_dict['_getStreamingBatch'] = (0x54, self.stream_parse.size, stream_string, 0, None, 1)

    def _parseStreamData(self, protocol_data, output_data):
        rtn_list = self.stream_parse.unpack(output_data)
        if len(rtn_list) == 1:
            rtn_list = rtn_list[0]

        self.latest_lock.acquire()
        self.new_data = True
        self.latest_lock.notify()
        self.latest_lock.release()
        data = (protocol_data, rtn_list)
        self.stream_last_data = data
        if self.record_data:
            self.stream_data.append(data)
        if self.callback_func:
            self.callback_func(data)

    def _dataReadLoop(self):
        while self.data_loop:
            try:
                self._readDataWiredProHeader()
            except(KeyboardInterrupt):
                print('\n! Received keyboard interrupt, quitting threads.\n')
                raise KeyboardInterrupt  # fix bug where a thread eats the interupt
            except:
                # traceback.print_exc()
                # _print("bad _parseStreamData parse")
                # _print('!!!!!inWaiting = {0}'.format(self.serial_port.inWaiting()))
                self._read_data = None
                try:
                    self.read_lock.release()
                except:
                    pass

    def _readDataWiredProHeader(self):
        _serial_port = self.serial_port
        # in_wait  = _serial_port.inWaiting()
        # if in_wait:
        # _print('!666! inWaiting = {0}'.format(in_wait))
        header_bytes = _serial_port.read(self.header_parse.size)
        if header_bytes:
            if self.timestamp_mode == TSS_TIMESTAMP_SENSOR:
                header_data = self.header_parse.unpack(header_bytes)
                header_list = padProtocolHeader71(header_data)
            elif self.timestamp_mode == TSS_TIMESTAMP_SYSTEM:
                sys_timestamp = time.perf_counter() #time.clock()  # time packet was parsed it might been in the system buffer a few ms
                sys_timestamp *= 1000000
                header_data = self.header_parse.unpack(header_bytes)
                header_list = padProtocolHeader69(header_data, sys_timestamp)
            else:
                header_data = self.header_parse.unpack(header_bytes)
                header_list = padProtocolHeader69(header_data, None)
            fail_byte, timestamp, cmd_echo, ck_sum, rtn_log_id, sn, data_size = header_list
            output_data = _serial_port.read(data_size)
            if cmd_echo == 0xff:
                if data_size:
                    self._parseStreamData(timestamp, output_data)
                return
            self.read_lock.acquire()
            if len(self.read_queue):  # here for a bug in the code
                uid, cmd_byte = self.read_queue.popleft()
                if cmd_byte == cmd_echo:
                    self.read_dict[uid] = (header_list, output_data)
                    self.read_lock.notify()  # dies in 3 seconds if there is a writeRead in wait
                else:
                    # _print('Unrequested packet found!!!')
                    # _hexDump(header_bytes, 'o')
                    # _hexDump(output_data, 'o')
                    self.read_queue.appendleft((uid, cmd_byte))
                self.read_lock.release()
                return
            # _print('Unrequested packet found!!!')
            # _hexDump(header_bytes, 'o')
            # _hexDump(output_data, 'o')
            self.read_lock.release()

    def getLatestStreamData(self, timeout):
        self.latest_lock.acquire()
        self.new_data = False
        self.latest_lock.wait(timeout)
        self.latest_lock.release()
        if self.new_data:
            return self.stream_last_data

    def setNewDataCallBack(self, callback):
        self.callback_func = callback

    def startRecordingData(self):
        self.record_data = True

    def stopRecordingData(self):
        self.record_data = False

    def clearRecordingData(self):
        self.stream_data = []

    # Convenience functions to replace commands 244(0xf4) and 245(0xf5)
    def setGlobalAxis(self, hid_type, config_axis, local_axis, global_axis, deadzone, scale, power):
        """ Sets an axis of the desired emulated input device as a 'Global Axis'
            style axis. Axis operating under this style use a reference vector
            and a consitent local vector to determine the state of the device's
            axis. As the local vector rotates, it is projected onto the global
            vector. Once the distance of that projection on the global vector
            exceeds the inputted "deadzone", the device will begin tranmitting
            non-zero values for the device's desired axis.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param config_axis: A string whose value may be either 'X' or 'Y'
                for a mouse or 'X', 'Y', or 'Z' for a joystick. This string
                defines what axis of the device is to be configured.
            
            @param local_axis: A list of 3 Floats whose value is a normalized
                Vector3. This vector represents the sensor's local vector to
                track.
            
            @param global_axis: A list of 3 Floats whose value is a normalized
                Vector3. This vector represents the global vector to project the
                local vector onto (should be orthoginal to the local vector).
            
            @param deadzone: A float that defines the minimum distance necessary
                for the device's axis to read a non-zero value.
            
            @param scale: A float that defines the linear scale for the values
                being returned for the axis.
            
            @param power: A float whose value is an exponental power used to
                further modify data being returned from the sensor.
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = hid_type
        # Set index
        axis_idx = ["X", "Y", "Z"]
        if cntl_class == TSS_MOUSE:
            axis_idx.pop(-1)
        config_axis = config_axis.upper()
        cntl_idx = -1
        try:
            cntl_idx = axis_idx.index(config_axis)
        except:
            _print("Invalid command for config_axis: {0:s}".format(config_axis))
            return False

        # Set mode
        if not self.setControlMode(cntl_class, cntl_idx, 0):
            return False

        # Create data array
        data_array = local_axis + global_axis + [deadzone, scale, power]

        # Set data
        for i in range(len(data_array)):
            if not self.setControlData(cntl_class, cntl_idx, i, data_array[i]):
                return False
        return True

    def setScreenPointAxis(self, hid_type, config_axis, dist_from_screen, dist_on_axis, collision_component, sensor_dir, button_halt):
        """ Sets an axis of the desired emulated input device as a 'Screen Point
            Axis' style axis. An axis operating under this style projects a
            vector along the sensor's direction vector into a mathmatical plane.
            The collision point on the plane is then used to determine what the
            device's axis's current value is. The direction vector is rotated
            based on the orientation of the sensor.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param config_axis: A string whose value may be either 'X' or 'Y'
                for a mouse or 'X', 'Y', or 'Z' for a joystick. This string
                defines what axis of the device is to be configured.
            
            @param dist_from_screen: A float whose value is the real world
                distance the sensor is from the user's screen. Must be the same
                units as dist_on_axis.
            
            @param dist_on_axis: A float whose value is the real world length of
                the axis along the user's screen (width of screen for x-axis,
                height of screen for y-axis). Must be the same units as
                dist_from_screen.
            
            @param collision_component: A string whose value may be 'X', 'Y', or
                'Z'. This string defines what component of the look vector's
                collision point on the virtual plane to use for manipulating the
                device's axis.
            
            @param sensor_dir: A string whose value may be 'X', 'Y', or 'Z'.
                This string defines which of the sensor's local axis to use for
                creating the vector to collide with the virtual plane.
            
            @param button_halt: A float whose value is a pause time in
                milliseconds. When a button is pressed on the emulated device,
                transmission of changes to the axis is paused for the inputted
                amount of time to prevent undesired motion detection when
                pressing buttons.
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = hid_type

        # Set index
        axis_idx = ["X", "Y", "Z"]
        if cntl_class == TSS_MOUSE:
            axis_idx.pop(-1)
        config_axis = config_axis.upper()
        cntl_idx = -1
        try:
            cntl_idx = axis_idx.index(config_axis)
        except:
            _print("Invalid command for config_axis: {0:s}".format(config_axis))
            return False

        # Set mode
        if not self.setControlMode(cntl_class, cntl_idx, 1):
            return False

        # Create data array
        axis_idx = ["X", "Y", "Z"]
        data_array = []
        data_array.append(dist_from_screen)
        data_array.append(dist_on_axis)
        collision_component = collision_component.upper()
        try:
            data_array.append(axis_idx.index(collision_component))
        except:
            _print("Invalid command for collision_component: {0:s}".format(collision_component))
            return False
        sensor_dir = sensor_dir.upper()
        try:
            data_array.append(axis_idx.index(sensor_dir))
        except:
            _print("Invalid command for sensor_dir: {0:s}".format(sensor_dir))
            return False
        data_array.append(0)
        data_array.append(0)
        data_array.append(0)
        data_array.append(button_halt)
        data_array.append(0)
        data_array.append(0)

        # Set data
        for i in range(len(data_array)):
            if not self.setControlData(cntl_class, cntl_idx, i, data_array[i]):
                return False
        return True

    def disableAxis(self, hid_type, config_axis):
        """ Disables an axis on the passed in device.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param config_axis: A string whose value may be either 'X' or 'Y'
                for a mouse or 'X', 'Y', or 'Z' for a joystick. This string
                defines what axis of the device is to be configured.
                    
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = hid_type

        # Set index
        axis_idx = ["X", "Y", "Z"]
        if cntl_class == TSS_MOUSE:
            axis_idx.pop(-1)
        config_axis = config_axis.upper()
        cntl_idx = -1
        try:
            cntl_idx = axis_idx.index(config_axis)
        except:
            _print("Invalid command for config_axis: {0:s}".format(config_axis))
            return False

        # Set mode
        return self.setControlMode(cntl_class, cntl_idx, 255)

    def setPhysicalButton(self, hid_type, button_idx, button_bind):
        """ Binds a sensor's physical button to an emulated device's button.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param button_idx: An integer whose value defines which button on
                the emulated device to configure. Default range is 0 through 7.
            
            @param button_bind: An integer whose value defines which physical
                button to bind to the emulated device's button to as defined by
                button_idx, either TSS_BUTTON_LEFT or TSS_BUTTON_RIGHT.
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = 1 + hid_type

        # Set mode
        if not self.setControlMode(cntl_class, button_idx, 0):
            return False

        # Create data
        if button_bind != TSS_BUTTON_LEFT and button_bind != TSS_BUTTON_RIGHT:
            _print("Invalid command for button_bind: {0:d}".format(button_bind))
            return False
        data = button_bind

        # Set data
        return self.setControlData(cntl_class, button_idx, 0, data)

    def setOrientationButton(self, hid_type, button_idx, local_axis, global_axis, max_dist):
        """ Sets up a device's button such that it is 'pressed' when a reference
            vector aligns itself with a local vector.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param button_idx: An integer whose value defines which button on
                the emulated device to configure. Default range is 0 through 7.
            
            @param local_axis: A list of 3 floats whose value represents a
                normalized Vector3. This vector represents the sensor's local
                vector to track.
            
            @param global_axis: A list of 3 floats whose value is a normalized
                Vector3. This vector represents the global vector to move the
                local vector towards for "pressing" (should not be colinear to
                the local vector).
            
            @param max_dist: A float whose value defines how close the local
                vector's orientation must be to the global vector for the button
                to be 'pressed'.
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = 1 + hid_type

        # Set mode
        if not self.setControlMode(cntl_class, button_idx, 1):
            return False

        # Create data array
        data_array = local_axis + global_axis + [max_dist]

        # Set data
        for i in range(7):
            if not self.setControlData(cntl_class, button_idx, i, data_array[i]):
                return False
        return True

    def setShakeButton(self, hid_type, button_idx, threshold):
        """ Sets up an emulated device's button such that it is 'pressed' when
            the sensor is shaken.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param button_idx: An integer whose value defines which button on
                the emulated device to configure. Default range is 0 through 7.
            
            @param threshold: A float whose value defines how many Gs of force
                must be experienced by the sensor before the button is
                'pressed'.
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = 1 + hid_type

        # Set mode
        if not self.setControlMode(cntl_class, button_idx, 2):
            return False

        # Create data array
        data_array = [0, 0, 0, threshold]

        # Set data
        for i in range(4):
            if not self.setControlData(cntl_class, button_idx, i, data_array[i]):
                return False
        return True

    def disableButton(self, hid_type, button_idx):
        """ Disables a button on the passed in emulated device.
            
            @param hid_type: An integer whose value defines whether the device
                in question is a TSS_JOYSTICK or TSS_MOUSE.
            
            @param button_idx: An integer whose value defines which button on
                the emulated device to configure. Default range is 0 through 7.
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        # Set class
        if hid_type != TSS_JOYSTICK and hid_type != TSS_MOUSE:
            _print("Invalid command for hid_type: {0:d}".format(hid_type))
            return False
        cntl_class = 1 + hid_type

        # Set mode
        return self.setControlMode(cntl_class, button_idx, 255)

    # Convenience functions for setting up simple mouse/joystick implimentations
    def setupSimpleMouse(self, diagonal_size, dist_from_screen, aspect_ratio, is_relative=True):
        """ Creates a simple emulated mouse device using the features of the
            sensor. Left button and right button emulate the mouse's left and
            right buttons respectivly and using the sensor as a pointing device
            with the front of the device facing towards the screen will move the
            mouse cursor.
            
            @param diagonal_size: A float whose value is the real world diagonal
                size of the user's screen.
            
            @param dist_from_screen: A float whose value is the real world
                distance the sensor is from the user's screen. Must be the same
                units as diagonal_size.
            
            @param aspect_ratio: A float whose value is the real world aspect
                ratio of the user's screen.
            
            @param is_relative: A boolean whose value expresses whether the
                mouse is to operate in relative mode (True) or absolute mode
                (False).
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        cur_mouse_rel = self.getMouseAbsoluteRelativeMode()
        if cur_mouse_rel != is_relative:
            if self.setMouseAbsoluteRelativeMode(is_relative):
                fail_byte, t_stamp, data = self.writeRead('softwareReset')
                if not fail_byte:
                    while self.getSerialNumber():
                        pass
                    self.close()
                    time.sleep(5)
                    while self.reconnect():
                        pass
        unit_hyp = (aspect_ratio ** 2 + 1) ** 0.5
        screen_multiplyer = diagonal_size / unit_hyp
        screen_width = screen_multiplyer * aspect_ratio
        screen_height = screen_multiplyer
        _print("Height: {0:2f}".format(screen_height))
        _print("Width: {0:2f}".format(screen_width))

        self.setScreenPointAxis(TSS_MOUSE, "X", dist_from_screen, screen_width, "X", "Z", 50)
        self.setScreenPointAxis(TSS_MOUSE, "Y", dist_from_screen, screen_height, "Y", "Z", 50)

        self.setPhysicalButton(TSS_MOUSE, 0, TSS_BUTTON_LEFT)
        self.setPhysicalButton(TSS_MOUSE, 1, TSS_BUTTON_RIGHT)
        self.disableButton(TSS_MOUSE, 2)
        self.disableButton(TSS_MOUSE, 3)
        self.disableButton(TSS_MOUSE, 4)
        self.disableButton(TSS_MOUSE, 5)
        self.disableButton(TSS_MOUSE, 6)
        self.disableButton(TSS_MOUSE, 7)

    def setupSimpleJoystick(self, deadzone, scale, power, shake_threshold, max_dist):
        """ Creates a simple emulated joystick device using the features of the
            sensor. The left and right physical buttons on the sensor act as
            buttons 0 and 1 for the joystick. Button 2 is a shake button.
            Buttons 3 and 4 are pressed when the sensor is rotated +-90 degrees
            on the Z-axis. Rotations on the sensor's Y and X axis correspond to
            movements on the joystick's X and Y axis.
            
            @param deadzone: A float that defines the minimum distance necessary
                for the device's axis to read a non-zero value.
            
            @param scale: A float that defines the linear scale for the values
                being returned for the axis.
            
            @param power:A float whose value is an exponental power used to
                further modify data being returned from the sensor.
            
            @param shake_threshold: A float whose value defines how many Gs of
                force must be experienced by the sensor before the button 2 is
                'pressed'.
            
            @param max_dist: A float whose value defines how close the local
                vector's orientation must  be to the global vector for buttons 3
                and 4 are "pressed".
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        self.setGlobalAxis(TSS_JOYSTICK, "X", [1, 0, 0], [0, 0, -1], deadzone, scale, power)
        self.setGlobalAxis(TSS_JOYSTICK, "Y", [0, 1, 0], [0, 0, -1], deadzone, scale, power)
        self.setPhysicalButton(TSS_JOYSTICK, 0, TSS_BUTTON_LEFT)
        self.setPhysicalButton(TSS_JOYSTICK, 1, TSS_BUTTON_RIGHT)
        self.setShakeButton(TSS_JOYSTICK, 2, shake_threshold)
        self.setOrientationButton(TSS_JOYSTICK, 3, [0, 1, 0], [-1, 0, 0], max_dist)
        self.setOrientationButton(TSS_JOYSTICK, 4, [0, 1, 0], [1, 0, 0], max_dist)
        self.disableButton(TSS_JOYSTICK, 5)
        self.disableButton(TSS_JOYSTICK, 6)
        self.disableButton(TSS_JOYSTICK, 7)

    # LightGun Functions
    def setupSimpleLightgun(self, diagonal_size, dist_from_screen, aspect_ratio, is_relative=True):
        """ Creates a simple emulated mouse based lightgun device using the
            features of the sensor. Left button of the sensor emulates the
            mouse's left button. Shaking the sensor emulates the mouse's right
            button. This configuration uses the sensor as a pointing device with
            the front of the device facing forward the screen will move the
            mouse cursor.
            
            @param diagonal_size: A float whose value is the real world diagonal
                size of the user's screen.
            
            @param dist_from_screen: A float whose value is the real world
                distance the sensor is from the user's screen. Must be the same
                units as diagonal_size.
        
            @param aspect_ratio: A float whose value is the real world aspect
                ratio of the user's screen.
            
            @param is_relative: A boolean whose value expresses whether the
                mouse is to operate in relative mode (True) or absolute mode
                (False).
            
            @return: True if the command was successfuly written to the device.
                False if the command was not written.
        """
        cur_mouse_rel = self.getMouseAbsoluteRelativeMode()
        if cur_mouse_rel != is_relative:
            if self.setMouseAbsoluteRelativeMode(is_relative):
                fail_byte, t_stamp, data = self.writeRead('softwareReset')
                if not fail_byte:
                    while self.getSerialNumber():
                        pass
                    self.close()
                    time.sleep(5)
                    while self.reconnect():
                        pass
        unit_hyp = (aspect_ratio ** 2 + 1) ** 0.5
        screen_multiplyer = diagonal_size / unit_hyp
        screen_width = screen_multiplyer * aspect_ratio
        screen_height = screen_multiplyer
        _print("Height: {0:2f}".format(screen_height))
        _print("Width: {0:2f}".format(screen_width))

        self.setScreenPointAxis(TSS_MOUSE, "X", dist_from_screen, screen_width, "X", "Z", 50)
        self.setScreenPointAxis(TSS_MOUSE, "Y", dist_from_screen, screen_height, "Y", "Z", 50)

        self.setPhysicalButton(TSS_MOUSE, 0, TSS_BUTTON_LEFT)
        self.setShakeButton(TSS_MOUSE, 1, 1.0)
        self.disableButton(TSS_MOUSE, 2)
        self.disableButton(TSS_MOUSE, 3)
        self.disableButton(TSS_MOUSE, 4)
        self.disableButton(TSS_MOUSE, 5)
        self.disableButton(TSS_MOUSE, 6)
        self.disableButton(TSS_MOUSE, 7)

    ##  80(0x50)
    def setStreamingSlots(self, slot0='null',
                          slot1='null',
                          slot2='null',
                          slot3='null',
                          slot4='null',
                          slot5='null',
                          slot6='null',
                          slot7='null'):
        slots = [slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7]
        slot_bytes = []
        for slot in slots:
            cmd_byte = self.command_dict[slot][0]
            slot_bytes.append(cmd_byte)
        fail_byte, timestamp, filler = self.writeRead('_setStreamingSlots', slot_bytes)
        self.stream_slot_cmds = slots
        self._generateStreamParse()
        return not fail_byte

    ##  81(0x51)
    def getStreamingSlots(self):
        if self.stream_slot_cmds is None:
            self.stream_slot_cmds = ['null'] * 8
        fail_byte, timestamp, slot_bytes = self.writeRead('_getStreamingSlots')
        need_update = False
        if slot_bytes:
            for slot_idx in range(len(self.stream_slot_cmds)):
                cmd_byte = slot_bytes[slot_idx]
                cmd_string = self.reverse_command_dict[cmd_byte]
                if self.stream_slot_cmds[slot_idx] != cmd_string:
                    self.stream_slot_cmds[slot_idx] = cmd_string
                    need_update = True
            if need_update:
                self._generateStreamParse()
            return self.stream_slot_cmds

    ##  82(0x52)
    def setStreamingTiming(self, interval, duration, delay, timestamp=False):
        arg_list = (interval, duration, delay)
        fail_byte, t_stamp, data = self.writeRead('_setStreamingTiming', arg_list)
        if not fail_byte:
            self.stream_timing = arg_list
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  83(0x53)
    def getStreamingTiming(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('_getStreamingTiming')
        if data:
            self.stream_timing = data
        if timestamp:
            return (data, t_stamp)
        return data

    ##  84(0x54)
    def getStreamingBatch(self, timestamp=False):
        if self.stream_parse is None:
            self._generateStreamParse()
        fail_byte, t_stamp, data = self.writeRead('_getStreamingBatch')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  85(0x55)
    def stopStreaming(self):
        self.record_data = False
        fail_byte, timestamp, slot_bytes = self.writeRead('stopStreaming')
        return not fail_byte

    ##  86(0x56)
    def startStreaming(self, start_record=False):
        self.record_data = start_record
        if self.stream_parse is None:
            self._generateStreamParse()
        fail_byte, timestamp, slot_bytes = self.writeRead('startStreaming')
        return not fail_byte

    ## generated functions USB and WL_ and EM_ and DL_ and BT_
    ##   0(0x00)
    def getTaredOrientationAsQuaternion(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTaredOrientationAsQuaternion')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   1(0x01)
    def getTaredOrientationAsEulerAngles(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTaredOrientationAsEulerAngles')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   2(0x02)
    def getTaredOrientationAsRotationMatrix(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTaredOrientationAsRotationMatrix')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   3(0x03)
    def getTaredOrientationAsAxisAngle(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTaredOrientationAsAxisAngle')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   4(0x04)
    def getTaredOrientationAsTwoVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTaredOrientationAsTwoVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   5(0x05)
    def getDifferenceQuaternion(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getDifferenceQuaternion')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   6(0x06)
    def getUntaredOrientationAsQuaternion(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUntaredOrientationAsQuaternion')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   7(0x07)
    def getUntaredOrientationAsEulerAngles(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUntaredOrientationAsEulerAngles')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   8(0x08)
    def getUntaredOrientationAsRotationMatrix(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUntaredOrientationAsRotationMatrix')
        if timestamp:
            return (data, t_stamp)
        return data

    ##   9(0x09)
    def getUntaredOrientationAsAxisAngle(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUntaredOrientationAsAxisAngle')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  10(0x0a)
    def getUntaredOrientationAsTwoVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUntaredOrientationAsTwoVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  11(0x0b)
    def getTaredTwoVectorInSensorFrame(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTaredTwoVectorInSensorFrame')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  12(0x0c)
    def getUntaredTwoVectorInSensorFrame(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUntaredTwoVectorInSensorFrame')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  16(0x10)
    def setEulerAngleDecompositionOrder(self, angle_order, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setEulerAngleDecompositionOrder', angle_order)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  17(0x11)
    def setMagnetoresistiveThreshold(self, threshold, trust_frames, lockout_decay, perturbation_detection_value, timestamp=False):
        arg_list = (threshold, trust_frames, lockout_decay, perturbation_detection_value)
        fail_byte, t_stamp, data = self.writeRead('setMagnetoresistiveThreshold', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  18(0x12)
    def setAccelerometerResistanceThreshold(self, threshold, lockout_decay, timestamp=False):
        arg_list = (threshold, lockout_decay)
        fail_byte, t_stamp, data = self.writeRead('setAccelerometerResistanceThreshold', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  19(0x13)
    def offsetWithCurrentOrientation(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('offsetWithCurrentOrientation')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  20(0x14)
    def resetBaseOffset(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('resetBaseOffset')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  21(0x15)
    def offsetWithQuaternion(self, quaternion, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('offsetWithQuaternion', quaternion)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  22(0x16)
    def setBaseOffsetWithCurrentOrientation(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setBaseOffsetWithCurrentOrientation')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  32(0x20)
    def getAllNormalizedComponentSensorData(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAllNormalizedComponentSensorData')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  33(0x21)
    def getNormalizedGyroRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getNormalizedGyroRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  34(0x22)
    def getNormalizedAccelerometerVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getNormalizedAccelerometerVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  35(0x23)
    def getNormalizedCompassVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getNormalizedCompassVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  37(0x25)
    def getAllCorrectedComponentSensorData(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAllCorrectedComponentSensorData')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  38(0x26)
    def getCorrectedGyroRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCorrectedGyroRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  39(0x27)
    def getCorrectedAccelerometerVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCorrectedAccelerometerVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  40(0x28)
    def getCorrectedCompassVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCorrectedCompassVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  41(0x29)
    def getCorrectedLinearAccelerationInGlobalSpace(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCorrectedLinearAccelerationInGlobalSpace')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  43(0x2b)
    def getTemperatureC(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTemperatureC')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  44(0x2c)
    def getTemperatureF(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTemperatureF')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  45(0x2d)
    def getConfidenceFactor(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getConfidenceFactor')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  64(0x40)
    def getAllRawComponentSensorData(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAllRawComponentSensorData')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  65(0x41)
    def getRawGyroscopeRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getRawGyroscopeRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  66(0x42)
    def getRawAccelerometerData(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getRawAccelerometerData')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  67(0x43)
    def getRawCompassData(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getRawCompassData')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  96(0x60)
    def tareWithCurrentOrientation(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('tareWithCurrentOrientation')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  97(0x61)
    def tareWithQuaternion(self, quaternion, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('tareWithQuaternion', quaternion)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  98(0x62)
    def tareWithRotationMatrix(self, rotation_matrix, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('tareWithRotationMatrix', rotation_matrix)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  99(0x63)
    def setStaticAccelerometerTrustValue(self, trust_value, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setStaticAccelerometerTrustValue', trust_value)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 100(0x64)
    def setConfidenceAccelerometerTrustValues(self, min_trust_value, max_trust_value, timestamp=False):
        arg_list = (min_trust_value, max_trust_value)
        fail_byte, t_stamp, data = self.writeRead('setConfidenceAccelerometerTrustValues', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 101(0x65)
    def setStaticCompassTrustValue(self, trust_value, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setStaticCompassTrustValue', trust_value)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 102(0x66)
    def setConfidenceCompassTrustValues(self, min_trust_value, max_trust_value, timestamp=False):
        arg_list = (min_trust_value, max_trust_value)
        fail_byte, t_stamp, data = self.writeRead('setConfidenceCompassTrustValues', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 103(0x67)
    def setDesiredUpdateRate(self, update_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setDesiredUpdateRate', update_rate)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 105(0x69)
    def setReferenceVectorMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setReferenceVectorMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 106(0x6a)
    def setOversampleRate(self, samples_per_iteration, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setOversampleRate', samples_per_iteration)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 107(0x6b)
    def setGyroscopeEnabled(self, enabled, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setGyroscopeEnabled', enabled)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 108(0x6c)
    def setAccelerometerEnabled(self, enabled, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setAccelerometerEnabled', enabled)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 109(0x6d)
    def setCompassEnabled(self, enabled, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setCompassEnabled', enabled)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 116(0x74)
    def setAxisDirections(self, axis_direction_byte, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setAxisDirections', axis_direction_byte)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 117(0x75)
    def setRunningAveragePercent(self, running_average_percent, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setRunningAveragePercent', running_average_percent)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 118(0x76)
    def setCompassReferenceVector(self, reference_vector, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setCompassReferenceVector', reference_vector)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 119(0x77)
    def setAccelerometerReferenceVector(self, reference_vector, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setAccelerometerReferenceVector', reference_vector)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 120(0x78)
    def resetKalmanFilter(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('resetKalmanFilter')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 121(0x79)
    def setAccelerometerRange(self, accelerometer_range_setting, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setAccelerometerRange', accelerometer_range_setting)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 123(0x7b)
    def setFilterMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setFilterMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 124(0x7c)
    def setRunningAverageMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setRunningAverageMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 125(0x7d)
    def setGyroscopeRange(self, gyroscope_range_setting, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setGyroscopeRange', gyroscope_range_setting)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 126(0x7e)
    def setCompassRange(self, compass_range_setting, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setCompassRange', compass_range_setting)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 128(0x80)
    def getTareAsQuaternion(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTareAsQuaternion')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 129(0x81)
    def getTareAsRotationMatrix(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getTareAsRotationMatrix')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 130(0x82)
    def getAccelerometerTrustValues(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAccelerometerTrustValues')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 131(0x83)
    def getCompassTrustValues(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCompassTrustValues')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 132(0x84)
    def getCurrentUpdateRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCurrentUpdateRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 133(0x85)
    def getCompassReferenceVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCompassReferenceVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 134(0x86)
    def getAccelerometerReferenceVector(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAccelerometerReferenceVector')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 140(0x8c)
    def getGyroscopeEnabledState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getGyroscopeEnabledState')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 141(0x8d)
    def getAccelerometerEnabledState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAccelerometerEnabledState')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 142(0x8e)
    def getCompassEnabledState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCompassEnabledState')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 143(0x8f)
    def getAxisDirections(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAxisDirections')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 144(0x90)
    def getOversampleRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getOversampleRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 145(0x91)
    def getRunningAveragePercent(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getRunningAveragePercent')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 146(0x92)
    def getDesiredUpdateRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getDesiredUpdateRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 148(0x94)
    def getAccelerometerRange(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAccelerometerRange')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 152(0x98)
    def getFilterMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getFilterMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 153(0x99)
    def getRunningAverageMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getRunningAverageMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 154(0x9a)
    def getGyroscopeRange(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getGyroscopeRange')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 155(0x9b)
    def getCompassRange(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCompassRange')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 156(0x9c)
    def getEulerAngleDecompositionOrder(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getEulerAngleDecompositionOrder')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 157(0x9d)
    def getMagnetoresistiveThreshold(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getMagnetoresistiveThreshold')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 158(0x9e)
    def getAccelerometerResistanceThreshold(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAccelerometerResistanceThreshold')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 159(0x9f)
    def getOffsetOrientationAsQuaternion(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getOffsetOrientationAsQuaternion')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 160(0xa0)
    def setCompassCalibrationCoefficients(self, matrix, bias, timestamp=False):
        arg_list = []
        arg_list.extend(matrix)
        arg_list.extend(bias)
        fail_byte, t_stamp, data = self.writeRead('setCompassCalibrationCoefficients', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 161(0xa1)
    def setAccelerometerCalibrationCoefficients(self, matrix, bias, timestamp=False):
        arg_list = []
        arg_list.extend(matrix)
        arg_list.extend(bias)
        fail_byte, t_stamp, data = self.writeRead('setAccelerometerCalibrationCoefficients', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 162(0xa2)
    def getCompassCalibrationCoefficients(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCompassCalibrationCoefficients')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 163(0xa3)
    def getAccelerometerCalibrationCoefficients(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getAccelerometerCalibrationCoefficients')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 164(0xa4)
    def getGyroscopeCalibrationCoefficients(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getGyroscopeCalibrationCoefficients')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 165(0xa5)
    def beginGyroscopeAutoCalibration(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('beginGyroscopeAutoCalibration')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 166(0xa6)
    def setGyroscopeCalibrationCoefficients(self, matrix, bias, timestamp=False):
        arg_list = []
        arg_list.extend(matrix)
        arg_list.extend(bias)
        fail_byte, t_stamp, data = self.writeRead('setGyroscopeCalibrationCoefficients', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 169(0xa9)
    def setCalibrationMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setCalibrationMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 170(0xaa)
    def getCalibrationMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getCalibrationMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 171(0xab)
    def setOrthoCalibrationDataPointFromCurrentOrientation(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setOrthoCalibrationDataPointFromCurrentOrientation')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 172(0xac)
    def setOrthoCalibrationDataPointFromVector(self, type, index, vector, timestamp=False):
        arg_list = (type, index, vector)
        fail_byte, t_stamp, data = self.writeRead('setOrthoCalibrationDataPointFromVector', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 173(0xad)
    def getOrthoCalibrationDataPoint(self, type, index, timestamp=False):
        arg_list = (type, index)
        fail_byte, t_stamp, data = self.writeRead('getOrthoCalibrationDataPoint', arg_list)
        if timestamp:
            return (data, t_stamp)
        return data

    ## 174(0xae)
    def performOrthoCalibration(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('performOrthoCalibration')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 175(0xaf)
    def clearOrthoCalibrationData(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('clearOrthoCalibrationData')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 227(0xe3)
    def setSleepMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setSleepMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 228(0xe4)
    def getSleepMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getSleepMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 240(0xf0)
    def setJoystickEnabled(self, enabled, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setJoystickEnabled', enabled)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 241(0xf1)
    def setMouseEnabled(self, enabled, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setMouseEnabled', enabled)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 242(0xf2)
    def getJoystickEnabled(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getJoystickEnabled')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 243(0xf3)
    def getMouseEnabled(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getMouseEnabled')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 244(0xf4)
    def setControlMode(self, control_class, control_index, handler_index, timestamp=False):
        arg_list = (control_class, control_index, handler_index)
        fail_byte, t_stamp, data = self.writeRead('setControlMode', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 245(0xf5)
    def setControlData(self, control_class, control_index, data_point_index, data_point, timestamp=False):
        arg_list = (control_class, control_index, data_point_index, data_point)
        fail_byte, t_stamp, data = self.writeRead('setControlData', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 246(0xf6)
    def getControlMode(self, control_class, control_index, timestamp=False):
        arg_list = (control_class, control_index)
        fail_byte, t_stamp, data = self.writeRead('getControlMode', arg_list)
        if timestamp:
            return (data, t_stamp)
        return data

    ## 247(0xf7)
    def getControlData(self, control_class, control_index, handler_index, timestamp=False):
        arg_list = (control_class, control_index, handler_index)
        fail_byte, t_stamp, data = self.writeRead('getControlData', arg_list)
        if timestamp:
            return (data, t_stamp)
        return data

    ## 251(0xfb)
    def setMouseAbsoluteRelativeMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setMouseAbsoluteRelativeMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 252(0xfc)
    def getMouseAbsoluteRelativeMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getMouseAbsoluteRelativeMode')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions USB and WL_ and EM_ and DL_ and BT_


class TSUSBSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        '_setUARTBaudRate': (0xe7, 0, None, 4, '>I', 1),
        'getUARTBaudRate': (0xe8, 4, '>I', 0, None, 1),
        'getButtonState': (0xfa, 1, '>B', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["USB", "USB-HH", "MUSB", "MUSB-HH", "USBWT", "USBWT-HH"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.01)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSUSBSensor._device_types)
        _print('Error serial port was not made')

    ## 231(0xe7)
    def setUARTBaudRate(self, baud_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('_setUARTBaudRate', baud_rate)
        if not fail_byte:
            self.baudrate = baud_rate
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions USB
    ## 232(0xe8)
    def getUARTBaudRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUARTBaudRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 250(0xfa)
    def getButtonState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getButtonState')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions USB


class TSWLSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        '_getWirelessPanID': (0xc0, 2, '>H', 0, None, 1),
        '_setWirelessPanID': (0xc1, 0, None, 2, '>H', 1),
        '_getWirelessChannel': (0xc2, 1, '>B', 0, None, 1),
        '_setWirelessChannel': (0xc3, 0, None, 1, '>B', 1),
        'commitWirelessSettings': (0xc5, 0, None, 0, None, 1),
        'getWirelessAddress': (0xc6, 2, '>H', 0, None, 1),
        'getBatteryVoltage': (0xc9, 4, '>f', 0, None, 1),
        'getBatteryPercentRemaining': (0xca, 1, '>B', 0, None, 1),
        'getBatteryStatus': (0xcb, 1, '>B', 0, None, 1),
        'getButtonState': (0xfa, 1, '>B', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["WL", "WL-HH", "MWL"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR, logical_id=None, dongle=None):
        if com_port is None and logical_id is None and dongle is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                new_inst.dongle = None
                new_inst.logical_id = None
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.01)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSWLSensor._device_types)
            _print('Error serial port was not made')
        if logical_id is not None and dongle:
            for tries in range(_wireless_retries + 1):
                fail_byte, timestamp, serial_number = dongle.faWriteRead(logical_id, 'getSerialNumber')
                if not fail_byte:
                    if serial_number in global_sensorlist:
                        rtn_inst = global_sensorlist[serial_number]
                        if rtn_inst.dongle:
                            _print("sensor was already paired before")
                            pass
                        rtn_inst.dongle = dongle
                        rtn_inst.logical_id = logical_id
                        dongle.wireless_table[logical_id] = serial_number
                        rtn_inst.switchToWirelessMode()
                        return rtn_inst
                    else:
                        new_inst = super(_TSSensor, cls).__new__(cls)
                        for tries in range(_wireless_retries + 1):
                            fail_byte, timestamp, hardware_version = dongle.faWriteRead(logical_id, 'getHardwareVersionString')
                            if not fail_byte:
                                new_inst.device_type = convertString(hardware_version)[4:-8].strip()
                                break
                        else:
                            new_inst.device_type = "WL"

                        new_inst.dongle = dongle
                        new_inst.logical_id = logical_id
                        new_inst.port_name = ""
                        new_inst.serial_port_settings = {}
                        new_inst.serial_port = None
                        new_inst.switchToWirelessMode()
                        new_inst.serial_number = serial_number
                        global_sensorlist[serial_number] = new_inst
                        return new_inst
            _print("raise wireless fail error here")
            return None
        _print('this sould never happen')
        return None

    def __init__(self, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR, logical_id=None, dongle=None):
        self.protocol_args = {'success_failure': True,
                              'timestamp': True,
                              'command_echo': True,
                              'data_length': True}
        if timestamp_mode != TSS_TIMESTAMP_SENSOR:
            self.protocol_args['timestamp'] = False
        self.timestamp_mode = timestamp_mode
        self.baudrate = baudrate
        reinit = False
        try:  # if this is set the class had been there before
            check = self.stream_parse
            reinit = True
            # _print("sensor reinit!!!")
        except:
            self._setupBaseVariables()
            self.callback_func = None
        if self.serial_port and not self.data_loop:
            self._setupProtocolHeader(**self.protocol_args)
            self._setupThreadedReadLoop()
        self.latest_lock = threading.Condition(threading.Lock())
        self.new_data = False
        if reinit:
            if self.stream_timing is not None:
                self.setStreamingTiming(*self.stream_timing)
            if self.stream_slot_cmds is not None:
                self.setStreamingSlots(*self.stream_slot_cmds)

    def close(self):
        if self.serial_port is not None:
            super(TSWLSensor, self).close()

    def _wirlessWriteRead(self, command, input_list=None):
        result = (True, None, None)
        for i in range(_wireless_retries + 1):
            result = self.dongle.faWriteRead(self.logical_id, command, input_list)
            if not result[0]:
                break
        return result

    def switchToWirelessMode(self):
        if self.dongle and self.logical_id is not None:
            self.writeRead = self._wirlessWriteRead
            self.wireless_com = True
            return True
        return False

    def switchToWiredMode(self):
        if self.serial_port:
            self.writeRead = self.f9WriteRead
            self.wireless_com = False
            return True
        return False

    ## 192(0xc0)
    def getWirelessPanID(self, timestamp=False):
        t_stamp = None
        data = None
        fail_byte, t_stamp, data = self.writeRead('_getWirelessPanID')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 193(0xc1)
    def setWirelessPanID(self, PanID, timestamp=False):
        t_stamp = None
        fail_byte = True
        if not self.wireless_com:
            fail_byte, t_stamp, data = self.writeRead('_setWirelessPanID', PanID)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 194(0xc2)
    def getWirelessChannel(self, timestamp=False):
        t_stamp = None
        data = None
        fail_byte, t_stamp, data = self.writeRead('_getWirelessChannel')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 195(0xc3)
    def setWirelessChannel(self, channel, timestamp=False):
        t_stamp = None
        fail_byte = True
        if not self.wireless_com:
            fail_byte, t_stamp, data = self.writeRead('_setWirelessChannel', channel)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions WL_
    ## 197(0xc5)
    def commitWirelessSettings(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('commitWirelessSettings')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 198(0xc6)
    def getWirelessAddress(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessAddress')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 201(0xc9)
    def getBatteryVoltage(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryVoltage')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 202(0xca)
    def getBatteryPercentRemaining(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryPercentRemaining')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 203(0xcb)
    def getBatteryStatus(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryStatus')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 250(0xfa)
    def getButtonState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getButtonState')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions WL_


class TSDongle(_TSBase):
    command_dict = _TSBase.command_dict.copy()
    command_dict.update({
        'setWirelessStreamingAutoFlushMode': (0xb0, 0, None, 1, '>B', 1),
        'getWirelessStreamingAutoFlushMode': (0xb1, 1, '>B', 0, None, 1),
        '_setWirelessStreamingManualFlushBitfield': (0xb2, 0, None, 2, '>H', 1),
        '_getWirelessStreamingManualFlushBitfield': (0xb3, 2, '>H', 0, None, 1),
        '_getManualFlushSingle': (0xb4, 0, None, 1, '>B', 1),
        '_getManualFlushBulk': (0xb5, 0, None, 0, None, 1),
        'broadcastSynchronizationPulse': (0xb6, 0, None, 0, None, 1),
        'getReceptionBitfield': (0xb7, 2, '>H', 0, None, 1),
        'getWirelessPanID': (0xc0, 2, '>H', 0, None, 1),
        'setWirelessPanID': (0xc1, 0, None, 2, '>H', 1),
        'getWirelessChannel': (0xc2, 1, '>B', 0, None, 1),
        'setWirelessChannel': (0xc3, 0, None, 1, '>B', 1),
        'commitWirelessSettings': (0xc5, 0, None, 0, None, 1),
        'getWirelessAddress': (0xc6, 2, '>H', 0, None, 1),
        'getSerialNumberAtLogicalID': (0xd0, 4, '>I', 1, '>B', 1),
        '_setSerialNumberAtLogicalID': (0xd1, 0, None, 5, '>BI', 1),
        'getWirelessChannelNoiseLevels': (0xd2, 16, '>16B', 0, None, 1),
        'setWirelessRetries': (0xd3, 0, None, 1, '>B', 1),
        'getWirelessRetries': (0xd4, 1, '>B', 0, None, 1),
        'getWirelessSlotsOpen': (0xd5, 1, '>B', 0, None, 1),
        'getSignalStrength': (0xd6, 1, '>B', 0, None, 1),
        'setWirelessHIDUpdateRate': (0xd7, 0, None, 1, '>B', 1),
        'getWirelessHIDUpdateRate': (0xd8, 1, '>B', 0, None, 1),
        'setWirelessHIDAsynchronousMode': (0xd9, 0, None, 1, '>B', 1),
        'getWirelessHIDAsynchronousMode': (0xda, 1, '>B', 0, None, 1),
        '_setWirelessResponseHeaderBitfield': (0xdb, 0, None, 4, '>I', 1),
        '_getWirelessResponseHeaderBitfield': (0xdc, 4, '>I', 0, None, 1),
        'setJoystickLogicalID': (0xf0, 0, None, 1, '>B', 1),
        'setMouseLogicalID': (0xf1, 0, None, 1, '>B', 1),
        'getJoystickLogicalID': (0xf2, 1, '>B', 0, None, 1),
        'getMouseLogicalID': (0xf3, 1, '>B', 0, None, 1)
    })

    wl_command_dict = TSWLSensor.command_dict.copy()

    _device_types = ["DNG"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(TSDongle, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.05)
                serial_port.flushInput()
                checkSoftwareVersionFromPort(serial_port)
                serial_port.write(bytearray((0xf7, 0xb7, 0xb7)))
                reception_bitfield = struct.unpack('>H', serial_port.read(2))[0]
                idx = 1
                for i in range(15):
                    if idx & reception_bitfield:
                        count = 0
                        serial_port.write(bytearray((0xf7, 0xd0, i, 0xd0 + i)))
                        wl_id = struct.unpack('>I', serial_port.read(4))[0]
                        while count < 15:
                            count += 1
                            serial_port.write(bytearray((0xf8, i, 0x56, 0x56 + i)))
                            did_fail = struct.unpack('>B', serial_port.read(1))[0]
                            if did_fail:
                                serial_port.read(1)
                            else:
                                _print("Stopped {0:08X} on try {1:d}".format(wl_id, count))
                                serial_port.read(2)
                                break
                    idx <<= 1
                return _generateSensorClass(new_inst, serial_port, TSDongle._device_types)
        _print('Error serial port was not made')

    def __init__(self, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        self.protocol_args = {'success_failure': True,
                              'timestamp': True,
                              'command_echo': True,
                              'logical_id': True,
                              'data_length': True}
        if timestamp_mode != TSS_TIMESTAMP_SENSOR:
            self.protocol_args['timestamp'] = False
        self.timestamp_mode = timestamp_mode
        self.baudrate = baudrate
        reinit = False
        try:  # if this is set the class had been there before
            check = self.wireless_table
            reinit = True
            # _print("sensor reinit!!!")
        except:
            self._setupBaseVariables()
        self._setupProtocolHeader(**self.protocol_args)
        self._setupThreadedReadLoop()
        self.setWirelessStreamingAutoFlushMode(1)
        self.startStreaming()

    def reconnect(self):
        self.close()
        if not tryPort(self.port_name):
            _print("tryport fail")
        try:
            serial_port = serial.Serial(self.port_name, baudrate=self.baudrate, timeout=0.5, writeTimeout=0.5)
            serial_port.applySettingsDict(self.serial_port_settings)
            self.serial_port = serial_port
            self.setWirelessStreamingAutoFlushMode(0)
            time.sleep(0.05)
            self.serial_port.flushInput()
            for i in range(15):
                serial_port.write(bytearray((0xf7, 0xd0, i, 0xd0 + i)))
                for i in range(10):
                    try:
                        wl_id = struct.unpack('>I', serial_port.read(4))[0]
                    except:
                        continue
                    break
                if wl_id != 0:
                    count = 0
                    while count < 25:
                        count += 1
                        serial_port.write(bytearray((0xf8, i, 0x56, 0x56 + i)))
                        did_fail = struct.unpack('>B', serial_port.read(1))[0]
                        if did_fail:
                            serial_port.read(1)
                        else:
                            _print("Stopped {0:08X} on try {1:d}".format(wl_id, count))
                            serial_port.read(2)
                            break
        except:
            traceback.print_exc()
            return False
        self._setupProtocolHeader(**self.protocol_args)
        self._setupThreadedReadLoop()
        self.setWirelessStreamingAutoFlushMode(1)
        return True

    def _setupBaseVariables(self):
        self.serial_number_hex = '{0:08X}'.format(self.serial_number)
        self.wireless_table = [0] * 15
        for i in range(15):
            tmp_id = self.f7WriteRead('getSerialNumberAtLogicalID', i)
            if tmp_id not in self.wireless_table or tmp_id == 0:
                self.wireless_table[i] = tmp_id
            else:
                self.f7WriteRead('_setSerialNumberAtLogicalID', (i, 0))

    def _setupProtocolHeader(self, success_failure=False,
                             timestamp=False,
                             command_echo=False,
                             checksum=False,
                             logical_id=False,
                             serial_number=False,
                             data_length=False):
        protocol_header = _generateProtocolHeader(success_failure,
                                                  timestamp,
                                                  command_echo,
                                                  checksum,
                                                  logical_id,
                                                  serial_number,
                                                  data_length)
        protocol_byte, self.header_parse, self.header_idx_lst = protocol_header
        d_header = self.f7WriteRead('_getWiredResponseHeaderBitfield')
        dwl_header = self.f7WriteRead('_getWirelessResponseHeaderBitfield')
        if d_header != protocol_byte or dwl_header != protocol_byte:
            self.f7WriteRead('_setWiredResponseHeaderBitfield', protocol_byte)
            self.f7WriteRead('_setWirelessResponseHeaderBitfield', protocol_byte)
            d_header = self.f7WriteRead('_getWiredResponseHeaderBitfield')
            dwl_header = self.f7WriteRead('_getWirelessResponseHeaderBitfield')
        if d_header != protocol_byte or dwl_header != protocol_byte:
            print("!!!!!fail d_header={0}, dwl_header={1}, protocol_header_byte={2}".format(d_header, dwl_header, protocol_byte))
            raise Exception

    # Wireless Old Protocol WriteRead
    def f8WriteRead(self, logical_id, command, input_list=None):
        command_args = self.command_dict[command]
        cmd_byte, out_len, out_struct, in_len, in_struct, compatibility = command_args
        packed_data = None
        if in_struct:
            if type(input_list) in (list, tuple):
                packed_data = struct.pack(in_struct, *input_list)
            else:
                packed_data = struct.pack(in_struct, input_list)
        write_array = makeWriteArray(0xf8, logical_id, cmd_byte, packed_data)
        self.serial_port.write(write_array)
        rtn_list = []
        output_data = self.serial_port.read(2)
        if len(output_data) == 2:
            fail_byte = struct.unpack('>B', output_data[0])[0]
            logical_id_byte = struct.unpack('>B', output_data[1])[0]
            rtn_list.append(fail_byte)
            if not fail_byte:
                self.serial_port.read(1)
            else:
                return True
            if out_struct:
                output_data = self.serial_port.read(out_len)
                rtn_list.append(struct.unpack(out_struct, output_data))
            if len(rtn_list) != 1:
                return rtn_list
            return rtn_list[0]
        return True

    ## Wireless New Protocol WriteRead
    def faWriteRead(self, logical_id, command, input_list=None):
        global global_counter
        command_args = self.wl_command_dict[command]
        cmd_byte, out_len, out_struct, in_len, in_struct, compatibility = command_args
        if self.compatibility < compatibility:
            raise Exception("Firmware for device on ( %s ) is out of date for this function. Recommend updating to latest firmware." % self.serial_port.name)
        packed_data = None
        if in_struct:
            if type(input_list) in (list, tuple):
                packed_data = struct.pack(in_struct, *input_list)
            else:
                packed_data = struct.pack(in_struct, input_list)
        write_array = makeWriteArray(0xfa, logical_id, cmd_byte, packed_data)

        while len(self.read_queue) > 15:
            _print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!too many commands!!!!!")
            time.sleep(0.01)
        self.read_lock.acquire()
        uid = global_counter
        global_counter += 1
        try:
            self.serial_port.write(write_array)  # release in reader thread
        except serial.SerialTimeoutException:
            self.read_lock.release()
            self.serial_port.close()
            # _print("SerialTimeoutException!!!!")
            return (True, None, None)
        except ValueError:
            try:
                # _print("trying to open it back up!!!!")
                self.serial_port.open()
                # _print("aaand open!!!!")
            except serial.SerialException:
                self.read_lock.release()
            # _print("SerialTimeoutException!!!!")
            return (True, None, None)
        queue_packet = (uid, cmd_byte)
        timeout_time = 0.5 + (len(self.read_queue) * 0.150)  # timeout increases as queue gets larger
        self.read_queue.append(queue_packet)
        start_time = time.perf_counter() + timeout_time #time.clock()
        read_data = None
        while (timeout_time > 0):
            self.read_lock.wait(timeout_time)
            read_data = self.read_dict.get(uid, None)

            if read_data is not None:
                break
            timeout_time = start_time - time.perf_counter() #time.clock()
            # _print("Still waiting {0} {1} {2} {3}".format(uid, command,logical_id, timeout_time))
        else:
            # _print("Operation timed out!!!!")
            try:
                self.read_queue.remove(queue_packet)
            except:
                traceback.print_exc()
            self.read_lock.release()
            return (True, None, None)
        self.read_lock.release()
        del self.read_dict[uid]
        header_list, output_data = read_data
        fail_byte, timestamp, cmd_echo, ck_sum, rtn_log_id, sn, data_size = header_list
        # _print("RESponse  {0} {1} {2} {3}".format(uid, command,logical_id, timeout_time))
        if logical_id != rtn_log_id:
            # _print("!!!!!!!!logical_id != rtn_log_id!!!!!")
            # _print(header_list)
            # _hexDump(output_data, 'o')
            # _print('!!!!!inWaiting = {0}'.format(self.serial_port.inWaiting()))
            return (True, timestamp, None)
        if cmd_echo != cmd_byte:
            # _print("!!!!!!!!cmd_echo!=cmd_byte!!!!!")
            # _print('cmd_echo= 0x{0:02x} cmd_byte= 0x{1:02x}'.format(cmd_echo, cmd_byte))
            # _print(header_list)
            # _hexDump(output_data, 'o')
            # _print('!!!!!inWaiting = {0}'.format(self.serial_port.inWaiting()))
            # _print('!!!!!!end')
            return (True, timestamp, None)
        rtn_list = None
        if not fail_byte:
            if out_struct:
                rtn_list = struct.unpack(out_struct, output_data)
                if len(rtn_list) == 1:
                    rtn_list = rtn_list[0]
            elif cmd_echo == 0x54:
                rtn_list = self[logical_id].stream_parse.unpack(output_data)
                if len(rtn_list) == 1:
                    rtn_list = rtn_list[0]
        else:
            # _print("fail_byte!!!!triggered")
            pass
        self._read_data = None
        return (fail_byte, timestamp, rtn_list)

    def __getitem__(self, idx):
        hw_id = self.wireless_table[idx]
        if hw_id == 0:
            return None
        # Check if sensor exists.
        if hw_id in global_sensorlist:
            rtn_inst = global_sensorlist[hw_id]
            if rtn_inst.dongle is self:
                return rtn_inst
            elif rtn_inst.dongle is None:
                _print("updating sensor {0:08X} to be wireless".format(hw_id))
                return TSWLSensor(timestamp_mode=self.timestamp_mode, dongle=self, logical_id=idx)
            return None
        # Else, make a new TSWLSensor
        else:
            _print("making new sensor {0:08X}".format(hw_id))
            return TSWLSensor(timestamp_mode=self.timestamp_mode, dongle=self, logical_id=idx)

    def getSensorFromDongle(self, idx):
        return self.__getitem__(idx)

    def setSensorToDongle(self, idx, hw_id):
        other_hw_id = self.wireless_table[idx]
        if other_hw_id != 0:
            if other_hw_id in global_sensorlist:
                other_sens = global_sensorlist[other_hw_id]
                other_sens.dongle = None
                other_sens.logical_id = None
            if hw_id not in self.wireless_table:
                if hw_id in global_sensorlist:
                    sensor = global_sensorlist[hw_id]
                    sensor.dongle = None
                    sensor.logical_id = None
                self.setSerialNumberAtLogicalID(idx, hw_id)
            else:
                if other_hw_id != hw_id:
                    other_idx = self.wireless_table.index(hw_id)
                    self.setSerialNumberAtLogicalID(other_idx, 0)
                    self.setSerialNumberAtLogicalID(idx, hw_id)
            return self.__getitem__(idx)
        elif hw_id != 0:
            self.setSerialNumberAtLogicalID(idx, hw_id)
            return self.__getitem__(idx)

    def _dataReadLoop(self):
        while self.data_loop:
            try:
                self._readDataWirelessProHeader()
            except(KeyboardInterrupt):
                print('\n! Received keyboard interrupt, quitting threads.\n')
                raise KeyboardInterrupt  # fix bug where a thread eats the interupt
                break
            except:
                # traceback.print_exc()
                # _print("bad _parseStreamData parse")
                # _print('!!!!!inWaiting = {0}'.format(self.serial_port.inWaiting()))
                try:
                    self.read_lock.release()
                except:
                    pass

    def _readDataWirelessProHeader(self):
        _serial_port = self.serial_port
        # in_wait = _serial_port.inWaiting()
        # if in_wait:
        # _print('!1025! inWaiting = {0}'.format(in_wait))
        header_bytes = _serial_port.read(self.header_parse.size)
        if header_bytes:
            # _hexDump(header_bytes, 'o')
            if self.timestamp_mode == TSS_TIMESTAMP_SENSOR:
                header_data = self.header_parse.unpack(header_bytes)
                header_list = padProtocolHeader87(header_data)
            elif self.timestamp_mode == TSS_TIMESTAMP_SYSTEM:
                sys_timestamp = time.perf_counter()  #time.clock()  # time packet was parsed it might been in the system buffer a few ms
                sys_timestamp *= 1000000
                header_data = self.header_parse.unpack(header_bytes)
                header_list = padProtocolHeader85(header_data, sys_timestamp)
            else:
                header_data = self.header_parse.unpack(header_bytes)
                header_list = padProtocolHeader85(header_data, None)
            fail_byte, timestamp, cmd_echo, ck_sum, rtn_log_id, sn, data_size = header_list
            # _print("!!!!fail_byte={0}, cmd_echo={1}, rtn_log_id={2}, data_size={3}".format(fail_byte, cmd_echo, rtn_log_id, data_size))
            output_data = _serial_port.read(data_size)

            if cmd_echo == 0xff:
                if data_size:
                    self[rtn_log_id]._parseStreamData(timestamp, output_data)
                return
            self.read_lock.acquire()
            # _print('retrning data!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            if len(self.read_queue):  # here for a bug in the code
                uid, cmd_byte = self.read_queue.popleft()
                if cmd_byte == cmd_echo:
                    self.read_dict[uid] = (header_list, output_data)
                    self.read_lock.notifyAll()  # dies in 3 seconds if there is a writeRead in wait
                else:
                    # _print('Unrequested packet found!!!')
                    # _hexDump(header_bytes, 'o')
                    # _hexDump(output_data, 'o')
                    self.read_queue.appendleft((uid, cmd_byte))
                self.read_lock.release()
                return
            # _print('Unrequested packet found (read_queue is empty)!!!')
            # _hexDump(header_bytes, 'o')
            # _hexDump(output_data, 'o')
            # _print("no status bytes")
            self.read_lock.release()

    ## 209(0xd1)
    def setSerialNumberAtLogicalID(self, logical_id, serial_number, timestamp=False):
        arg_list = (logical_id, serial_number)
        fail_byte, t_stamp, data = self.writeRead('_setSerialNumberAtLogicalID', arg_list)
        if not fail_byte:
            self.wireless_table[logical_id] = serial_number
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions DNG
    ## 176(0xb0)
    def setWirelessStreamingAutoFlushMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setWirelessStreamingAutoFlushMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 177(0xb1)
    def getWirelessStreamingAutoFlushMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessStreamingAutoFlushMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 182(0xb6)
    def broadcastSynchronizationPulse(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('broadcastSynchronizationPulse')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 183(0xb7)
    def getReceptionBitfield(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getReceptionBitfield')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 192(0xc0)
    def getWirelessPanID(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessPanID')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 193(0xc1)
    def setWirelessPanID(self, PanID, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setWirelessPanID', PanID)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 194(0xc2)
    def getWirelessChannel(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessChannel')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 195(0xc3)
    def setWirelessChannel(self, channel, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setWirelessChannel', channel)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 197(0xc5)
    def commitWirelessSettings(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('commitWirelessSettings')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 198(0xc6)
    def getWirelessAddress(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessAddress')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 208(0xd0)
    def getSerialNumberAtLogicalID(self, logical_id, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getSerialNumberAtLogicalID', logical_id)
        if timestamp:
            return (data, t_stamp)
        return data

    ## 210(0xd2)
    def getWirelessChannelNoiseLevels(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessChannelNoiseLevels')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 211(0xd3)
    def setWirelessRetries(self, retries, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setWirelessRetries', retries)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 212(0xd4)
    def getWirelessRetries(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessRetries')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 213(0xd5)
    def getWirelessSlotsOpen(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessSlotsOpen')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 214(0xd6)
    def getSignalStrength(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getSignalStrength')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 215(0xd7)
    def setWirelessHIDUpdateRate(self, update_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setWirelessHIDUpdateRate', update_rate)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 216(0xd8)
    def getWirelessHIDUpdateRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessHIDUpdateRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 217(0xd9)
    def setWirelessHIDAsynchronousMode(self, mode, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setWirelessHIDAsynchronousMode', mode)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 218(0xda)
    def getWirelessHIDAsynchronousMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getWirelessHIDAsynchronousMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 240(0xf0)
    def setJoystickLogicalID(self, logical_id, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setJoystickLogicalID', logical_id)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 241(0xf1)
    def setMouseLogicalID(self, logical_id, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('setMouseLogicalID', logical_id)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## 242(0xf2)
    def getJoystickLogicalID(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getJoystickLogicalID')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 243(0xf3)
    def getMouseLogicalID(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getMouseLogicalID')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions DNG


class TSEMSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        'setPinMode': (0x1d, 0, None, 2, '>BB', 1),
        'getPinMode': (0x1e, 2, '>BB', 0, None, 1),
        'getInterruptStatus': (0x1f, 1, '>B', 0, None, 1),
        '_setUARTBaudRate': (0xe7, 0, None, 4, '>I', 1),
        'getUARTBaudRate': (0xe8, 4, '>I', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["EM", "EM-HH"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.01)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSEMSensor._device_types)
        _print('Error serial port was not made')

    ## 231(0xe7)
    def setUARTBaudRate(self, baud_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('_setUARTBaudRate', baud_rate)
        if not fail_byte:
            self.baudrate = baud_rate
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions EM_
    ##  29(0x1d)
    def setPinMode(self, mode, pin, timestamp=False):
        arg_list = (mode, pin)
        fail_byte, t_stamp, data = self.writeRead('setPinMode', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  30(0x1e)
    def getPinMode(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getPinMode')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  31(0x1f)
    def getInterruptStatus(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getInterruptStatus')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 232(0xe8)
    def getUARTBaudRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUARTBaudRate')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions EM_


class TSDLSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        'turnOnMassStorage': (0x39, 0, None, 0, None, 1),
        'turnOffMassStorage': (0x3a, 0, None, 0, None, 1),
        'formatAndInitializeSDCard': (0x3b, 0, None, 0, None, 1),
        'beginDataLoggingSession': (0x3c, 0, None, 0, None, 1),
        'endDataLoggingSession': (0x3d, 0, None, 0, None, 1),
        'setClockValues': (0x3e, 0, None, 6, '>6B', 1),
        'getClockValues': (0x3f, 6, '>6B', 0, None, 1),
        'getBatteryVoltage': (0xc9, 4, '>f', 0, None, 1),
        'getBatteryPercentRemaining': (0xca, 1, '>B', 0, None, 1),
        'getBatteryStatus': (0xcb, 1, '>B', 0, None, 1),
        'getButtonState': (0xfa, 1, '>B', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["DL", "DL-HH"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.01)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSDLSensor._device_types)
        _print('Error serial port was not made')

    ## generated functions DL_
    ##  57(0x39)
    def turnOnMassStorage(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('turnOnMassStorage')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  58(0x3a)
    def turnOffMassStorage(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('turnOffMassStorage')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  59(0x3b)
    def formatAndInitializeSDCard(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('formatAndInitializeSDCard')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  60(0x3c)
    def beginDataLoggingSession(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('beginDataLoggingSession')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  61(0x3d)
    def endDataLoggingSession(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('endDataLoggingSession')
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  62(0x3e)
    def setClockValues(self, month, day, year, hour, minute, second, timestamp=False):
        arg_list = (month, day, year, hour, minute, second)
        fail_byte, t_stamp, data = self.writeRead('setClockValues', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  63(0x3f)
    def getClockValues(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getClockValues')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 201(0xc9)
    def getBatteryVoltage(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryVoltage')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 202(0xca)
    def getBatteryPercentRemaining(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryPercentRemaining')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 203(0xcb)
    def getBatteryStatus(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryStatus')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 250(0xfa)
    def getButtonState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getButtonState')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions DL_


class TSBTSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        'getBatteryVoltage': (0xc9, 4, '>f', 0, None, 1),
        'getBatteryPercentRemaining': (0xca, 1, '>B', 0, None, 1),
        'getBatteryStatus': (0xcb, 1, '>B', 0, None, 1),
        '_setUARTBaudRate': (0xe7, 0, None, 4, '>I', 1),
        'getUARTBaudRate': (0xe8, 4, '>I', 0, None, 1),
        'getButtonState': (0xfa, 1, '>B', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["BT", "BT-HH", "MBT"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=2.5, writeTimeout=2.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.25)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSBTSensor._device_types)
        _print('Error serial port was not made')

    ## 231(0xe7)
    def setUARTBaudRate(self, baud_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('_setUARTBaudRate', baud_rate)
        if not fail_byte:
            self.baudrate = baud_rate
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions BT_
    ## 201(0xc9)
    def getBatteryVoltage(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryVoltage')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 202(0xca)
    def getBatteryPercentRemaining(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryPercentRemaining')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 203(0xcb)
    def getBatteryStatus(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getBatteryStatus')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 232(0xe8)
    def getUARTBaudRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUARTBaudRate')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 250(0xfa)
    def getButtonState(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getButtonState')
        if timestamp:
            return (data, t_stamp)
        return data


## END generated functions BT_

class TSLXSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        'setInterruptType': (0x1d, 0, None, 3, '>BBB', 1),
        'readInterruptType': (0x1e, 3, '>BBB', 0, None, 1),
        'getInterruptStatus': (0x1f, 1, '>B', 0, None, 1),
        '_setUARTBaudRate': (0xe7, 0, None, 4, '>I', 1),
        'getUARTBaudRate': (0xe8, 4, '>I', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["LX", "LX-HH"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.01)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSLXSensor._device_types)
        _print('Error serial port was not made')

    ## 231(0xe7)
    def setUARTBaudRate(self, baud_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('_setUARTBaudRate', baud_rate)
        if not fail_byte:
            self.baudrate = baud_rate
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions EM_
    ##  29(0x1d)
    def setInterruptType(self, mode, pin, timestamp=False):
        arg_list = (mode, pin)
        fail_byte, t_stamp, data = self.writeRead('setInterruptType', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  30(0x1e)
    def readInterruptType(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('readInterruptType')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  31(0x1f)
    def getInterruptStatus(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getInterruptStatus')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 232(0xe8)
    def getUARTBaudRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUARTBaudRate')
        if timestamp:
            return (data, t_stamp)
        return data

## END generated functions LX_

class TSNANOSensor(_TSSensor):
    command_dict = _TSSensor.command_dict.copy()
    command_dict.update({
        'setInterruptType': (0x1d, 0, None, 3, '>BBB', 1),
        'readInterruptType': (0x1e, 3, '>BBB', 0, None, 1),
        'getInterruptStatus': (0x1f, 1, '>B', 0, None, 1),
        '_setUARTBaudRate': (0xe7, 0, None, 4, '>I', 1),
        'getUARTBaudRate': (0xe8, 4, '>I', 0, None, 1)
    })

    reverse_command_dict = dict(map(lambda x: [x[1][0], x[0]], command_dict.items()))

    _device_types = ["Nano", "NANO-HH"]

    def __new__(cls, com_port=None, baudrate=_baudrate, timestamp_mode=TSS_TIMESTAMP_SENSOR):
        if com_port is None:
            return None
        if com_port:
            if type(com_port) is str:
                port_name = com_port
            elif type(com_port) is ComInfo:
                port_name = com_port.com_port
            else:
                _print("An erronous parameter was passed in")
                return None
            if baudrate not in _allowed_baudrates:
                baudrate = _baudrate
                _print("Error baudrate value not allowed. Using default.")
            serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.5, writeTimeout=0.5)
            if serial_port is not None:
                new_inst = super(_TSSensor, cls).__new__(cls)
                serial_port.write(bytearray((0xf7, 0x56, 0x56)))
                time.sleep(0.01)
                serial_port.flushInput()
                return _generateSensorClass(new_inst, serial_port, TSNANOSensor._device_types)
        _print('Error serial port was not made')

    ## 231(0xe7)
    def setUARTBaudRate(self, baud_rate, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('_setUARTBaudRate', baud_rate)
        if not fail_byte:
            self.baudrate = baud_rate
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ## generated functions EM_
    ##  29(0x1d)
    def setInterruptType(self, mode, pin, timestamp=False):
        arg_list = (mode, pin)
        fail_byte, t_stamp, data = self.writeRead('setInterruptType', arg_list)
        if timestamp:
            return (not fail_byte, t_stamp)
        return not fail_byte

    ##  30(0x1e)
    def readInterruptType(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('readInterruptType')
        if timestamp:
            return (data, t_stamp)
        return data

    ##  31(0x1f)
    def getInterruptStatus(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getInterruptStatus')
        if timestamp:
            return (data, t_stamp)
        return data

    ## 232(0xe8)
    def getUARTBaudRate(self, timestamp=False):
        fail_byte, t_stamp, data = self.writeRead('getUARTBaudRate')
        if timestamp:
            return (data, t_stamp)
        return data

## END generated functions Nano

global_broadcaster = Broadcaster()
