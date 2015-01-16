import wx
import wx.lib.platebtn as platebtn
import itertools
import _winreg
import _winreg as reg
from itertools import count
import wmi
import re
import sys
import subprocess
from subprocess import check_output
import os
import serial
from threading import Thread
from pyblehci import BLEBuilder
from pyblehci import BLEParser
import sched
import time
from threading import Lock
import Log
import serial
import binascii

TARGET_NAME = 1
TARGET_ADDR = 2
FAIL = 3
SUCCESS = 4
SINGLE_READ = 5
SINGLE_WRITE = 6
NOTIFICATION = 7
INDICATION = 8

def dummy():
    pass

def dummy_result_callback():
    Log.screen("dummy_result_callback invoked")
    pass

class DongleMgr:
    mgr_idx = 0
    ble_serial_port_name = ""
    ble_dongle = None
    ble_builder = None
    ble_parser = None
    max_capacity = 3

    # Command queue for dongle/connection mgmt
    cmd_mgmt_lock = None
    conn_handle_addr_dict = {}          # Read/Write/Notification handle related information
    addr_handle_dict = {}               # Persistent cache
    cmd_mgmt_known_name_addr = {}       # Known device names and their corresponding address
    cmd_mgmt = {}
    # Command queue for per target commands
    cmd_tgt = {}
    cmd_tgt_lock = {}

    def __init__(self, idx, port_name):
        self.mgr_idx = idx
        self.ble_serial_port_name = port_name
        self.cmd_mgmt_lock = Lock()

        try:
            self.ble_dongle = serial.Serial(port = self.ble_serial_port_name, baudrate = 115200)
            if self.ble_dongle == None:
                Log.screen("BLE USB Dongle in use, please close other applications and try again.")

        except serial.SerialException:
            Log.screen("BLE USB Dongle in use, please close other applications and try again.")

        except UnicodeDecodeError:
            Log.screen("BLE USB Dongle in use, please close other applications and try again.")

        if self.ble_dongle != None:
            # Initialize BLE Dongle
            self.ble_builder = BLEBuilder(self.ble_dongle)
            self.ble_parser = BLEParser(self.ble_dongle, callback = self.analyse_packet)

            # Send the initialize USB dongle command
            ret, ret_reason = self.__issue_mgmt_cmd__("fe00", 2, None, None)    # GAP Device Initialization
            if ret != SUCCESS:
                Log.screen("Failed to initialize USB BLE Dongle")
            else:
                Log.screen("Initialize done!")
        else:
            Log.screen("Failed to open BLE USB Dongle " + self.ble_serial_port_name)

    def stop(self):
        if self.ble_parser != None:
            Log.screen("======================== S T O P I N G =============================")
            Log.log("======================== S T O P I N G =============================")
            for chandle in self.cmd_tgt.keys():
                self.disconnect(chandle)
            self.ble_parser.stop()
            time.sleep(1)

    # FIXME: check multiple DongleMgr
    def analyse_packet(self, (packet, dictionary)):
        Log.log(dictionary)
        # Check dongle ack
        if dictionary['event'][1] == "GAP_HCI_ExtensionCommandStatus":
            # check mgmt cmds
            self.cmd_mgmt_lock.acquire()
            for key in self.cmd_mgmt.keys():
                if key.decode('hex')[::-1] == dictionary['op_code'][0] and self.cmd_mgmt[key]["status"] == "CMD_SENT":
                    if dictionary['status'][1] == "00":
                        self.cmd_mgmt[key]["status"] = "CMD_ACKED_BY_DONGLE"
                        for q_element in self.cmd_mgmt[key]["sched"].queue:
                            if q_element[2] == self.timeout_handle_dongle_nak:
                                self.cmd_mgmt[key]["sched"].cancel(q_element)
                                self.cmd_mgmt_lock.release()
                                return
                    else:
                        self.cmd_mgmt[key]["status"] = "CMD_REJECTED_BY_DONGLE " + dictionary['status'][1]
                        self.cmd_mgmt[key]["result_reason"] = dictionary['status'][1]
                        self.cmd_mgmt_lock.release()
                        return
            self.cmd_mgmt_lock.release()
            # check tgt cmds
            for conn_handle_key in self.cmd_tgt_lock.keys():
                self.cmd_tgt_lock[conn_handle_key].acquire()
                for key in self.cmd_tgt[conn_handle_key].keys():
                    if key.decode('hex')[::-1] == dictionary['op_code'][0] and self.cmd_tgt[conn_handle_key][key]["status"] == "CMD_SENT":
                        if dictionary['status'][1] == "00":
                            self.cmd_tgt[conn_handle_key][key]["status"] = "CMD_ACKED_BY_DONGLE"
                            for q_element in self.cmd_tgt[conn_handle_key][key]["sched"].queue:
                                if q_element[2] == self.timeout_handle_dongle_nak:
                                    self.cmd_tgt[conn_handle_key][key]["sched"].cancel(q_element)
                                    self.cmd_tgt_lock[conn_handle_key].release()
                                    return
                        else:
                            self.cmd_tgt[conn_handle_key][key]["status"] = "CMD_REJECTED_BY_DONGLE " + dictionary['status'][1]
                            self.cmd_tgt_lock[conn_handle_key].release()
                            return
                self.cmd_tgt_lock[conn_handle_key].release()


        if dictionary['event'][1] == "GAP_DeviceInitDone":
            cmd_id = "fe00"
            # check mgmt cmds
            self.cmd_mgmt_lock.acquire()
            if cmd_id in self.cmd_mgmt:
                if dictionary['status'][1] == "00":
                    self.cmd_mgmt[cmd_id]["result"] = SUCCESS
                else:
                    self.cmd_mgmt[cmd_id]["result_reason"] = dictionary['status'][1]

                if len(self.cmd_mgmt[cmd_id]["sched"].queue) > 0:
                    self.cmd_mgmt[cmd_id]["sched"].cancel(self.cmd_mgmt[cmd_id]["sched"].queue[0])
                self.cmd_mgmt_lock.release()
                return
            self.cmd_mgmt_lock.release()

        if dictionary['event'][1] == "GAP_DeviceInformation":
            if dictionary['event_type'][1] == "04":
                data_field_list = list(dictionary['data_field'][0])
                data_field_list.pop(0)
                data_field_list.pop(0)
                self.cmd_mgmt_lock.acquire()
                if not ''.join(data_field_list) in self.cmd_mgmt_known_name_addr:
                    self.cmd_mgmt_known_name_addr[''.join(data_field_list)] = [dictionary['addr'], dictionary['addr_type'][0]]
                self.cmd_mgmt_lock.release()
                self.callback_device_info(''.join(data_field_list), dictionary['addr'], dictionary['addr_type'][0])

        if dictionary['event'][1] == "GAP_EstablishLink":
            cmd_id = "fe09"
            # check mgmt cmds
            self.cmd_mgmt_lock.acquire()
            if cmd_id in self.cmd_mgmt:
                if dictionary['status'][1] == "00":
                    self.cmd_mgmt[cmd_id]["result"] = SUCCESS
                    self.cmd_mgmt[cmd_id]["result_reason"] = dictionary['conn_handle'][0]
                    if dictionary['conn_handle'][0] in self.cmd_tgt or dictionary['conn_handle'][0] in self.cmd_tgt_lock:
                        Log.screen("BUG: conn_handle not cleaned up!")
                    self.cmd_tgt[dictionary['conn_handle'][0]] = {}
                    self.cmd_tgt_lock[dictionary['conn_handle'][0]] = Lock()
                else:
                    self.cmd_mgmt[cmd_id]["result_reason"] = dictionary['status'][1]

                if len(self.cmd_mgmt[cmd_id]["sched"].queue) > 0:
                    self.cmd_mgmt[cmd_id]["sched"].cancel(self.cmd_mgmt[cmd_id]["sched"].queue[0])
                self.cmd_mgmt_lock.release()
                return
            self.cmd_mgmt_lock.release()

        if dictionary['event'][1] == "GAP_LinkTerminated":
            cmd_id = "fe0a"
            # check mgmt cmds
            self.cmd_mgmt_lock.acquire()
            if cmd_id in self.cmd_mgmt:
                if dictionary['status'][1] == "00":
                    self.cmd_mgmt[cmd_id]["result"] = SUCCESS
                    if (not dictionary['conn_handle'][0] in self.cmd_tgt) or (not dictionary['conn_handle'][0] in self.cmd_tgt_lock):
                        Log.screen("BUG: conn_handle not seen!")
                    del self.cmd_tgt[dictionary['conn_handle'][0]]
                    del self.cmd_tgt_lock[dictionary['conn_handle'][0]]
                else:
                    self.cmd_mgmt[cmd_id]["result_reason"] = dictionary['status'][1]

                if len(self.cmd_mgmt[cmd_id]["sched"].queue) > 0:
                    self.cmd_mgmt[cmd_id]["sched"].cancel(self.cmd_mgmt[cmd_id]["sched"].queue[0])
                self.cmd_mgmt_lock.release()
                return
            self.cmd_mgmt_lock.release()

        if dictionary['event'][1] == "ATT_ReadByTypeRsp":
            cmd_id = "fdb4" # take one default
            conn_handle = dictionary['conn_handle'][0]
            # check tgt cmds
            if conn_handle in self.cmd_tgt_lock:
                self.cmd_tgt_lock[conn_handle].acquire()

                # Check fdb4/fd88
                if "fdb4" in self.cmd_tgt[conn_handle] and "fd88" in self.cmd_tgt[conn_handle]:
                    Log.screen("BUG: fdb4 and fd88 should not appear at the same time")
                else:
                    if "fd88" in self.cmd_tgt[conn_handle]:
                        cmd_id = "fd88"

                if cmd_id in self.cmd_tgt[conn_handle]:
                    if dictionary['status'][1] == "00":
                        self.cmd_tgt[conn_handle][cmd_id]["result"] = SUCCESS
                        if self.cmd_tgt[conn_handle][cmd_id]["result_reason"] == None:
                            self.cmd_tgt[conn_handle][cmd_id]["result_reason"] = []
                        ret = {}
                        ret["handle"] = dictionary['results'][0]['handle'][1]
                        ret["data"] = ''
                        n = 2
                        for i in range(0, len(dictionary['results'][0]['data'][1]), n):
                            ret["data"] = ret["data"] + dictionary['results'][0]['data'][1][(len(dictionary['results'][0]['data'][1]) - i - n) : (len(dictionary['results'][0]['data'][1]) - i)]
                        self.cmd_tgt[conn_handle][cmd_id]["result_reason"].append(ret)
                    elif dictionary['status'][1] == "1a":
                        if len(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue) > 0:
                            self.cmd_tgt[conn_handle][cmd_id]["sched"].cancel(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue[0])
                self.cmd_tgt_lock[conn_handle].release()

        if dictionary['event'][1] == "ATT_WriteRsp":
            cmd_id = "fd92"
            conn_handle = dictionary['conn_handle'][0]
            # check tgt cmds
            if conn_handle in self.cmd_tgt_lock:
                self.cmd_tgt_lock[conn_handle].acquire()
                if cmd_id in self.cmd_tgt[conn_handle]:
                    if dictionary['status'][1] == "00":
                        self.cmd_tgt[conn_handle][cmd_id]["result"] = SUCCESS

                    if len(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue) > 0:
                        self.cmd_tgt[conn_handle][cmd_id]["sched"].cancel(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue[0])
                self.cmd_tgt_lock[conn_handle].release()

        if dictionary['event'][1] == "ATT_ReadRsp":
            cmd_id = "fd8a"
            conn_handle = dictionary['conn_handle'][0]
            # check tgt cmds
            if conn_handle in self.cmd_tgt_lock:
                self.cmd_tgt_lock[conn_handle].acquire()
                if cmd_id in self.cmd_tgt[conn_handle]:
                    if dictionary['status'][1] == "00":
                        self.cmd_tgt[conn_handle][cmd_id]["result"] = SUCCESS
                        self.cmd_tgt[conn_handle][cmd_id]["result_reason"] = '' 
                        n = 2
                        for i in range(0, len(dictionary['value'][1]), n):
                            self.cmd_tgt[conn_handle][cmd_id]["result_reason"] = self.cmd_tgt[conn_handle][cmd_id]["result_reason"] + dictionary['value'][1][(len(dictionary['value'][1]) - i - n) : (len(dictionary['value'][1]) - i)]
                    if len(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue) > 0:
                        self.cmd_tgt[conn_handle][cmd_id]["sched"].cancel(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue[0])
                self.cmd_tgt_lock[conn_handle].release()

        if dictionary['event'][1] == "ATT_ErrorRsp":
            Log.log("Got error response:")
            Log.log(dictionary)

            cmd_id = "fd92"
            conn_handle = dictionary['conn_handle'][0]
            # check tgt cmds
            if conn_handle in self.cmd_tgt_lock:
                self.cmd_tgt_lock[conn_handle].acquire()
                if cmd_id in self.cmd_tgt[conn_handle]:
                    if dictionary['status'][1] == "00":
                        if len(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue) > 0:
                            self.cmd_tgt[conn_handle][cmd_id]["sched"].cancel(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue[0])
                self.cmd_tgt_lock[conn_handle].release()

    def init_handler_device_info(self, cmd_ref, target_name):
        cmd_ref["dev_name"] = target_name
        cmd_ref["dev_addr_info"] = []

    def callback_device_info(self, dev_name, dev_addr, addr_type):
        cmd_id = "fe04"                     # GAP Device Discovery Request
        self.cmd_mgmt_lock.acquire()
        if not cmd_id in self.cmd_mgmt:
            self.cmd_mgmt_lock.release()
            return
        
        if self.cmd_mgmt[cmd_id]["dev_name"] == dev_name:
            self.cmd_mgmt[cmd_id]["dev_addr_info"] = [dev_addr, addr_type]
            self.cmd_mgmt[cmd_id]["result"] = SUCCESS
            self.cmd_mgmt[cmd_id]["result_reason"] = [dev_addr, addr_type]
            self.cmd_mgmt[cmd_id]["sched"].cancel(self.cmd_mgmt[cmd_id]["sched"].queue[0])
            self.cmd_mgmt_lock.release()
        else:
            self.cmd_mgmt_lock.release()

    def connect(self, name_or_addr, target, addr_type = '\x00'):
        addr = ""
        addr_for_connect = ''
        if name_or_addr == TARGET_NAME:
            retry_cnt = 0
            ret = FAIL
            addr_info = []
            while retry_cnt < 5:
                # Check device name from cache results
                self.cmd_mgmt_lock.acquire()
                if target in self.cmd_mgmt_known_name_addr:
                    #addr = self.cmd_mgmt_known_name_addr[target][0][1]
                    #addr_for_connect = self.cmd_mgmt_known_name_addr[target][0][0]
                    #addr_type = self.cmd_mgmt_known_name_addr[target][1]
                    addr_info = self.cmd_mgmt_known_name_addr[target]
                    ret = SUCCESS
                self.cmd_mgmt_lock.release()

                if ret == SUCCESS:
                    break

                ret, addr_info = self.__issue_mgmt_cmd__("fe04", 10, self.init_handler_device_info, target, mode="\x03")
                if ret == SUCCESS:
                    self.__issue_mgmt_cmd__("fe05", 2, None, None)      # Cancel discovery
                    break
                else:
                    time.sleep(5)
                retry_cnt = retry_cnt + 1

            if ret != SUCCESS:
                return ret, "Device not found"

            addr = addr_info[0][1]
            addr_for_connect = addr_info[0][0]
            addr_type = addr_info[1]

        else:
            addr = target
            addr_for_connect = ''
            n = 2
            for i in range(0, len(addr), n):
                addr_for_connect = addr_for_connect + chr(int(addr[(len(addr) - i - n) : (len(addr) - i)], 16))

        # Connect to device
        Log.log("Connecting to " + addr)

        self.cmd_mgmt_lock.acquire()
        if not addr in self.addr_handle_dict:
            self.addr_handle_dict[addr] = {}
            self.addr_handle_dict[addr]['notification'] = {}
            self.addr_handle_dict[addr]['read'] = {}
            self.addr_handle_dict[addr]['write'] = {}
        self.cmd_mgmt_lock.release()

        while 1:
            ret, handle = self.__issue_mgmt_cmd__("fe09", 10, None, None, peer_addr = addr_for_connect, addr_type_peer = addr_type)   # GAP Establish Link Request
            if ret == SUCCESS:
                Log.log("Connected to " + addr)
                self.cmd_mgmt_lock.acquire()
                if handle in self.conn_handle_addr_dict:
                    del self.conn_handle_addr_dict[handle]
                self.conn_handle_addr_dict[handle] = addr
                self.cmd_mgmt_lock.release()
                break
            else:
                if handle == '10':
                    time.sleep(5)
                else:
                    Log.log("Failed to connect to " + addr)
                    break

        return ret, handle

    def disconnect(self, chandle):
        if not chandle in self.cmd_tgt_lock:
            Log.screen("Error: connection not initialized yet")
            return FAIL, "Not connected"

        ret, info = self.__issue_mgmt_cmd__("fe0a", 20, None, None, conn_handle = chandle)   # GAP Terminate Link Request

        self.cmd_mgmt_lock.acquire()
        if chandle in self.conn_handle_addr_dict:
            del self.conn_handle_addr_dict[chandle]
        self.cmd_mgmt_lock.release()

        return ret, info

    # Used for dongle management: initialize, connect to device etc.
    # Input: cmd_id, cmd_parameter*, timeout, callback, 
    # Notice: caller will be blocked here until rsp returned or timeout
    # callback will be invoked for every packet received from dongle
    def __issue_mgmt_cmd__(self, cmd_id, timeout, init_handler, init_data, **kwargs):
        self.cmd_mgmt_lock.acquire()
        if cmd_id in self.cmd_mgmt:
            self.cmd_mgmt_lock.release()
            return FAIL, '10'

        self.cmd_mgmt[cmd_id] = {}
        self.cmd_mgmt[cmd_id]["lock"] = Lock()
        self.cmd_mgmt[cmd_id]["status"] = "CMD_SENT"
        self.cmd_mgmt[cmd_id]["result"] = FAIL
        self.cmd_mgmt[cmd_id]["result_reason"] = "NA"
        if init_handler != None:
            init_handler(self.cmd_mgmt[cmd_id], init_data)
        self.cmd_mgmt[cmd_id]["sched"] = sched.scheduler(time.time, time.sleep)
        self.cmd_mgmt[cmd_id]["sched"].enter(1, 1, self.timeout_handle_dongle_nak, (cmd_id, 1))
        self.cmd_mgmt[cmd_id]["sched"].enter(1 + timeout, 1, dummy, ())
        self.cmd_mgmt_lock.release()

        Log.log("Send mgmt command " + cmd_id)
        try:
            self.ble_builder.send(cmd_id, **kwargs)
        except serial.SerialTimeoutException:
            Log.log("serial write timeout exception")

        self.cmd_mgmt[cmd_id]["sched"].run()

        self.cmd_mgmt_lock.acquire()
        result = self.cmd_mgmt[cmd_id]["result"]
        reason = self.cmd_mgmt[cmd_id]["result_reason"]
        del self.cmd_mgmt[cmd_id]
        self.cmd_mgmt_lock.release()

        return result, reason

    # Used to send command to remove target
    # Input: cmd_id, cmd_parameter*, timeout, callback, 
    # Notice: caller will be blocked here until rsp returned or timeout
    # callback will be invoked for every packet received from dongle
    def __issue_tgt_cmd__(self, chandle, cmd_id, timeout, callback, cmd_mode, **kwargs):
        if not chandle in self.cmd_tgt_lock:
            Log.screen("Error: connection not initialized yet")
            return FAIL, "No connection"

        self.cmd_tgt_lock[chandle].acquire()
        if cmd_id in self.cmd_tgt[chandle]:
            self.cmd_tgt_lock[chandle].release()
            return FAIL, "Command in progress"

        self.cmd_tgt[chandle][cmd_id] = {}
        self.cmd_tgt[chandle][cmd_id]["lock"] = Lock()
        self.cmd_tgt[chandle][cmd_id]["status"] = "CMD_SENT"
        self.cmd_tgt[chandle][cmd_id]["result"] = FAIL
        self.cmd_tgt[chandle][cmd_id]["result_reason"] = None
        self.cmd_tgt[chandle][cmd_id]["cmd_mode"] = cmd_mode
        self.cmd_tgt[chandle][cmd_id]["sched"] = sched.scheduler(time.time, time.sleep)
        self.cmd_tgt[chandle][cmd_id]["sched"].enter(1, 1, self.timeout_handle_dongle_nak, (cmd_id, 2, chandle))
        self.cmd_tgt[chandle][cmd_id]["sched"].enter(1 + timeout, 1, dummy, ())
        self.cmd_tgt_lock[chandle].release()

        Log.log("Send tgt command " + cmd_id)
        try:
            self.ble_builder.send(cmd_id, **kwargs)
        except serial.SerialTimeoutException:
            Log.log("serial write timeout exception")
        self.cmd_tgt[chandle][cmd_id]["sched"].run()

        self.cmd_tgt_lock[chandle].acquire()
        result = self.cmd_tgt[chandle][cmd_id]["result"]
        reason = self.cmd_tgt[chandle][cmd_id]["result_reason"]
        del self.cmd_tgt[chandle][cmd_id]
        self.cmd_tgt_lock[chandle].release()

        return result, reason

    def timeout_handle_dongle_nak(self, cmd_id, cmd_type, conn_handle = None):
        if cmd_type == 1:
            self.cmd_mgmt_lock.acquire()
            if not cmd_id in self.cmd_mgmt:
                Log.screen("BUG: cmd_id " + cmd_id + " not found on self.cmd_mgmt")
                self.cmd_mgmt_lock.release()
                return

            if self.cmd_mgmt[cmd_id]["status"] == "CMD_SENT":
                Log.screen("WARNING: dongle hang without cmd " + cmd_id + " response")

            if self.cmd_mgmt[cmd_id]["result_reason"] == "NA":
                self.cmd_mgmt[cmd_id]["result_reason"] = "Command not accepted by dongle"

            self.cmd_mgmt[cmd_id]["sched"].cancel(self.cmd_mgmt[cmd_id]["sched"].queue[0])
            self.cmd_mgmt_lock.release()
        else:
            self.cmd_tgt_lock[conn_handle].acquire()
            if not cmd_id in self.cmd_tgt[conn_handle]:
                Log.screen("BUG: cmd_id " + cmd_id + " not found on cmd_tgt[" + conn_handle + "]")
                self.cmd_tgt_lock[conn_handle].release()
                return

            if self.cmd_tgt[conn_handle][cmd_id]["status"] == "CMD_SENT":
                Log.screen("WARNING: dongle hang without conn_handle " + conn_handle + " cmd " + cmd_id + " response")

            self.cmd_tgt[conn_handle][cmd_id]["result_reason"] = "Command not accepted by dongle"

            self.cmd_tgt[conn_handle][cmd_id]["sched"].cancel(self.cmd_tgt[conn_handle][cmd_id]["sched"].queue[0])
            self.cmd_tgt_lock[conn_handle].release()

    def read_char(self, chandle, char_uuid, callback):
        lookup_success = 0
        read_handle = None
        # Check dictionary for handle
        self.cmd_mgmt_lock.acquire()
        if chandle in self.conn_handle_addr_dict:
            if char_uuid in self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['read']:
                lookup_success = 1
                read_handle = self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['read'][char_uuid]
        self.cmd_mgmt_lock.release()

        if lookup_success == 0:
            Log.log("discover handle for char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
            ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd88", 60, dummy, SINGLE_READ, conn_handle = chandle, cmd_type = char_uuid.decode('hex')[::-1])
            Log.log("discover handle for char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")

            if ret != SUCCESS:
                return ret, ret_reason

            read_handle = ret_reason[0]['data'][4:6] + ret_reason[0]['data'][2:4]

            # Add result to dictionary
            self.cmd_mgmt_lock.acquire()
            if chandle in self.conn_handle_addr_dict:
                self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['read'][char_uuid] = read_handle
            self.cmd_mgmt_lock.release()

        # FIXME: Use cache
        Log.log("read handle " + binascii.b2a_hex(chandle) + " start")
        ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd8a", 10, callback, SINGLE_READ, conn_handle = chandle, handle = read_handle.decode('hex')[::-1])
        Log.log("read handle " + binascii.b2a_hex(chandle) + " done")
        return ret, ret_reason

    def write_char(self, chandle, char_uuid, data):
        lookup_success = 0
        write_handle = None
        # Check dictionary for handle
        self.cmd_mgmt_lock.acquire()
        if chandle in self.conn_handle_addr_dict:
            if char_uuid in self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['write']:
                lookup_success = 1
                write_handle = self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['write'][char_uuid]
        self.cmd_mgmt_lock.release()

        if lookup_success == 0:
            Log.log("discover handle for char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
            ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd88", 60, dummy, SINGLE_READ, conn_handle = chandle, cmd_type = char_uuid.decode('hex')[::-1])
            Log.log("discover handle for char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")

            if ret != SUCCESS:
                return ret, ret_reason

            write_handle = ret_reason[0]['data'][4:6] + ret_reason[0]['data'][2:4]

            # Add result to dictionary
            self.cmd_mgmt_lock.acquire()
            if chandle in self.conn_handle_addr_dict:
                self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['write'][char_uuid] = write_handle
            self.cmd_mgmt_lock.release()

        # Do the write staff
        Log.log("Write handle: " + write_handle)
        Log.log("write char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
        data_reverse = ''
        n = 2
        for i in range(0, len(data), n):
            data_reverse = data_reverse + data[(len(data) - i - n) : (len(data) - i)]
        ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd92", 10, dummy, SINGLE_WRITE, conn_handle = chandle, handle = write_handle.decode('hex')[::-1], value = data_reverse.decode('hex')[::-1])
        Log.log("write char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")
        return ret, ret_reason

    def handle_util(self, c):
        if c == '0':
            return 0, '1'
        elif c == '1':
            return 0, '2'
        elif c == '2':
            return 0, '3'
        elif c == '3':
            return 0, '4'
        elif c == '4':
            return 0, '5'
        elif c == '5':
            return 0, '6'
        elif c == '6':
            return 0, '7'
        elif c == '7':
            return 0, '8'
        elif c == '8':
            return 0, '9'
        elif c == '9':
            return 0, 'a'
        elif c == 'a':
            return 0, 'b'
        elif c == 'b':
            return 0, 'c'
        elif c == 'c':
            return 0, 'd'
        elif c == 'd':
            return 0, 'e'
        elif c == 'e':
            return 0, 'f'
        elif c == 'f':
            return 1, '0'

    def get_notification_handle(self, handle):
        carry, cl = self.handle_util(handle[1:2])
        if carry == 0:
            return handle[0:1] + cl
        else:
            carry, ch = self.handle_util(handle[0:1])
            return ch + cl

    def notification_char_enable(self, chandle, char_uuid, callback):
        lookup_success = 0
        write_handle = None
        # Check dictionary for handle
        self.cmd_mgmt_lock.acquire()
        if chandle in self.conn_handle_addr_dict:
            if char_uuid in self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['notification']:
                lookup_success = 1
                write_handle = self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['notification'][char_uuid]
        self.cmd_mgmt_lock.release()

        if lookup_success == 0:
            Log.log("discover notification handle char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
            ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd88", 60, dummy, SINGLE_READ, conn_handle = chandle, cmd_type = char_uuid.decode('hex')[::-1])
            Log.log("discover notification handle char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")

            if ret != SUCCESS:
                return ret, ret_reason

            write_handle = ret_reason[0]['data'][4:6] + self.get_notification_handle(ret_reason[0]['data'][2:4])

            # Add result to dictionary
            self.cmd_mgmt_lock.acquire()
            if chandle in self.conn_handle_addr_dict:
                self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['notification'][char_uuid] = write_handle
            self.cmd_mgmt_lock.release()
        
        # Do the write staff
        Log.log("Write 0x01 to handle: " + write_handle)

        Log.log("Enable notification " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
        ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd92", 10, dummy, SINGLE_WRITE, conn_handle = chandle, handle = write_handle.decode('hex')[::-1], value = '\x01\x00')
        Log.log("Enable notification " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")
        return ret, ret_reason

    def notification_char_disable(self, chandle, char_uuid):
        lookup_success = 0
        write_handle = None
        # Check dictionary for handle
        self.cmd_mgmt_lock.acquire()
        if chandle in self.conn_handle_addr_dict:
            if char_uuid in self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['notification']:
                lookup_success = 1
                write_handle = self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['notification'][char_uuid]
        self.cmd_mgmt_lock.release()

        if lookup_success == 0:
            Log.log("discover notification handle char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
            ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd88", 60, dummy, SINGLE_READ, conn_handle = chandle, cmd_type = char_uuid.decode('hex')[::-1])
            Log.log("discover notification handle char uuid " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")

            if ret != SUCCESS:
                return ret, ret_reason

            write_handle = ret_reason[0]['data'][4:6] + self.get_notification_handle(ret_reason[0]['data'][2:4])
        
            # Add result to dictionary
            self.cmd_mgmt_lock.acquire()
            if chandle in self.conn_handle_addr_dict:
                self.addr_handle_dict[self.conn_handle_addr_dict[chandle]]['notification'][char_uuid] = write_handle
            self.cmd_mgmt_lock.release()

        # Do the write staff
        Log.log("Write 0x00 to handle: " + write_handle)

        Log.log("Disable notification " + binascii.b2a_hex(chandle) + "/" + char_uuid + " start")
        ret, ret_reason = self.__issue_tgt_cmd__(chandle, "fd92", 10, dummy, SINGLE_WRITE, conn_handle = chandle, handle = write_handle.decode('hex')[::-1], value = '\x00\x00')
        Log.log("Disable notification " + binascii.b2a_hex(chandle) + "/" + char_uuid + " done")
        return ret, ret_reason

