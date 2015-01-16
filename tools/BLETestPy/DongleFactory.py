import wx
import wx.lib.platebtn as platebtn
import itertools
import _winreg
import _winreg as reg
from itertools import count
import wmi
import re
import serial

class DongleFactory:      
    def register(self, methodName, constructor, *args, **kargs):
        """register a constructor"""
        _args = [constructor]
        _args.extend(args)
        setattr(self, methodName,apply(DongleFunctor,_args, kargs))
        
    def unregister(self, methodName):
        """unregister a constructor"""
        delattr(self, methodName)

class DongleFunctor:
    def __init__(self, function, *args, **kargs):
        assert callable(function), "function should be a callable obj"
        self._function = function
        self._args = args
        self._kargs = kargs
        
    def __call__(self, *args, **kargs):
        """call function"""

        # Check whether serial port available
        port_name = ""
        if len(args) <= 1:
            pattern = re.compile('COM[0-9]+')
            ti_ble = re.compile('CC2540')
            c = wmi.WMI()
            wql = "Select * From Win32_USBControllerDevice"
            for item in c.query(wql):
                if ti_ble.search(item.Dependent.Caption) != None and pattern.search(item.Dependent.Caption) != None:
                    port_name = pattern.search(item.Dependent.Caption).group()
        else:
            port_name = args[1]

        if port_name == "":
            return None

        serial_dongle = None
        try:
            serial_dongle = serial.Serial(port = port_name, baudrate = 115200)
        except serial.SerialException:
            pass
        except UnicodeDecodeError:
            pass

        if serial_dongle == None:
            return None

        serial_dongle.close()

        _args = list(self._args)
        # Add port name here if not provided
        if len(args) == 1:
            arg_one = args[0]
            args = [arg_one, port_name]
        _args.extend(args)
        _kargs = self._kargs.copy()
        _kargs.update(kargs)
        return apply(self._function,_args,_kargs)

