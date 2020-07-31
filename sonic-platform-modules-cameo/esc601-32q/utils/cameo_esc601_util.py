#!/usr/bin/env python
#
# Copyright (C) 2019 Cameo Networks, Inc.


"""
Usage: %(scriptName)s [options] command object

options:
    -h | --help     : this help message
    -d | --debug    : run with debug mode
    -f | --force    : ignore error during installation or clean 
command:
    install     : install drivers and generate related sysfs nodes
    clean       : uninstall drivers and remove related sysfs nodes
    show        : show all systen status
    sff         : dump SFP eeprom
    set         : change board setting with fan|led|sfp    
"""

import os
import commands
import sys, getopt
import logging
import re
import time
PROJECT_NAME = 'esc601_32q'
version = '0.1.0'
verbose = False
DEBUG = False
args = []
ALL_DEVICE = {}
DEVICE_NO = {'led': 5, 'fan': 1, 'thermal': 3, 'psu': 2, 'sfp': 54}
FORCE = 0
# logging.basicConfig(filename= PROJECT_NAME+'.log', filemode='w',level=logging.DEBUG)
# logging.basicConfig(level=logging.INFO)


if DEBUG == True:
    print sys.argv[0]
    print 'ARGV      :', sys.argv[1:]


def main():
    global DEBUG
    global args
    global FORCE

    if len(sys.argv) < 2:
        show_help()

    options, args = getopt.getopt(sys.argv[1:], 'hdf', ['help',
                                                        'debug',
                                                        'force',
                                                        ])
    if DEBUG == True:
        print options
        print args
        print len(sys.argv)

    for opt, arg in options:
        if opt in ('-h', '--help'):
            show_help()
        elif opt in ('-d', '--debug'):
            DEBUG = True
            logging.basicConfig(level=logging.INFO)
        elif opt in ('-f', '--force'):
            FORCE = 1
        else:
            logging.info('no option')
    for arg in args:
        if arg == 'install':
            do_install()
        elif arg == 'clean':
            do_uninstall()
        elif arg == 'show':
            device_traversal()
        elif arg == 'sff':
            if len(args) != 2:
                show_eeprom_help()
            elif int(args[1]) == 0 or int(args[1]) > DEVICE_NO['sfp']:
                show_eeprom_help()
            else:
                show_eeprom(args[1])
            return
        elif arg == 'set':
            if len(args) < 3:
                show_set_help()
            else:
                set_device(args[1:])
            return
        else:
            show_help()

    return 0


def show_help():
    print __doc__ % {'scriptName': sys.argv[0].split("/")[-1]}
    sys.exit(0)


def show_set_help():
    cmd = sys.argv[0].split("/")[-1] + " " + args[0]
    print  cmd + " [led|sfp|fan]"
    print  "    use \"" + cmd + " led 0-4 \"  to set led color"
    print  "    use \"" + cmd + " fan 0-100\" to set fan duty percetage"
    print  "    use \"" + cmd + " sfp 1-54 {0|1}\" to set sfp# tx_disable"
    sys.exit(0)


def show_eeprom_help():
    cmd = sys.argv[0].split("/")[-1] + " " + args[0]
    print  "    use \"" + cmd + " 1-54 \" to dump sfp# eeprom"
    sys.exit(0)


def my_log(txt):
    if DEBUG == True:
        print "[ROY]" + txt
    return


def log_os_system(cmd, show):
    logging.info('Run :' + cmd)
    status, output = commands.getstatusoutput(cmd)
    my_log(cmd + "with result:" + str(status))
    my_log("      output:" + output)
    if status:
        logging.info('Failed :' + cmd)
        if show:
            print('Failed :' + cmd)
    return status, output


def driver_check():
    ret, lsmod = log_os_system("lsmod| grep cameo", 0)
    logging.info('mods:' + lsmod)
    if len(lsmod) == 0:
        return False
    return True


kos = [
    'depmod -a',
    'modprobe i2c_dev',
    'modprobe x86-64-cameo-esc601-32q',
    'modprobe lm90',
    'modprobe nct7511',
    'modprobe mcp3425_smbus',
    'modprobe at24_smbus',
    'modprobe at24',
    'modprobe zrh2800k2',
    'modprobe tps53679'
]


def driver_install():
    global FORCE
    for i in range(0, len(kos)):
        status, output = log_os_system(kos[i], 1)
        if status:
            if FORCE == 0:
                return status
    return 0


def driver_uninstall():
    global FORCE
    for i in range(0, len(kos)):
        rm = kos[-(i + 1)].replace("modprobe", "modprobe -rq")
        rm = rm.replace("insmod", "rmmod")
        status, output = log_os_system(rm, 1)
        if status:
            if FORCE == 0:
                return status
    return 0


i2c_prefix = '/sys/bus/i2c/devices/'
sfp_map = [9, 10, 11, 12, 13, 14, 15, 16,
           17, 18, 19, 20, 21, 22, 23, 24,
           25, 26, 27, 28, 29, 30, 31, 32,
           33, 34, 35, 36, 37, 38, 39, 40]
mknod = [
    # enable port led stream
    'echo 1 > /sys/class/hwmon/hwmon2/device/ESC601_LED/led_ctrl',
    #
    'echo pca9548 0x73 > /sys/bus/i2c/devices/i2c-0/new_device',
    # Port1--Port32 QSFP EEPROM
    'echo pca9548 0x74 > /sys/bus/i2c/devices/i2c-5/new_device',
    'echo pca9548 0x74 > /sys/bus/i2c/devices/i2c-6/new_device',
    'echo pca9548 0x74 > /sys/bus/i2c/devices/i2c-7/new_device',
    'echo pca9548 0x74 > /sys/bus/i2c/devices/i2c-8/new_device',

    'echo pca9548 0x77 > /sys/bus/i2c/devices/i2c-0/new_device',
    # G781 sensors
    'echo g781 0x4c > /sys/bus/i2c/devices/i2c-42/new_device', 
    'echo g781 0x4c > /sys/bus/i2c/devices/i2c-43/new_device',
    'echo g781 0x4c > /sys/bus/i2c/devices/i2c-44/new_device',
    # NCT7511Y sensor & fan control
    'echo nct7511 0x2e > /sys/bus/i2c/devices/i2c-41/new_device',
	# mcp3425 adc
	'echo mcp3425_smbus 0x68 > /sys/bus/i2c/devices/i2c-47/new_device',

    'echo 24c64smbus 0x56 > /sys/bus/i2c/devices/i2c-0/new_device',
    # zrh2800k2 psu
    'echo zrh2800k2 0x58 > /sys/bus/i2c/devices/i2c-45/new_device',
    'echo zrh2800k2 0x59 > /sys/bus/i2c/devices/i2c-45/new_device',
    # tps53681
    'echo tps53679 0x6C > /sys/bus/i2c/devices/i2c-46/new_device',
    'echo tps53679 0x6E > /sys/bus/i2c/devices/i2c-46/new_device',
    'echo tps53679 0x70 > /sys/bus/i2c/devices/i2c-46/new_device'
]
def get_attr_value(attr_path):
    retval = 'ERR'
    if not os.path.isfile(attr_path):
        return retval

    try:
        with open(attr_path, 'r') as fd:
            retval = fd.read()
    except Exception as error:
        logging.error("Unable to open ", attr_path, " file !")

    retval = retval.rstrip('\r\n')
    fd.close()
    return retval

def bmc_is_exist():
    value = ''
    bmc_filePath = '/sys/class/hwmon/hwmon2/device/ESC601_BMC/bmc_present'
    if os.path.exists(bmc_filePath):
       value = get_attr_value(bmc_filePath)
       if value.find('not') < 0:
            return True
       else:
            return False
    else:
       return False

def is_24c04():
    value = ''
    filePath = '/sys/class/hwmon/hwmon2/device/ESC601_SYS/hw_version'
    if os.path.exists(filePath):
       value = get_attr_value(filePath)
       if int(value[-4:],16) < 0x10:
            return True
       else:
            return False
    else:
       return False

def device_install():
    global FORCE
    
    if bmc_is_exist():
       mknod.remove('echo pca9548 0x77 > /sys/bus/i2c/devices/i2c-0/new_device')
       mknod.remove('echo g781 0x4c > /sys/bus/i2c/devices/i2c-42/new_device')
       mknod.remove('echo g781 0x4c > /sys/bus/i2c/devices/i2c-43/new_device')
       mknod.remove('echo g781 0x4c > /sys/bus/i2c/devices/i2c-44/new_device')
       mknod.remove('echo nct7511 0x2e > /sys/bus/i2c/devices/i2c-41/new_device')
       mknod.remove('echo mcp3425_smbus 0x68 > /sys/bus/i2c/devices/i2c-47/new_device')
       mknod.remove('echo zrh2800k2 0x58 > /sys/bus/i2c/devices/i2c-45/new_device')
       mknod.remove('echo zrh2800k2 0x59 > /sys/bus/i2c/devices/i2c-45/new_device')
       mknod.remove('echo tps53679 0x6C > /sys/bus/i2c/devices/i2c-46/new_device')
       mknod.remove('echo tps53679 0x6E > /sys/bus/i2c/devices/i2c-46/new_device')
       mknod.remove('echo tps53679 0x70 > /sys/bus/i2c/devices/i2c-46/new_device')

    if is_24c04():
       mknod.remove('echo 24c64smbus 0x56 > /sys/bus/i2c/devices/i2c-0/new_device') 
       mknod.append('echo 24c04 0x56 > /sys/bus/i2c/devices/i2c-0/new_device')
       if not bmc_is_exist():
           mknod.remove('echo tps53679 0x6C > /sys/bus/i2c/devices/i2c-46/new_device')
           mknod.append('echo tps53679 0x6C > /sys/bus/i2c/devices/i2c-4/new_device')
           mknod.remove('echo tps53679 0x6E > /sys/bus/i2c/devices/i2c-46/new_device')
           mknod.append('echo tps53679 0x6E > /sys/bus/i2c/devices/i2c-4/new_device')
           mknod.remove('echo tps53679 0x70 > /sys/bus/i2c/devices/i2c-46/new_device')
           mknod.append('echo tps53679 0x70 > /sys/bus/i2c/devices/i2c-4/new_device')


 
    for i in range(0, len(mknod)):
        # for pca954x need times to built new i2c buses
        if mknod[i].find('pca954') != -1:
            time.sleep(1)

        status, output = log_os_system(mknod[i], 1)
        if status:
            print output
            if FORCE == 0:
                return status
    for i in range(0, len(sfp_map)):
        status, output = log_os_system("echo optoe1 0x50 > /sys/bus/i2c/devices/i2c-" + str(sfp_map[i]) + "/new_device",1)
        if status:
            print output
            if FORCE == 0:
                return status
    return


def device_uninstall():
    global FORCE
    nodelist = mknod

    for i in range(len(nodelist)):
        target = nodelist[-(i + 1)]
        temp = target.split()
        del temp[1]
        temp[-1] = temp[-1].replace('new_device', 'delete_device')
        status, output = log_os_system(" ".join(temp), 1)
        if status:
            print output
            if FORCE == 0:
                return status

    return


def system_ready():
    if driver_check() == False:
        return False
    if not device_exist():
        return False
    return True


def do_install():
    print "Checking system...."
    if driver_check() == False:
        print "No driver, installing...."
        status = driver_install()
        if status:
            if FORCE == 0:
                return status
    else:
        print PROJECT_NAME.upper() + " drivers detected...."
    if not device_exist():
        print "No device, installing...."
        status = device_install()
        if status:
            if FORCE == 0:
                return status
    else:
        print PROJECT_NAME.upper() + " devices detected...."
    return


def do_uninstall():
    print "Checking system...."
    if not device_exist():
        print PROJECT_NAME.upper() + " has no device installed...."
    else:
        print "Removing device...."
        status = device_uninstall()
        if status:
            if FORCE == 0:
                return status

    if driver_check() == False:
        print PROJECT_NAME.upper() + " has no driver installed...."
    else:
        print "Removing installed driver...."
        status = driver_uninstall()
        if status:
            if FORCE == 0:
                return status

    return


def devices_info():
    return


def show_eeprom(index):
    return


def set_device(args):
    return


# get digits inside a string.
# Ex: 31 for "sfp31"
def get_value(input):
    digit = re.findall('\d+', input)
    return int(digit[0])


def device_traversal():
    attr_path = '/sys/class/hwmon/hwmon2/device/ESC601_Sensor/sensor_temp'
    try:
        reg_file = open(attr_path, 'r')
    except IOError as e:
        print "Error: unable to open file: %s" % str(e)
        return False
    print("Thermal sensors(1-4) temperature:\n")
    for line in reg_file.readlines():
        print line
    reg_file.close()
    return


def device_exist():
    ret1, log = log_os_system("ls " + i2c_prefix + "*0056", 0)
    return not ret1


if __name__ == "__main__":
    main()
