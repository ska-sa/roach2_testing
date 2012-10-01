#!/usr/bin/python

import os, sys, time, select, termios, tty, ftdi, subprocess, serial, logging, telnetlib, shutil
from optparse import OptionParser
from mpsse import *
import i2c_functions as iicf
import xmodem_tx as xtx
import defs_max16071, defs_max1805, defs_ad7414
import defs_r2_ats as defs
from bcolours import bcolours as c
import qdr_tst
import corr

def config_mon():
  try:
    i2c_bus = open_ftdi_b()
  except:
    raise
  try:
    if check_ppc_i2c():
      print "        Setting current monitor:"
      print "        Setting software enable configuration...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.SW_EN_CONF, defs_max16071.SW_EN_CONF_VAL))
      print "        Setting overcurrent primary threshold and current-sense control...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.OCPT_CSC1, defs_max16071.OCPT_CSC1_VAL))
      print "        Setting full-scale range for ADCs 1-4...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.ADC_CONF_4321, defs_max16071.ADC_CONF_4321_VAL))
      print "        Setting full-scale range for ADCs 5-8...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.ADC_CONF_8765, defs_max16071.ADC_CONF_8765_VAL))
      print "        Setting voltage monitor:"
      print "        Setting software enable configuration...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.SW_EN_CONF, defs_max16071.SW_EN_CONF_VAL))
      print "        Setting overcurrent primary threshold and current-sense control...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.OCPT_CSC1, defs_max16071.OCPT_CSC1_VAL))
      print "        Setting full-scale range for ADCs 1-4...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.ADC_CONF_4321, defs_max16071.ADC_CONF_4321_VAL))
      print "        Setting full-scale range for ADCs 5-8...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.ADC_CONF_8765, defs_max16071.ADC_CONF_8765_VAL))
    else:
      raise Exception('ERROR: I2C bus could not be secured from the PPC (check PPC state), current and voltage monitors not set.')
  finally:
    i2c_bus.Close()

def read_voltage(vbus, i2c_bus):
  ch = defs.V_MON_MAP[vbus][0]
  addr = defs.V_MON_MAP[vbus][1]
  ch_idx = (ch - 1)*2
  msb = defs_max16071.CH_ARRAY[ch_idx]
  lsb = defs_max16071.CH_ARRAY[ch_idx+1]
  voltage = ((iicf.i2c_regread(i2c_bus, addr , msb)) << 2) + ((iicf.i2c_regread(i2c_bus, addr, lsb)) >> 6)
  # get full scale voltage for requested channel
  if ch < 5:
    adc_conf = iicf.i2c_regread(i2c_bus, addr, defs_max16071.ADC_CONF_4321)
    fs_v = defs_max16071.ADC_RNG[(adc_conf >> ch_idx) & 0x03]
  else:
    adc_conf = iicf.i2c_regread(i2c_bus, addr, defs_max16071.ADC_CONF_8765)
    fs_v = defs_max16071.ADC_RNG[(adc_conf >> (ch_idx - 8)) & 0x03]
  # if 12V monitor apply voltage divider
  if vbus == '12V':
    volts = (voltage/1024.0)*fs_v*defs_max16071.V_DIV_12V
  else:
    volts = (voltage/1024.0)*fs_v
  return volts

def read_current(vbus, i2c_bus):
  ch = defs.C_MON_MAP[vbus]
  ch_idx = (ch - 1)*2
  msb = defs_max16071.CH_ARRAY[ch_idx]
  lsb = defs_max16071.CH_ARRAY[ch_idx+1]
  addr = iicf.ADDR_C_MON
  voltage = ((iicf.i2c_regread(i2c_bus, addr , msb)) << 2) + ((iicf.i2c_regread(i2c_bus, addr, lsb)) >> 6)
  # get full scale voltage for requested channel
  if ch < 5:
    adc_conf = iicf.i2c_regread(i2c_bus, addr, defs_max16071.ADC_CONF_4321)
    fs_v = defs_max16071.ADC_RNG[(adc_conf >> ch_idx) & 0x03]
  else:
    adc_conf = iicf.i2c_regread(i2c_bus, addr, defs_max16071.ADC_CONF_8765)
    fs_v = defs_max16071.ADC_RNG[(adc_conf >> (ch_idx - 8)) & 0x03]
  res = defs_max16071.SNS_RES[ch - 1]
  # Select gain resistor according to ROACH2 revision
  if REV == 1:
    gain = 1 + (1e5/defs_max16071.GAIN_RES_REV1[ch - 1])
  else:
    gain = 1 + (1e5/defs_max16071.GAIN_RES_REV2[ch - 1])
  amps = ((voltage/1024.0)*fs_v)/(res*gain)
  return amps

# read max16071 onboard current
def read_ob_current(vbus, i2c_bus):
  if vbus == '5V0':
    addr = iicf.ADDR_C_MON
    res = defs_max16071.SNS_RES_5V0
  else: #12V current
    addr = iicf.ADDR_V_MON
    res = defs_max16071.SNS_RES_12V
  voltage = iicf.i2c_regread(i2c_bus, addr, defs_max16071.MON_C)
  #Get current-sense gain setting
  curr_sns_gn = defs_max16071.CURR_SNS_GAIN[((iicf.i2c_regread(i2c_bus, addr, defs_max16071.OCPT_CSC1)) >> 2) & 0x03]
  amps = (((voltage/255.0)*1.4)/curr_sns_gn)/res
  return amps

def read_vmon_gpio(gpio):
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      ch = defs.V_MON_GPIO[gpio] - 1
      addr = iicf.ADDR_V_MON
      gpio_rd = iicf.i2c_regread(i2c_bus, addr, defs_max16071.GPIO_INPUT_STATE)
      val = (gpio_rd >> ch) & 0x01
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), voltage monitor gpio not read.')
  return val

def check_currents(dic):
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      c_err = []
      for i, v in dic.iteritems():
        vbus = i[:-2]
        if (vbus == '12V') | (vbus == '5V0'):
          amps = read_ob_current(vbus, i2c_bus)
        else:
          amps = read_current(vbus, i2c_bus)
        if i[-1] == 'H':
          if amps > v:
            c_err.append(i)
            c_err.append(amps)
        else:
          if amps < v:
            c_err.append(i)
            c_err.append(amps)
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), currents not read.')
  return c_err

def check_voltages():
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      v_err = []
      # Select voltage profile according to ROACH2 Revision
      if REV == 1:
        dic = defs.V_THRESHOLD_REV1
      else:
        dic = defs.V_THRESHOLD_REV2
      for i, v in dic.iteritems():
        volts = read_voltage(i[:-2], i2c_bus)
        if i[-1] == 'H':
          if volts > v:
            v_err.append(i)
            v_err.append(volts)
        else:
          if volts < v:
            v_err.append(i)
            v_err.append(volts)
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), voltages not read.')
  return v_err

def print_v_c():
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      print "    1V0 Bus: %.3fV, %.3fA" %(read_voltage('1V0', i2c_bus), read_current('1V0', i2c_bus))
      print "    1V5 Bus: %.3fV, %.3fA" %(read_voltage('1V5', i2c_bus), read_current('1V5', i2c_bus))
      print "    1V8 Bus: %.3fV, %.3fA" %(read_voltage('1V8', i2c_bus), read_current('1V8', i2c_bus))
      print "    2V5 Bus: %.3fV, %.3fA" %(read_voltage('2V5', i2c_bus), read_current('2V5', i2c_bus))
      print "    3V3 Bus: %.3fV, %.3fA" %(read_voltage('3V3', i2c_bus), read_current('3V3', i2c_bus))
      print "    5V0 Bus: %.3fV, %.3fA" %(read_voltage('5V0', i2c_bus), read_ob_current('5V0', i2c_bus))
      if REV == 1:
        # MAX16071 mod puts 12v line on 3v3aux monitor
        print "    12V Bus: %.3fV, %.3fA" %((read_voltage('3V3_AUX', i2c_bus)*defs_max16071.V_DIV_12V), read_ob_current('12V', i2c_bus))
        print "    3V3 Aux: not avialable on revision 1"
      else:
        print "    12V Bus: %.3fV, %.3fA" %(read_voltage('12V', i2c_bus), read_ob_current('12V', i2c_bus))
        print "    3V3 Aux: %.3fV" %read_voltage('3V3_AUX', i2c_bus)
        print "    5V0 Aux: %.3fV" %read_voltage('5V0_AUX', i2c_bus)
      print "    MGT 1.2V Power Good = %d" %read_vmon_gpio('MGT_1V2_PG')
      print "    MGT 1.0V Power Good = %d" %read_vmon_gpio('MGT_1V0_PG')
      print ""
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), voltages and currents not read.')

def read_temp(sensor, i2c_bus):
  if sensor == 'PPC':
    temp = iicf.i2c_regread(i2c_bus, iicf.ADDR_PPC_FPGA_TEMP, defs_max1805.RET1)
  elif sensor == 'FPGA':
    temp = iicf.i2c_regread(i2c_bus, iicf.ADDR_PPC_FPGA_TEMP, defs_max1805.RET2)
  elif sensor == 'INLET':
    temp_reg = iicf.i2c_regread2b(i2c_bus, iicf.ADDR_INLET_TEMP, defs_ad7414.TEMP)
    temp = ((temp_reg[0]<<2)+(temp_reg[1]>>6))*0.25
  elif sensor == 'OUTLET':
    temp_reg = iicf.i2c_regread2b(i2c_bus, iicf.ADDR_OUTLET_TEMP, defs_ad7414.TEMP)
    temp = ((temp_reg[0]<<2)+(temp_reg[1]>>6))*0.25
  return temp

def check_temps():
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      t_err = []
      for i, v in defs.T_THRESHOLD.iteritems():
        temp = read_temp(i[:-4], i2c_bus)
        if i[-1] == 'H':
          if temp > v:
            t_err.append(i)
            t_err.append(temp)
        else:
          if temp < v:
            t_err.append(i)
            t_err.append(temp)
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), temperatures not read.')
  return t_err

def print_temps():
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      print '    PPC Temp: %d degreesC' %read_temp('PPC', i2c_bus)
      print '    FPGA Temp: %d degreesC' %read_temp('FPGA', i2c_bus)
      print '    Inlet Temp: %0.2f degreesC' %read_temp('INLET', i2c_bus)
      print '    Inlet Temp: %0.2f degreesC' %read_temp('OUTLET', i2c_bus)
      print ''
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), temperatures not read.')

#To select menu item without pressing enter, a timeout is added so that the menu loop is not blocked.
def isdata():
    return select.select([sys.stdin], [], [], 1) == ([sys.stdin], [], [])

def getkey():
  old_settings = termios.tcgetattr(sys.stdin)
  answer = None
  try:
    tty.setraw(sys.stdin.fileno())
    if isdata():
      answer = sys.stdin.read(1)
    return answer
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def getkey_block():
  old_settings = termios.tcgetattr(sys.stdin)
  try:
    tty.setraw(sys.stdin.fileno())
    answer = sys.stdin.read(1)
    return answer
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def set_serial_number():
  if SEL_MANUF:
    print "    Select manufacturer (default = Digicom):"
    for i, v in enumerate(defs.MANUF):
      print ("    %d) %s" %(i+1, v))
    answer = getkey_block()
    try:
      ans = int(answer)
      if ans in range(1, i+2):
        manuf = defs.MANUF.values()[ans-1]
      else:
        manuf = defs.MANUF['Digicom']
    except ValueError:
      manuf = defs.MANUF['Digicom']
  else:
    manuf = defs.MANUF['Digicom']
  done = False
  while not done:
    batch = raw_input("    Batch Number (0-255): ")
    try:
      batch = int(batch)
      if batch not in range(256):
        print ('    Input valid batch number') 
      else:
        done = True
    except ValueError:
      print ('    Input valid batch number')
  done = False
  while not done:
    board = raw_input("    Board Number (0-255): ")
    try:
      board = int(board)
      if board not in range(256):
        print ('    Input valid board number') 
      else:
        done = True
    except ValueError:
      print ('    Input valid board number')
  sn = {     
        'manufacturer' : manuf, 
        'type'         : 0x01, 
        'revision'     : REV,
        'batch'        : batch, 
        'board'        : board 
  } 
  manuf_id = sn['manufacturer']
  manuf_name = find_key(defs.MANUF, manuf_id)
  print '\n    Selected serial number:'
  print '        Manufacturer ID: %s (%s)' %(chr(manuf_id), manuf_name)
  print '        Type: %s (ROACH2)' %sn['type']
  print '        Revision: %s ' %sn['revision']
  print '        Batch: %s ' %sn['batch']
  print '        Board: %s ' %sn['board']
  print ''
  return sn



def program_eeprom(sn):
  if check_ppc_i2c():
    try:
      i2c_bus = open_ftdi_b()
    except:
      raise
    try:
      print '    Writing boot params for config G boot to EEPROM locations 0x00 to 0x0f...',
      for i, v in enumerate(defs.CONFIG_G_BOOT):
        iicf.i2c_regwrite(i2c_bus, iicf.ADDR_BOOT_EEPROM_0, i, v)
        time.sleep(0.005)
      print 'done.'
      print '    Writing serial number to EEPROM locations 0x10 to 0x14...',
      sn_array = []
      sn_array.append(sn['manufacturer'])
      sn_array.append(sn['type'])
      sn_array.append(sn['revision'])
      sn_array.append(sn['batch'])
      sn_array.append(sn['board'])
      for i in range (0x10,0x15):
        iicf.i2c_regwrite(i2c_bus, iicf.ADDR_BOOT_EEPROM_0, i, sn_array[i - 0x10])
        time.sleep(0.005)
      print 'done.'
      print '    Verifying EEPROM contents...',
      ver = []
      for i in range(0x15):
        ver.append(iicf.i2c_regread(i2c_bus, iicf.ADDR_BOOT_EEPROM_0, i))
      if (ver[0:0x10] <> defs.CONFIG_G_BOOT) | (ver[0x10:0x15] <> sn_array):
        raise Exception, ('EEPROM0 contents not read back correctly: \n%s' %ver)
      print 'done.'
      print ''
    finally:
      i2c_bus.Close()
  else:
    raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), EEPROM write not done.')


def find_key(dic, val):
    """return the key of dictionary dic given the value"""
    return [k for k, v in dic.iteritems() if v == val][0]

def open_ftdi_d():
  f = ftdi.ftdi_context()
  ftdi.ftdi_init(f)
  res = ftdi.ftdi_set_interface(f, ftdi.INTERFACE_D)
  if res <> 0:
    raise Exception('FTDI Interface ERROR: %s' %ftdi_int_err[res])
  res = ftdi.ftdi_usb_open(f, defs.R2_VID, defs.R2_PID)
  if res <> 0:
    raise Exception('FTDI Open ERROR: %s' %ftdi_open_err[res])
  return f

def disable_ftdi_a():
  f = ftdi.ftdi_context()
  ftdi.ftdi_init(f)
  res = ftdi.ftdi_set_interface(f, ftdi.INTERFACE_A)
  if res <> 0:
    raise Exception('FTDI Interface ERROR: %s' %ftdi_int_err[res])
  res = ftdi.ftdi_usb_open(f, defs.R2_VID, defs.R2_PID)
  if res <> 0:
    raise Exception('FTDI Open ERROR: %s' %ftdi_open_err[res])
  # Set all lins on interface A as inputs.
  res = ftdi.ftdi_set_bitmode(f, 0x00, ftdi.BITMODE_BITBANG)
  if res <> 0:
    raise Exception('FTDI bitmode set ERROR: %s' %ftdi_bit_err[res])
  ftdi.ftdi_usb_close(f)

def open_ftdi_b():
  try:
    i2c_bus = MPSSE()
    i2c_bus.Open(defs.R2_VID, defs.R2_PID, I2C, ONE_HUNDRED_KHZ, MSB, IFACE_B, None)
    return i2c_bus
  except Exception as e:
    if e.message == 'device not found':
      print '\nERROR: Could not connect to I2C bus, check USB connection.'
    raise

# This method tries to write to the ftdi chip the specified amount, it seems that on some boards more than one write
# is required. Data is one byte: '\xx', returns 1 if successfull, otherwise error code.
def ftdi_write(interface, data, retries = 3):
  retry = 0
  while retry < retries:
    res = ftdi.ftdi_write_data(interface, data, 1)
    if res == 1:
      retry = retries
    else:
      print c.WARNING + 'WARNING: FTDI write failed, retrying.' + c.ENDC
      retry += 1
      time.sleep(0.5)
  return res

def power_force(ftdi_obj, state):
    # Rev1: force power on: PCTRL_EN (bit5), PCTRL_ONn (bit4)
    # Rev2: force power on: PCTRL_EN (bit5), PCTRL_ON (bit4) 
    rev1_on = '\x20'
    rev1_off = '\x30'
    rev2_on = '\x30'
    rev2_off = '\x20'
    if REV == 1 and state == 'on':
      byte = rev1_on
    elif REV == 1 and state == 'off':
      byte = rev1_off
    elif REV == 2 and state == 'on':
      byte = rev2_on
    else:
      byte = rev2_off
    return ftdi_write(ftdi_obj, byte)

def test_power():
  try:
    f = open_ftdi_d()
  except:
    raise
  try:
    print '    Testing power force on.'
    # Rev1: enable PCTRL_EN and PCTRL_ONn lines on ftdi interface D
    # Rev2: enable PCTRL_EN and PCTRL_ON lines on ftdi interface D
    res = ftdi.ftdi_set_bitmode(f, 0x30, ftdi.BITMODE_BITBANG)
    if res <> 0:
      raise Exception('FTDI bitmode set ERROR: %s' %ftdi_bit_err[res])
    
    # Forcing board on and checking ATX power good
    print '        Forcing board on...', 
    res = power_force(f, 'on')
    if res <> 1:
      raise Exception('ERROR: FTDI write error, code: %d' %res)
    time.sleep(defs.PG_DELAY)
    # check ATX_PWR_OK
    if not read_vmon_gpio('ATX_PWR_OK'):
      raise Exception('FATAL: Board did not power up on power force on.')
    print 'done.'

    # Checking voltages
    print '    Checking voltage tolerances...',
    v_err = check_voltages()
    if v_err:
      print 'FATAL: Voltage error detected, forcing board off'
      res = power_force(f, 'off')
      if res <> 1:
        raise Exception('FATAL: Voltages out of range: %s, AND FTDI write ERROR, code: %d, WARNING: Board did not switch off, switch off manually!' %(v_err, res))
      raise Exception, ('FATAL: Voltages out of range: %s' %v_err)
    print 'done.'
    print '    Waiting 1 seconds for currents to settle...',
    sys.stdout.flush()
    time.sleep(1)
    print 'done.'

    # Checking currents
    print '    Checking unconfigured current tolerances...',
    c_err = check_currents(defs.UC_C_THRESHOLD)
    if c_err:
      print 'FATAL: Unconfigured current tolerance error detected, forcing board off'
      res = power_force(f, 'off')
      if res <> 1:
        raise Exception('FATAL: Currents out of range: %s, AND FTDI write ERROR, code: %d, WARNING: Board did not switch off, switch off manually!' %(c_err, res))
      raise Exception, ('FATAL: Currents out of range: %s' %c_err)
    print 'done.'

    # Checking temperatures
    print '    Checking temperatures...',
    t_err = check_temps()
    if t_err:
      print 'FATAL: Temperature sensor\s out of range, forcing board off'
      res = power_force(f, 'off')
      if res <> 1:
        raise Exception('FATAL: Temperature sensor\s out of range: %s, AND FTDI write ERROR, code: %d, WARNING: Board did not switch off, switch off manually!' %(t_err, res))
      raise Exception, ('FATAL: Temperature sensor\s out of range: %s' %t_err)
    print 'done.'
    print_v_c()
    print_temps()

    # Forcing board off and checking ATX power good   
    print '    Forcing board off...',
    res = power_force(f, 'off')
    if res <> 1:
      raise Exception('ERROR: FTDI write error, code: %d' %res)
    time.sleep(defs.PG_DELAY)
    # check ATX_PWR_OK
    if read_vmon_gpio('ATX_PWR_OK'):
      raise Exception('FATAL: Board did not power down on power force off.')
    print 'done.'

    # Restoring normal power control
    print '    Disabling power force...',
    if ftdi_write(f, '\x00') <> 1:
      raise Exception('ERROR: FTDI write error, code: %d' %res)

    # Enabling PB_CTRL (power button) and simulating presses
    res = ftdi.ftdi_set_bitmode(f, 0x40, ftdi.BITMODE_BITBANG)
    if res <> 0:
      raise Exception('FTDI bitmode set ERROR: %s' %ftdi_bit_err[res])
    print 'done.'
    time.sleep(0.5)
    curr_state = read_vmon_gpio('ATX_PWR_OK')
    print '    Current board state is %s.' %state[curr_state]
    print '    Simulating power button press...'
    press_pb('on')
    press_pb('off')
    press_pb('on')
  finally:
    ftdi.ftdi_usb_close(f)

def scan_jtag_chain():
  print '    Initializing and scanning JTAG chain...',
  sys.stdout.flush()
  proc = subprocess.Popen(['python', 'scan_chain.py'], stdout=subprocess.PIPE)
  out = proc.communicate()[0]
  print 'done.'
  if (out == defs.JTAG_SCAN_CYPRESS) | (out == defs.JTAG_SCAN_RENESAS):
    print '    JTAG chain succesfully scanned.\n'
  else:
    raise Exception, ('FATAL: JTAG scan chain not correct: \n\n%s' %out)

def load_ppc(mac_file):
  print '    Converting MAC file to urj...',
  sys.stdout.flush()
  proc = subprocess.Popen(['python', 'ocdc_macro_convert.py', mac_file], stdout=subprocess.PIPE)
  out = proc.communicate()[0]
  outfile = open('temp.urj', 'w')
  outfile.write(out)
  outfile.close()
  print 'done.'
  print '    Uploading the urj file via JTAG...',
  sys.stdout.flush()
  #subprocess.call(['jtag', 'temp.urj'])
  proc = subprocess.Popen(['jtag', 'temp.urj'], stdout=subprocess.PIPE)
  out = proc.communicate()[0]
  print 'done.'

def load_urj(urj_file):
  print '    Uploading the urj file via JTAG...',
  sys.stdout.flush()
  proc = subprocess.Popen(['jtag', urj_file], stdout=subprocess.PIPE)
  out = proc.communicate()[0]
  print 'done.'

def open_ftdi_uart(port, baud, timeout = 1):
  try:
    ser = serial.Serial(port, baud, timeout = timeout)
    ser.flushInput()
    ser.flushOutput()
    return ser
  except:
    print '\nERROR: Serial port could not be opened, check USB connection.'
    raise

def find_str_ser(serial_obj, string, timeout, display = True):
  serial_obj.flushOutput()
  tout = 0
  srch = -1
  buff = '' 
  while (tout < timeout) & (srch == -1):
    res = serial_obj.read(1)
    if res == '':
      tout = tout + 1
    else:
      if display:
        sys.stdout.write(res)
        sys.stdout.flush()
      buff += res
      srch = buff.find(string)
      tout = 0
  if srch == -1:
    return False, buff
  else:
    return True, buff

def print_outp_ser(serial_obj, timeout):
  #serial_obj.flushOutput()
  tout = 0
  buff = ''
  while (tout < timeout):
    res = serial_obj.read(1)
    if res == '':
      tout = tout + 1
    else:
      sys.stdout.write(res)
      sys.stdout.flush()
      buff += res
      tout = 0
  return buff    

# The I2C bus has 2 masters, FTDI and PPC. This method will check the state of the PPC and wait till it has released the I2C bus.
# This method opens a serial port object, if the serial port is already open that object wont be affected.
# Note that the delays are multiplied by 10 as the serial port timeout is set to 0.1s
def check_ppc_i2c():
  try:
    serial_obj = open_ftdi_uart(ser_port, baud, 0.1)
  except:
    raise
  try:
    serial_obj.flushInput()
    serial_obj.flushOutput()
    i2c_avbl = False
    if find_str_ser(serial_obj, 'DRAM:', 8, False)[0]:
      if find_str_ser(serial_obj, 'stop autoboot:', defs.UBOOT_DELAY*10, False)[0]:
        serial_obj.write('\n')
        i2c_avbl = True
      else:
        raise Exception('ERROR: U-Boot did not load correctly during checking that the PPC released the I2C bus.')
    # Check if there is activity on the serial port, if so Linux is booting.
    elif (len(serial_obj.read(20)) > 0):
      print '    Waiting for Linux to boot, this will take about a minute.' 
      tout = 0
      while (not i2c_avbl) and (tout < defs.BOOT_DELAY*10):
        serial_obj.write('\n')
        if find_str_ser(serial_obj, '=>', 1*10, False)[0]:
          i2c_avbl = True
        else:
          serial_obj.write('\n')
          if find_str_ser(serial_obj, 'login:', 1*10, False)[0]:
            i2c_avbl = True
        tout += 1
      if not i2c_avbl:
        raise Exception('ERROR: Timed out waiting for Linux to boot.')
    else:
      i2c_avbl = True
    return i2c_avbl
  finally:
    serial_obj.flushInput()
    serial_obj.flushOutput()
    serial_obj.close()

def press_pb(request):
  try:
    f = open_ftdi_d()
  except:
    raise
  try:
    pb_dic = {'on' : 1, 'off' : 0}
    curr_state = read_vmon_gpio('ATX_PWR_OK')
    if curr_state == pb_dic[request]:
      print '    Board is currently %s' %request
    else:
      print '    Switching board %s.' %find_key(pb_dic, not(curr_state))
      sys.stdout.flush()
      res = ftdi.ftdi_set_bitmode(f, 0x40, ftdi.BITMODE_BITBANG)
      if res <> 0:
        raise Exception('FTDI bitmode set ERROR: %s' %ftdi_bit_err[res])
      if ftdi_write(f, '\x40') <> 1:
        raise Exception('ERROR: FTDI write error, code: %d' %res)
      # poll ATX_PWR_OK until board state changes, timeout = 7 x 0.5 seconds
      new_state = curr_state
      tout = 0
      while (new_state == curr_state) and (tout < 7):
        new_state = read_vmon_gpio('ATX_PWR_OK')
        time.sleep(0.5)
        tout = tout + 1
      if ftdi_write(f, '\x00') <> 1:
        raise Exception('ERROR: FTDI write error, code: %d' %res)
      # More than 4 seconds elapsed, board should have switched state by now.
      if tout >= 7:
        raise Exception, ('Power button did not have an effect.')
  finally:
    ftdi.ftdi_usb_close(f)

def get_assigned_ip(ser_obj):
  print '    Checking PPC state...',
  ser_obj.write('\n')
  #Determine if boffile is present to see if system already netbooted.
  found, buff = find_str_ser(ser_obj, '~#', 1, False)
  linux_booted = False
  if buff.find('login:') <> -1:
    ser_obj.write('root\n')
    time.sleep(0.1)
    ser_obj.flushInput()
    ser_obj.flushOutput()
    linux_booted = True
  elif found:
    linux_booted = True
  if linux_booted:
    ser_obj.write('\n\n')
    ser_obj.write('ls /boffiles/%s\n'%defs.QDR_TST_BOF)
    if find_str_ser(ser_obj, 'cannot access', 1, False)[0]:
      linux_booted = False
  if not linux_booted:
    print 'Linux not booted, cycling power and net booting.'
    press_pb('off')
    press_pb('on')
    ser_obj.write('run netboot\n')
    print '    Waiting for Linux to net boot, this may take a minute...',
    sys.stdout.flush()
    if not find_str_ser(ser_obj, 'Linux/PowerPC load:', 10, False)[0]:
      raise Exception('ERROR: Linux did not boot correctly.')
    print 'loading romfs...',
    sys.stdout.flush()
    if not find_str_ser(ser_obj, 'Sending DHCP requests', 10, False)[0]:
      raise Exception('ERROR: Linux did not boot correctly.')
    print 'getting IP...',
    sys.stdout.flush()
    if not find_str_ser(ser_obj, 'login:', 10, False)[0]:
      raise Exception('ERROR: Linux did not boot correctly.')
    print 'done.'
    sys.stdout.flush()
    print '    Checking that the boffile is present...',
    ser_obj.write('root\n')
    time.sleep(0.1)
    ser_obj.flushInput()
    ser_obj.flushOutput()
    ser_obj.write('\n\n')
    ser_obj.write('ls /boffiles/%s\n'%defs.QDR_TST_BOF)
    if find_str_ser(ser_obj, 'cannot access', 1, True)[0]:
      raise Exception('ERROR: Boffile not found, check NFS directory, or Linux not booted correctly.')
    print 'done.'
  else:
    print 'Linux booted.'
  # Get assigned IP address
  ser_obj.flushInput()
  ser_obj.flushOutput()
  ser_obj.write('\n\n')
  ser_obj.write('/sbin/ifconfig\n')
  found, buff = find_str_ser(ser_obj, 'UP BROADCAST RUNNING', 1, False)
  if not found:
      raise Exception('ERROR: Linux not loaded or did not boot correctly or IP address not leased.')
  start_idx = buff.find('inet addr:') + 10
  stop_idx = buff.find('Bcast:') - 1 
  return buff[start_idx:stop_idx]

if __name__ == "__main__":
  
  p = OptionParser()
  p.set_usage('roach2_ats.py revision')
  opts, args = p.parse_args(sys.argv[1:])
  # If this option is false Digicom will be automatically selected as the manufacturer.
  SEL_MANUF = False
  # There are many differences between ROACH2 Rev 1 and 2. Select which version to test here.
  if args == []:
    REV = 2
  elif args[0] == '1':
    REV = 1
  else:
    REV = 2
  os.system('clear')

  if REV == 1:
    sn = defs.SN_REV1
  else:
    sn = defs.SN_REV2
  ser_port = '/dev/ttyUSB2'
  baud = 115200
  state = ['off', 'on']
  # Error states for setting ftdi interface
  ftdi_int_err = {-1:'unknown interface', -2:'usb device unavailable'}
  # Error states for setting bitmode
  ftdi_bit_err = {-1:'can\'t enable bitbang mode', -2:'USB device unavailable'}
  # Error states for opening ftdi interface in bitbang mode
  ftdi_open_err = {
     0 : 'all fine',
    -1 : 'usb_find_busses() failed',
    -2 : 'usb_find_devices() failed',
    -3 : 'usb device not found',
    -4 : 'unable to open device',
    -5 : 'unable to claim device',
    -6 : 'reset failed',
    -7 : 'set baudrate failed',
    -8 : 'get product description failed',
    -9 : 'get serial number failed',
    -10: 'unable to close device'
  }

  # create logger
  logger = logging.getLogger('r2_ats')
  logger.setLevel(logging.DEBUG)
  # create file handler which logs event debug messages
  fh = logging.FileHandler('test.log')
  fh.setLevel(logging.DEBUG)
  # create console handler with a higher log level
  ch = logging.StreamHandler()
  ch.setLevel(logging.ERROR)
  # create formatter and add it to the handlers
  formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
  fh.setFormatter(formatter)
  ch.setFormatter(formatter)
  # add the handlers to the logger
  logger.addHandler(fh)
  logger.addHandler(ch)

  # Set default colours
  DEF_C = {
    '1':c.ENDC,
    '2':c.ENDC,
    '3':c.ENDC,
    '4':c.ENDC,
    '5':c.ENDC,
    '6':c.ENDC,
    '7':c.ENDC,
    '8':c.ENDC,
    '9':c.ENDC,
    '0':c.ENDC,
    'w':c.ENDC,
    'e':c.ENDC,
    'r':c.ENDC,
    't':c.ENDC,
    'q':c.ENDC,
    'm':c.ENDC
  }

  # Set menu flags
  usb_conn = False
  sn_set = False
  power_test = False
  pb_on = False
  pb_off = False
  print_vc = False
  scan_jtag = False
  prog_eeprom = False
  read_eeprom = False
  flash_chk = False
  uboot_load = False
  kernel_load = False
  root_load = False
  cpld_done = False
  qdr_ok = False
  test_usb = False
  uboot_rem = False
  quit = False

  # print_menu and menu_refresh is used to only reprint the menu when something changes.
  print_menu = True
  menu_refresh = False
  menu_text = 'ROACH2 Rev %d Testing - ' %REV + c.WARNING + 'USB not connected' + c.ENDC
  # Set default print colours
  col = DEF_C.copy()

  # Check if serial port present
  while not quit:
    #The while loop polls keypresses and USB connection status. 
    #A timeout is added to the keypress function so the loop does not hog the machine, 
    #this is not elegant but it keeps things simple!
    #NOTE: the current method may lose keypresses, but it is very unlikely and does not really matter
    #the user can just press the menu key again, but should still be fixed.

    # Detect USB disconnect, use this signal to reset all flags
    usb_disc = False
    try:
      with open(ser_port) as f: 
        usb_conn = True
    except IOError:
      # Detect disconnect
      if usb_conn:
        usb_conn = False
        usb_disc = True
    finally:
      if usb_conn:
        f.close()

    try:
      # Reset colours and flags on USB disconnect
      if usb_disc:
        menu_text = 'ROACH2 Rev %d Testing - ' %REV + c.WARNING + 'USB not connected' + c.ENDC
        col = DEF_C.copy()
        sn_set = False
        power_teset = False
        pb_on = False
        pb_off = False
        print_vc = False
        scan_jtag = False
        prog_eeprom = False
        read_eeprom = False
        flash_shk = False
        uboot_load = False
        kernel_load = False
        root_load = False
        cpld_done = False
        qdr_ok = False
        test_usb = False
        uboot_rem = False
        print_menu = True
        menu_refresh = False
        os.system('clear')

      # Set menu colours
      if usb_conn and (not menu_refresh):
        print_menu = True
        menu_refresh = True
        menu_text = 'ROACH2 Rev %d Testing - ' %REV + c.OKGREEN + 'USB connected' + c.ENDC
        os.system('clear')
      if sn_set:
        col['1'] = c.OKGREEN
      if power_test:
        col['2'] = c.OKGREEN
      if pb_on:
        col['3'] = c.OKGREEN
      if pb_off:
        col['4'] = c.OKGREEN
      if print_vc:
        col['5'] = c.OKGREEN
      if scan_jtag:
        col['6'] = c.OKGREEN
      if prog_eeprom:
        col['7'] = c.OKGREEN
      if read_eeprom:
        col['8'] = c.OKGREEN
      if flash_chk:
        col['9'] = c.OKGREEN
      if uboot_load:
        col['0'] = c.OKGREEN
      elif uboot_rem:
        col['0'] = c.ENDC
        col['m'] = c.ENDC
      if cpld_done:
        col['w'] = c.OKGREEN
      if qdr_ok:
        col['e'] = c.OKGREEN
      if test_usb:
        col['t'] = c.OKGREEN
      
      if print_menu:
        manuf_id = sn['manufacturer']
        manuf_name = find_key(defs.MANUF, manuf_id)
        print ''
        print col['1'] + 'Selected serial number:' + c.ENDC
        print col['1'] + '  Manufacturer ID: %s (%s)' %(chr(manuf_id), manuf_name) + c.ENDC
        print col['1'] + '  Type: %s (ROACH2)' %sn['type'] + c.ENDC
        print col['1'] + '  Revision: %s ' %sn['revision'] + c.ENDC
        print col['1'] + '  Batch: %s ' %sn['batch'] + c.ENDC
        print col['1'] + '  Board: %s ' %sn['board'] + c.ENDC
        print ''
        print menu_text
        print col['1'] + '    1) Set serial number for ROACH2' + c.ENDC
        print col['2'] + '    2) Power-up tests, JTAG scan and program EEPROM' + c.ENDC
        print col['3'] + '    3) Switch board on' + c.ENDC
        print col['4'] + '    4) Switch board off' + c.ENDC
        print col['5'] + '    5) Print voltages, currents and temperatures' + c.ENDC
        print col['6'] + '    6) Scan JTAG chain' + c.ENDC
        print col['7'] + '    7) Program EEPROM' + c.ENDC
        print col['8'] + '    8) Display EEPROM contents' + c.ENDC
        print col['9'] + '    9) Test FLASH and PPC DDR2 memory' + c.ENDC
        print col['0'] + '    0) Load U-boot, kernel and filesystem' + c.ENDC
        print col['w'] + '    w) Program CPLD' + c.ENDC
        print col['e'] + '    e) Test QDR memory' + c.ENDC
        print col['r'] + '    r) Run preliminary DDR3, ZDOK, TGE, 1GE tests' + c.ENDC
        print col['t'] + '    t) Test PPC USB host' + c.ENDC
        print col['m'] + '    m) Unload U-Boot (if U-Boot does not start correctly and holds the I2C bus).' + c.ENDC
        print col['q'] + '    q) Quit' + c.ENDC
      answer = getkey()
      if answer == None:
        answer = ''
        print_menu = False
      else:
        # Set menu colour to fail, will be set to pass if tests pass.
        try:
          col[answer] = c.FAIL
        except KeyError:
          print_menu = True

      if '1' in answer:
        sn_set = False
        sn = set_serial_number()
        sn_set = True
        print_menu = True
      elif '2' in answer:
        print c.OKBLUE + '\n    Testing power, configuring EEPROM and scanning JTAG chain' + c.ENDC
        power_test = False
        if not sn_set:
          print '    WARNING: Serial number not set, press \'1\' to set or any key to use default...'
          answer = getkey_block()
          if '1' in answer:
            done = False
            while not done:
              sn = set_serial_number()
              print '    Is this correct? Press any key or \'n\' to re-enter serial number.'
              answer = getkey_block()
              if 'n' not in answer:
                sn_set = True
                done = True
        config_mon();
        test_power()
        program_eeprom(sn)
        scan_jtag_chain()
        power_test = True
        print_menu = True
      elif '3' in answer:
        print ''
        pb_on = False
        press_pb('on')
        pb_on = True
        print_menu = True
      elif '4' in answer:
        print ''
        pb_off = False
        press_pb('off')
        pb_off = True
        print_menu = True
      elif '5' in answer:
        print_vc = False
        curr_state = read_vmon_gpio('ATX_PWR_OK')
        print c.OKBLUE + ('\n    Current board state is %s.' %state[curr_state]) + c.ENDC
        config_mon()
        print_v_c()
        print_temps()
        print_vc = True
        print_menu = True
      elif '6' in answer:
        press_pb('on')
        print c.OKBLUE + ('\n    Scanning JTAG chain.') + c.ENDC
        scan_jtag = False
        scan_jtag_chain()
        print_menu = True
        scan_jtag = True
      elif '7' in answer:
        print c.OKBLUE + ('\n    Programming EEPROM.') + c.ENDC
        prog_eeprom = False
        if not sn_set:
          print '    WARNING: Serial number not set, press \'1\' to set or any key to use default...'
          answer = getkey_block()
          if '1' in answer:
            done = False
            while not done:
              sn = set_serial_number()
              print '    Is this correct? Press any key or \'n\' to re-enter serial number.'
              answer = getkey_block()
              if 'n' not in answer:
                sn_set = True
                done = True
        program_eeprom(sn)
        prog_eeprom = True
        print_menu = True
      elif '8' in answer:
        read_eeprom = False
        if check_ppc_i2c():
          try:
            i2c_bus = open_ftdi_b()
          except:
            raise
          try:
            print ('\n\nEEPROM at 0x%02x' %iicf.ADDR_BOOT_EEPROM_0)
            for i in range (256):
              if (i % 0x10 == 0):
                print('\n0x%02x' %i),
              print ('%02x' %(iicf.i2c_regread(i2c_bus, iicf.ADDR_BOOT_EEPROM_0, i))),

            print ('\n\nEEPROM at 0x%02x' %iicf.ADDR_BOOT_EEPROM_1)
            for i in range (256):
              if (i % 0x10 == 0):
                print('\n0x%02x' %i),
              print ('%02x' %(iicf.i2c_regread(i2c_bus, iicf.ADDR_BOOT_EEPROM_1, i))),
            print''
            print''
            read_eeprom = True
          finally:
            i2c_bus.Close()
            print_menu = True
        else:
          raise Exception('ERROR: I2c bus could not be secured from the PPC (check PPC state), EEPROM contents not read.')
      elif '9' in answer:
        print c.OKBLUE + ('\n    Checking FLASH and PPC DDR2 memory.') + c.ENDC
        try:
          ser = open_ftdi_uart(ser_port, baud)
        except:
          raise
        try:
          flash_chk = False
          press_pb('off')
          press_pb('on')
          print '    Loading flash checking program via JTAG, this will take while...'
          load_ppc('support_files/flashck.mac')
          if not find_str_ser(ser, 'pass', 3)[0]:
            raise Exception, ('FATAL: FLASH memory test failed.')
          print c.OKBLUE + ('\n\n    FLASH memory test passed.\n') + c.ENDC
          if REV == 1:
            xio = xtx.Xmodem_tx(ser, defs.UBOOT_REV1_MEMTEST, fh)
          else:
            xio = xtx.Xmodem_tx(ser, defs.UBOOT_REV2_MEMTEST, fh)
          print '    Loading DDR2 checking program via JTAG, this will take while...'
          load_ppc('support_files/program.mac')
          print '    Sending U-Boot via Xmodem.'
          if not xio.xmdm_send():
            raise Exception('FATAL: U-Boot Xmodem transfer not successfull.')
          if not find_str_ser(ser, 'SDRAM test passes', 1)[0]:
            raise  Exception('FATAL: PPC DDR2 memory test failed.')
          print c.OKBLUE + ('\n\n    PPC DDR2 memory test passed.') + c.ENDC
          print '\n    Clearing PPC.'
          load_ppc('support_files/program.mac')
          flash_chk = True
        finally:
          ser.close()
          print_menu = True
      elif '0' in answer:
        print c.OKBLUE + ('\n    Loading U-Boot, Linux kernel and on board root file system.')
        print '        Press any key to load all three.'
        print '        Press 1 to load U-Boot.'
        print '        Press 2 to load the kernel.'
        print '        Press 3 to load the root file system.' + c.ENDC
        answer = getkey_block()
        load_all = False
        load_uboot = False
        load_kernel = False
        load_root = False
        if '1' in answer:
          load_uboot = True
        elif '2' in answer:
          load_kernel = True
        elif '3' in answer:
          load_root = True
        else:
          load_all = True

        try:
          ser = open_ftdi_uart(ser_port, baud)
        except:
          raise
        try:
          if load_all or load_uboot:
            uoot_load = False
            press_pb('off')
            press_pb('on')
            if REV == 1:
              xio = xtx.Xmodem_tx(ser, defs.UBOOT_REV1, fh)
            else:
              xio = xtx.Xmodem_tx(ser, defs.UBOOT_REV2, fh)
            print '    Loading rinit (PPC Xmodem receiver program) via JTAG, this will take while...'
            load_ppc('support_files/program.mac')
            start_time = time.time()
            print '    Sending U-Boot via Xmodem.'
            if not xio.xmdm_send():
              print 'Elapsed: %f' %(time.time() - start_time)
              raise Exception('FATAL: U-Boot Xmodem transfer not successfull.')
            if not find_str_ser(ser, 'stop autoboot:', defs.UBOOT_DELAY)[0]:
              raise  Exception('FATAL: U-Boot did not boot after x-modem transfer.')
            ser.write('\n')
            ser.write('run clearenv\n')
            print_outp_ser(ser, 1)
            ser.write('reset\n')
            if not find_str_ser(ser, 'stop autoboot:', defs.UBOOT_DELAY)[0]:
              raise Exception('FATAL: U-Boot did not boot after reset.')
            ser.write('\n')
            ser.write('saveenv\n')
            print_outp_ser(ser, 1)
            uboot_load = True
          if load_all or load_kernel:
            kernel_load = False
            if load_kernel:
              press_pb('off')
              press_pb('on')
            ser.write('run tftpkernel\n')
            if not find_str_ser(ser, 'Waiting for PHY', 1)[0]:
              raise Exception('ERROR: U-Boot not loaded, load U-Boot before loading kernel.')
            if not find_str_ser(ser, 'DHCP client bound to address', 60)[0]:
              raise Exception('ERROR: IP address not assigned, check connections and DHCP server.')
            if not find_str_ser(ser, 'done\r\n=>', 40)[0]:
              raise Exception('ERROR: Kernel did not load.')
            kernel_load = True
          if load_all or load_root:
            if load_root:
              press_pb('off')
              press_pb('on')
            root_load = False
            ser.write('run tftproot\n')
            if not find_str_ser(ser, 'ENET Speed is', 1)[0]:
              raise Exception('ERROR: U-Boot not loaded, load U-Boot before loading root file system.')
            if not find_str_ser(ser, 'DHCP client bound to address', 60)[0]:
              raise Exception('ERROR: IP address not assigned, check connections and DHCP server.')
            if not find_str_ser(ser, 'done\r\n=>', 5.5*60)[0]:
              raise Exception('ERROR: Root file system did not load.')
            root_load = True
        finally:
          ser.close()
          print_menu = True
      elif 'w' in answer:
        try:
          ser = open_ftdi_uart(ser_port, baud)
        except:
          raise
        try:
          print c.OKBLUE + '\n    Programming CPLD.' + c.ENDC
          cpld_done = False
          ser.flushInput()
          ser.flushOutput()
          press_pb('on')
          print '    Erasing CPLD...',
          load_urj('support_files/erase_cpld.urj')
          print '    Programming CPLD...',
          load_urj('support_files/program_cpld.urj')
          press_pb('off')
          press_pb('on')
          print '    Dumping CPLD mapped memory to confirm CPLD configuration.'  
          ser.write('md 0xc0000000 8\n')
          out = print_outp_ser(ser, 1)
          print ''
          if out.find(defs.CPLD_MD) == -1:
            raise Exception, ('FATAL: CPLD did not program correctly.')
          cpld_done = True
        finally:
          ser.close()
          print_menu = True
      elif 'e' in answer:
        try:
          ser = open_ftdi_uart(ser_port, baud)
        except:
          raise
        try:
          print c.OKBLUE + '\n    Testing QDR memory.' + c.ENDC
          qdr_ok = False
          ser.flushInput()
          ser.flushOutput()
          try:
            with open('/home/nfs/roach2/current/boffiles/%s'%defs.QDR_TST_BOF) as f: pass
          except:
            inpath = 'support_files/%s'%defs.QDR_TST_BOF
            outpath = '/home/nfs/roach2/current/boffiles/%s'%defs.QDR_TST_BOF
            shutil.copyfile(inpath, outpath)
            os.chmod(outpath, 0777)
          ip_addr = get_assigned_ip(ser)
          qdr_ok = qdr_tst.test_qdr(ip_addr, defs.QDR_TST_BOF)
        finally:
          try: 
            for f in fpga: f.close()
          except: pass
          ser.close()
          print_menu = True
      elif 'r' in answer:
        try:
          ser = open_ftdi_uart(ser_port, baud)
        except:
          raise
        try:
          print c.OKBLUE + '\n    Running preliminary DDR3, ZDOK, TGE and 1GE tests.' + c.ENDC
          press_pb('off')
          press_pb('on')
          ser.write('\n')
          if not find_str_ser(ser, '=>', 1, False)[0]:
            raise Exception('ERROR: U-Boot did not load correctly after powerup.')
          ser.write('dhcp\n')
          if not find_str_ser(ser, 'Bytes transferred', 10)[0]:
            raise Exception('DHCP request not successful.')
          time.sleep(0.5)
          if REV == 1:
            ser.write('tftp 100000 roach2_bsp_rev2.bin\n')
          else:
            ser.write('tftp 100000 roach2_bsp_rev2.bin\n')
          print_outp_ser(ser, 2)
          ser.write('r2smap 100000\n')
          print_outp_ser(ser, 2)
          ser.write('r2bit v6comm\n')
          print_outp_ser(ser, 2)
          ser.write('r2bit v6gbe\n')
          print_outp_ser(ser, 2)
          ser.write('r2bit ddr3\n')
          print_outp_ser(ser, 2)
          for i in range(8):
            ser.write('r2bit tge %d\n' %i)
            print_outp_ser(ser, 2)
          for i in range(2):
            ser.write('r2bit zdok %d\n' %i)
            print_outp_ser(ser, 2)
        finally:
          ser.close()
          print_menu = True
      elif 't' in answer:
        try:
          ser = open_ftdi_uart(ser_port, baud)
        except:
          raise
        try:
          print c.OKBLUE + '\n    Loading roach2_bsp.bin from USB flash drive and programming FPGA.' + c.ENDC
          test_usb = False
          press_pb('off')
          press_pb('on')
          # Scan the USB bus 5 times, u-boot sometimes takes a while to detect the USB drive
          retry = 0
          while not find_str_ser(ser, '1 Storage Device(s) found', 3)[0]:
            ser.write('usb start\n')
            retry += 1
            if retry == 5:
              raise Exception('ERROR: USB flash drive not detected.')
          ser.write('fatload usb 0 100000 roach2_bsp.bin\n')
          print ''
          print ''
          print 'Reading from USB flash drive, this will take about a minute.'
          print ''
          if not find_str_ser(ser, '19586188 bytes read', 60, True)[0]:
            raise Exception('ERROR: File not loaded successfully from USB drive')
          time.sleep(0.1)
          ser.write('r2smap 100000\n')
          if not find_str_ser(ser, 'info: SelectMAP configuration succeeded', 5, True)[0]:
            raise Exception('ERROR: SelectMAP configuration failed.')
          print ''
          print 'USB host working correctly.'
          test_usb = True
        finally:
          ser.close()
          print_menu = True
      elif 'm' in answer:
        print c.OKBLUE + ('\n    Removing U-Boot.') + c.ENDC
        uboot_rem = False
        press_pb('off')
        press_pb('on')
        print '\n    Unloading U-Boot via JTAG, this will take while...'
        load_ppc('support_files/program.mac')
        print '    Done.'
        uboot_rem = True
        uboot_load = False
        print_menu = True
      elif 'q' in answer:
        quit =  True
    except RuntimeError as e: 
      print c.FAIL + '%s'%e + c.ENDC
      print_menu = True
    except:
      exc_type = sys.exc_info()[0]
      exc_mess = sys.exc_info()[1]
      print ''
      print c.FAIL+ ("%s %s" %(exc_mess, exc_type)) + c.ENDC
      print_menu = True






# Code to shorten the JTAG chain and disable FTDI JTAG to use a JTAG debugger.
        #try: 
        #  f = open_ftdi_d()
        #  res = ftdi.ftdi_set_bitmode(f, 0x08, ftdi.BITMODE_BITBANG)
        #  if res <> 0:
        #   raise Exception('FTDI bitmode set ERROR: %s' %ftdi_bit_err[res])
        #  res = ftdi_write(f, '\x08')   
        #  if res <> 1:
        #    raise Exception('ERROR: FTDI write error, code: %d' %res)
        #finally:
        #  ftdi.ftdi_usb_close(f)
        #try:  
        # disable_ftdi_a()
        #xcept:
        # raise
