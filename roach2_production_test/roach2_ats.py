#!/usr/bin/python

import sys, time, select, termios, tty, ftdi, subprocess, serial, logging
from mpsse import *
import i2c_functions as iicf
import xmodem_tx as xtx
import defs_max16071, defs_max1805, defs_ad7414
import defs_r2_ats as defs


def config_cmon():
  print "        Setting software enable configuration...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.SW_EN_CONF, defs_max16071.SW_EN_CONF_VAL))
  print "        Setting overcurrent primary threshold and current-sense control...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.OCPT_CSC1, defs_max16071.OCPT_CSC1_VAL))
  print "        Setting full-scale range for ADCs 1-4...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.ADC_CONF_4321, defs_max16071.ADC_CONF_4321_VAL))
  print "        Setting full-scale range for ADCs 5-8...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_C_MON, defs_max16071.ADC_CONF_8765, defs_max16071.ADC_CONF_8765_VAL))

def config_vmon():
  print "        Setting software enable configuration...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.SW_EN_CONF, defs_max16071.SW_EN_CONF_VAL))
  print "        Setting overcurrent primary threshold and current-sense control...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.OCPT_CSC1, defs_max16071.OCPT_CSC1_VAL))
  print "        Setting full-scale range for ADCs 1-4...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.ADC_CONF_4321, defs_max16071.ADC_CONF_4321_VAL))
  print "        Setting full-scale range for ADCs 5-8...",(iicf.i2c_regwrite(i2c_bus, iicf.ADDR_V_MON, defs_max16071.ADC_CONF_8765, defs_max16071.ADC_CONF_8765_VAL))

def read_voltage(vbus):
  ch = defs.V_MON_MAP[vbus]
  ch_idx = (ch - 1)*2
  msb = defs_max16071.CH_ARRAY[ch_idx]
  lsb = defs_max16071.CH_ARRAY[ch_idx+1]
  addr = iicf.ADDR_V_MON
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

def read_current(vbus):
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
  gain = 1 + (1e5/defs_max16071.GAIN_RES[ch - 1])
  amps = ((voltage/1024.0)*fs_v)/(res*gain)
  return amps

# read max16071 onboard current
def read_ob_current(vbus):
  if vbus == '5V':
    addr = iicf.ADDR_C_MON
    res = defs_max16071.SNS_RES_5V0
  else:
    addr = iicf.ADDR_V_MON
    res = defs_max16071.SNS_RES_12V
  voltage = iicf.i2c_regread(i2c_bus, addr, defs_max16071.MON_C)
  #Get current-sense gain setting
  curr_sns_gn = defs_max16071.CURR_SNS_GAIN[((iicf.i2c_regread(i2c_bus, addr, defs_max16071.OCPT_CSC1)) >> 2) & 0x03]
  amps = (((voltage/255.0)*1.4)/curr_sns_gn)/res
  return amps

def read_vmon_gpio(gpio):
  ch = defs.V_MON_GPIO[gpio] - 1
  addr = iicf.ADDR_V_MON
  gpio_rd = iicf.i2c_regread(i2c_bus, addr, defs_max16071.GPIO_INPUT_STATE)
  val = (gpio_rd >> ch) & 0x01
  return val

def check_currents(dic):
  c_err = []
  for i, v in dic.iteritems():
    vbus = i[:-2]
    if (vbus == '12V') | (vbus == '5V0'):
      amps = read_ob_current(vbus)
    else:
      amps = read_current(vbus)
    if i[-1] == 'H':
      if amps > v:
        c_err.append(i)
        c_err.append(amps)
    else:
      if amps < v:
        c_err.append(i)
        c_err.append(amps)
  return c_err

def check_voltages():
  v_err = []
  for i, v in defs.V_THRESHOLD.iteritems():
    volts = read_voltage(i[:-2])
    if i[-1] == 'H':
      if volts > v:
        v_err.append(i)
        v_err.append(volts)
    else:
      if volts < v:
        v_err.append(i)
        v_err.append(volts)
  return v_err

def print_v_c():
  print "    1V0 Monitor: %.3fv" %read_voltage('1V0')
  print "    1V5 Monitor: %.3fv" %read_voltage('1V5')
  print "    1V8 Monitor: %.3fv" %read_voltage('1V8')
  print "    2V5 Monitor: %.3fv" %read_voltage('2V5')
  print "    3V3 Monitor: %.3fv" %read_voltage('3V3')
  print "    5V0 Monitor: %.3fv" %read_voltage('5V0')
  print "    12V Monitor: %.3fv" %read_voltage('12V')
  print "    3V3 Aux Monitor: %.3fv" %read_voltage('3V3_AUX')
  print "    12V Monitor (rev1 mod): %.3fv" %(read_voltage('3V3_AUX')*defs_max16071.V_DIV_12V)
  print "    12V current: %.3fA" %read_ob_current('12V')
  print "    5V0 current: %.3fA" %read_ob_current('5V')
  print "    3V3 current: %.3fA" %read_current('3V3')
  print "    2V5 current: %.3fA" %read_current('2V5')
  print "    1V8 current: %.3fA" %read_current('1V8')
  print "    1V5 current: %.3fA" %read_current('1V5')
  print "    1V0 current: %.3fA" %read_current('1V0')
  print "    MGT 1.2V Power Good = %d" %read_vmon_gpio('MGT_1V2_PG')
  print "    MGT 1.0V Power Good = %d" %read_vmon_gpio('MGT_1V0_PG')
  print ""


def read_temp(sensor):
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
  t_err = []
  for i, v in defs.T_THRESHOLD.iteritems():
    temp = read_temp(i[:-4])
    if i[-1] == 'H':
      if temp > v:
        t_err.append(i)
        t_err.append(temp)
    else:
      if temp < v:
        t_err.append(i)
        t_err.append(temp)
  return t_err
  
def print_temps():
  print '    PPC Temp: %d degreesC' %read_temp('PPC')
  print '    FPGA Temp: %d degreesC' %read_temp('FPGA')
  print '    Inlet Temp: %0.2f degreesC' %read_temp('INLET')
  print '    Inlet Temp: %0.2f degreesC' %read_temp('OUTLET')
  print ''

#To select menu item without pressing enter
def getkey():
  old_settings = termios.tcgetattr(sys.stdin)
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  answer = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
  return answer

def set_serial_number():
  print "Select manufacturer (default = Digicom):"
  for i, v in enumerate(defs.MANUF):
    print ("    %d) %s" %(i+1, v))
  answer = getkey()
  try:
    ans = int(answer)
    if ans in range(1, i+2):
      manuf = defs.MANUF.values()[ans-1]
    else:
      manuf = defs.MANUF['Digicom']
  except ValueError:
    manuf = defs.MANUF['Digicom']
  done = False
  while not done:
    rev = raw_input("Select ROACH2 revision (default = 2): ")
    try:
      rev = int(rev)
      if rev not in range(1,3):
        print ('Revision %d not supported' %rev)
      else:
        done = True
    except ValueError:
      rev = 2
      done = True
  done = False
  while not done:
    batch = raw_input("Batch Number (0-255): ")
    try:
      batch = int(batch)
      if batch not in range(256):
        print ('Input valid batch number') 
      else:
        done = True
    except ValueError:
      print ('Input valid batch number')
  done = False
  while not done:
    board = raw_input("Board Number (0-255): ")
    try:
      board = int(board)
      if board not in range(256):
        print ('Input valid board number') 
      else:
        done = True
    except ValueError:
      print ('Input valid board number')
  sn = {     
        'manufacturer' : manuf, 
        'type'         : 0x01, 
        'revision'     : rev, 
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
  print '    Writing boot parameters for configuration G boot to EEPROM locations 0x00 to 0x0f...',
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


def find_key(dic, val):
    """return the key of dictionary dic given the value"""
    return [k for k, v in dic.iteritems() if v == val][0]

def open_ftdi_d():
  f = ftdi.ftdi_context()
  ftdi.ftdi_init(f)
  ftdi.ftdi_set_interface(f, ftdi.INTERFACE_D)
  ftdi.ftdi_usb_open(f, defs.R2_VID, defs.R2_PID)
  return f

def open_ftdi_b():
  i2c_bus = MPSSE()
  i2c_bus.Open(defs.R2_VID, defs.R2_PID, I2C, ONE_HUNDRED_KHZ, MSB, IFACE_B, None)
  return i2c_bus

def test_power():
  print '    Testing power force on.'
  # enable PCTRL_EN and PCTRL_ONn lines on ftdi interface D
  ftdi.ftdi_set_bitmode(f, 0x30, ftdi.BITMODE_BITBANG)
  # force power on: PCTRL_EN = 1, PCTRL_ONn = 0, 
  ftdi.ftdi_write_data(f, '\x20', 1)
  print '        Board forced on...', 
  time.sleep(defs.PG_DELAY)
  # check ATX_PG
  if not read_vmon_gpio('ATX_PG'):
    raise Exception, 'FATAL: Board did not power up on power force on.'
  print 'done.'
  print '    Checking voltage tolerances...',
  v_err = check_voltages()
  if v_err:
    print 'FATAL: Voltage error detected, forcing board off'
    ftdi.ftdi_write_data(f, '\x30', 1)
    raise Exception, ('FATAL: Voltages out of range: %s' %v_err)
  print 'done.'
  print '    Waiting 1 seconds for currents to settle...',
  sys.stdout.flush()
  time.sleep(1)
  print 'done.'
  print '    Checking unconfigured current tolerances...',
  c_err = check_currents(defs.UC_C_THRESHOLD)
  if c_err:
    print 'FATAL: Unconfigured current tolerance error detected, forcing board off'
    ftdi.ftdi_write_data(f, '\x30', 1)
    raise Exception, ('FATAL: Unconfigured currents out of range: %s' %c_err)
  print 'done.'
  print '    Checking temperatures...',
  t_err = check_temps()
  if t_err:
    print 'FATAL: Temperature sensor\s out of range, forcing board off'
    ftdi.ftdi_write_data(f, '\x30', 1)
    raise Exception, ('FATAL: Temperature sensor\s out of range: %s' %t_err)
  print 'done.'
  print_v_c()
  print_temps()
  print '    Forcing board off...',
  ftdi.ftdi_write_data(f, '\x30', 1)
  on = True
  while on:
    on = read_vmon_gpio('ATX_PG')
  print 'done.'
  print '    Disabling power force...',
  ftdi.ftdi_write_data(f, '\x00', 1)
  ftdi.ftdi_set_bitmode(f, 0x40, ftdi.BITMODE_BITBANG)
  print 'done.'
  time.sleep(0.5)
  curr_state = read_vmon_gpio('ATX_PG')
  print '    Current board state is %s.' %state[curr_state]
  print '    Simulating power button press...',
  ftdi.ftdi_write_data(f, '\x40', 1)
  # poll ATX_PG until board state changes
  new_state = curr_state
  tout = 0
  while (new_state == curr_state) and (tout < 10):
    new_state = read_vmon_gpio('ATX_PG')
    time.sleep(0.5)
    tout = tout + 1
  ftdi.ftdi_write_data(f, '\x00', 1)
  if new_state == curr_state:
    raise Exception, ('FATAL: Power button did not cycle board.')
  print 'done.'
  print '    Current board state is %s.' %state[new_state]
  if not new_state:
    print '    Switching board on...',
    time.sleep(defs.PB_DELAY)
    ftdi.ftdi_write_data(f, '\x40', 1)
    time.sleep(defs.PB_DELAY)
    ftdi.ftdi_write_data(f, '\x00', 1)
    time.sleep(defs.PG_DELAY)
    new_state = read_vmon_gpio('ATX_PG')
    if not new_state:
      raise Exception, ('FATAL: Power button did not cycle board.')
    print 'done.'

def scan_jtag_chain():
  print '    Initializing and scanning JTAG chain...',
  sys.stdout.flush()
  proc = subprocess.Popen(['python', 'scan_chain.py'], stdout=subprocess.PIPE)
  out = proc.communicate()[0]
  print 'done.'
  if out <> defs.JTAG_SCAN:
    raise Exception, ('FATAL: JTAG scan chain not correct: \n\n%s' %out)
  print '    JTAG chain succesfully scanned.\n'

def load_ppc(mac_file):
  print '    Converting MAC file to urj...',
  sys.stdout.flush()
  proc = subprocess.Popen(['python', 'ocdc_macro_convert.py', mac_file], stdout=subprocess.PIPE)
  out = proc.communicate()[0]
  outfile = open('temp.urj', 'w')
  outfile.write(out)
  outfile.close()
  print 'done.'
  print'    Uploading the urj file via JTAG.'
  sys.stdout.flush()
  #subprocess.call(['jtag', 'temp.urj'])
  proc = subprocess.Popen(['jtag', 'temp.urj'])
  out = proc.communicate()[0]

def load_urj(urj_file):
  print'    Uploading the urj file via JTAG.'
  sys.stdout.flush()
  proc = subprocess.Popen(['jtag', urj_file])
  out = proc.communicate()[0]

def open_ftdi_uart(port, baud):
  ser = serial.Serial(port, baud, timeout=1)
  ser.flushInput()
  ser.flushOutput()
  return ser

def find_str_ser(serial_obj, string, timeout):

  serial_obj.flushInput()
  serial_obj.flushOutput()
  tout = 0
  srch = -1
  buff = '' 
  while (tout < timeout) & (srch == -1):
    res = serial_obj.read(1)
    if res == '':
      tout = tout + 1
    else:
      sys.stdout.write(res)
      sys.stdout.flush()
      buff += res
      srch = buff.find(string)
      tout = 0
  return buff 

def print_outp_ser(serial_obj, timeout):
  serial_obj.flushInput()
  serial_obj.flushOutput()
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

def press_pb(request):
  pb_dic = {'on' : 1, 'off' : 0}
  curr_state = read_vmon_gpio('ATX_PG')
  if curr_state == pb_dic[request]:
    print '    Board is currently %s' %request
    print ''
  else:
    print '    Switching board %s.' %find_key(pb_dic, not(curr_state))
    ftdi.ftdi_set_bitmode(f, 0x40, ftdi.BITMODE_BITBANG)
    ftdi.ftdi_write_data(f, '\x40', 1)
    # poll ATX_PG until board state changes
    new_state = curr_state
    tout = 0
    while (new_state == curr_state) and (tout < 10):
      new_state = read_vmon_gpio('ATX_PG')
      time.sleep(0.5)
      tout = tout + 1
    ftdi.ftdi_write_data(f, '\x00', 1)
    print ''
    time.sleep(0.5)
    if tout == 10:
      raise Exception, ('Power button did not have an effect.')

if __name__ == "__main__":

  sn = defs.SN
  sn_set = False
  ser_port = '/dev/ttyUSB2'
  baud = 115200
  state = ['off', 'on']

  quit = False

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


  while not quit:
    print """Main Menu
    1) Set serial number for ROACH2
    2) Power-up tests, JTAG scan and program EEPROM
    3) Switch board on
    4) Switch board off
    5) Print voltages, currents and temperatures
    6) Scan JTAG chain
    7) Program EEPROM
    8) Display EEPROM contents
    9) Test FLASH memory
    0) Load U-boot
    w) Program CPLD
    e) Run board support package
    q) Quit
    """
    answer = getkey()
  
    if '1' in answer:
      sn = set_serial_number()
      sn_set = True
    if '2' in answer:
      try:
        if not sn_set:
          print '    WARNING: Serial number not set, press \'1\' to set or any key to use default...'
          answer = getkey()
          if '1' in answer:
            done = False
            while not done:
              sn = set_serial_number()
              print '    Is this correct? Press any key or \'n\' to re-enter serial number.'
              answer = getkey()
              if 'n' not in answer:
                done = True
        # open ftdi interface B in i2c mode
        print '    Connecting to I2C bus...',
        i2c_bus = open_ftdi_b()
        print 'done.'
        print '    Configuring current monitor...'
        config_cmon();
        print '    Configuring voltage monitor...'
        config_vmon();
        # open ftdi interface D
        f = open_ftdi_d()
        test_power()
        scan_jtag_chain()
        program_eeprom(sn)

      finally:
        ftdi.ftdi_usb_close(f)
        i2c_bus.Close()
    elif '3' in answer:
      try:
        i2c_bus = open_ftdi_b()
        f = open_ftdi_d()
        press_pb('on')
      finally:
        ftdi.ftdi_usb_close(f)
        i2c_bus.Close()
    elif '4' in answer:
      try:
        i2c_bus = open_ftdi_b()
        f = open_ftdi_d()
        press_pb('off')
      finally:
        ftdi.ftdi_usb_close(f)
        i2c_bus.Close()
    elif '5' in answer:
      try:
        i2c_bus = open_ftdi_b()
        curr_state = read_vmon_gpio('ATX_PG')
        print '    Current board state is %s.' %state[curr_state]
        print_v_c()
        print_temps()
      finally:
        i2c_bus.Close()
    elif '6' in answer:
      scan_jtag_chain()
    elif '7' in answer:
      try:
        i2c_bus = open_ftdi_b()
        program_eeprom(sn)
      finally:
        i2c_bus.Close()
    elif '8' in answer:
      try:
        i2c_bus = open_ftdi_b()
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
      finally:
        i2c_bus.Close()
    elif '9' in answer:
      try:
        ser = open_ftdi_uart(ser_port, baud)
        load_ppc('support_files/flashck.mac')
        tout = 0
        buff = []
        while tout < 3:
          res = ser.read(1)
          if res == '':
            tout = tout + 1
          else:
            sys.stdout.write(res)
            sys.stdout.flush()
            buff.append(res)
            tout = 0
        out = ''.join(buff)
        if out.find('pass') == -1:
          raise Exception, ('FATAL: FLASH memory test failed.')
        print 'FLASH memory test passed.'
        print ''
        print ''

      finally:
        ser.close()
    elif '0' in answer:
      try:
        ser = open_ftdi_uart(ser_port, baud)
        xio = xtx.Xmodem_tx(ser, defs.UBOOT_PATH, fh)
        load_ppc('support_files/program.mac')
        #load_urj('support_files/clear_reset.urj')
        ser.flushInput()
        ser.flushOutput()
        xio.xmdm_send()
        find_str_ser(ser, 'stop autoboot:', 3)
        ser.write('\n')    
        ser.write('run clearenv\n')
        print_outp_ser(ser, 1)
        ser.write('reset\n')
        find_str_ser(ser, 'stop autoboot:', 1)
        ser.write('\n')
        ser.write('saveenv\n')
        print_outp_ser(ser, 1)
        print ''
        print ''
      finally:
        ser.close()
    elif 'w' in answer:
      try:
        i2c_bus = open_ftdi_b()
        f = open_ftdi_d()
        ser = open_ftdi_uart(ser_port, baud)
        ser.flushInput()
        ser.flushOutput()
        press_pb('on')
        print '    Erasing CPLD...',
        load_urj('support_files/erase_cpld.urj')
        print 'done.'
        print '    Programming CPLD...',
        load_urj('support_files/program_cpld.urj')
        print 'done.'
        print ''
        print ''
        press_pb('off')
        press_pb('on')
        find_str_ser(ser, 'stop autoboot:', 3)
        ser.write('\n')
        time.sleep(1)
        # read data from CPLD to see if it programmed correctly  
        ser.write('md 0xc0000000 8\n')
        out = print_outp_ser(ser, 1)
        print ''
        if out.find(defs.CPLD_MD) == -1:
          raise Exception, ('FATAL: CPLD did not program correctly.')
      finally:
        ftdi.ftdi_usb_close(f)
        i2c_bus.Close()
        ser.close()
    elif 'e' in answer:
      try:
        i2c_bus = open_ftdi_b()
        f = open_ftdi_d()
        ser = open_ftdi_uart(ser_port, baud)
        press_pb('off')
        press_pb('on')
        find_str_ser(ser, 'stop autoboot:', 3)
        ser.write('\n')
        time.sleep(0.5)
        ser.write('dhcp\n')
        find_str_ser(ser, 'Bytes transferred', 10)
        time.sleep(0.5)
        ser.write('tftp 100000 roach2_bsp.bin\n')
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


        print ''
        print ''
      finally:
        ftdi.ftdi_usb_close(f)
        i2c_bus.Close()
        ser.close()
    elif 'q' in answer:
      quit = True
