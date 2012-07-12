# i2c_functions.py
import time


def i2c_regwrite_raw(device, i2c_address, register, data):
  """Write register on I2C bus.

  Keyword arguments:
  device -- object of type mpsse.MPSSE
  i2c_address -- 7 bit I2C device address
  register -- 8 bit register address
  data -- 8 bit data

  """

  addr_write = i2c_address * 2
  
  device.Start()
  device.Write(chr(addr_write))
  if device.GetAck() == 0:
    device.Write(chr(register))
  else:
    device.Stop()
    raise RuntimeError("Failed to ack write address.")
  if device.GetAck() == 0:
    device.Write(chr(data))
  else:
    device.Stop()
    raise RuntimeError("Failed to ack register.")
  if device.GetAck() == 0:
    device.Stop()
  else:
    device.Stop()
    raise RuntimeError("Failed to ack data.")
  return "Success" 

def i2c_regwrite(device, i2c_address, register, data, retry = 1, backoff = 1):
  """Write register on I2C bus.

  Keyword arguments:
  device -- object of type mpsse.MPSSE
  i2c_address -- 7 bit I2C device address
  register -- 8 bit register address
  data -- 8 bit data
  retry -- number of retries
  backoff -- time between retries in seconds

  """
  count = 0
  success = False
  while count < retry:
    try:
      result = i2c_regwrite_raw(device, i2c_address, register, data)
      success = True
      count = retry + 1
    except RuntimeError as e:
      print 'I2C write error: %s' %e
      time.sleep(backoff)
      count = count + 1
  if not success:
    raise RuntimeError('Error: I2C write failed.') 
  else:
    return result

def i2c_regread_raw(device, i2c_address, register):
  """Read register on I2C bus.

  Keyword arguments:
  device -- object of type mpsse.MPSSE
  i2c_address -- 7 bit I2C device address
  register -- 8 bit register address

  """

  addr_write = i2c_address * 2
  addr_read  = addr_write + 1
  
  device.Start()
  device.Write(chr(addr_write))
  if device.GetAck() == 0:
    device.Write(chr(register))
  else:
    raise RuntimeError("Failed to ack write address.")
  if device.GetAck() == 0:
    device.Start()
    device.Write(chr(addr_read))
    device.SetAck(1)
    rd_byte = device.Read(1)
    device.Stop()
  else:
    raise RuntimeError("Failed to ack read address.")
  return ord(rd_byte)

def i2c_regread(device, i2c_address, register, retry = 5, backoff = 1):
  """Read register on I2C bus.

  Keyword arguments:
  device -- object of type mpsse.MPSSE
  i2c_address -- 7 bit I2C device address
  register -- 8 bit register address
  retry -- number of retries
  backoff -- time between retries in seconds

  """
  count = 0
  success = False
  while count < retry:
    try:
      rd_byte = i2c_regread_raw(device, i2c_address, register)
      success = True
      count = retry + 1
    except RuntimeError as e:
      print 'I2C read error: %s' %e
      time.sleep(backoff)
      count = count + 1
  if not success:
    raise RuntimeError('Error: I2C read failed.') 
  else:
    return rd_byte
  
def i2c_regread2b_raw(device, i2c_address, register):
  """Read register twice on I2C bus.

  Keyword arguments:
  device -- object of type mpsse.MPSSE
  i2c_address -- 7 bit I2C device address
  register -- 8 bit register address
  returns -- two byte array, [0] = first read, [1] = second read

  """

  addr_write = i2c_address * 2
  addr_read  = addr_write + 1
  
  device.Start()
  device.Write(chr(addr_write))
  if device.GetAck() == 0:
    device.Write(chr(register))
  else:
    raise RuntimeError("Failed to ack write address.")
  if device.GetAck() == 0:
    device.Start()
    device.Write(chr(addr_read))
    device.SetAck(0)
    rd_byte1 = device.Read(1)
    device.SetAck(1)
    rd_byte2 = device.Read(1)
    device.Stop()
  else:
    raise RuntimeError("Failed to ack read address.")
  return (ord(rd_byte1), ord(rd_byte2))
  
def i2c_regread2b(device, i2c_address, register, retry = 5, backoff = 1):
  """Read register on I2C bus.

  Keyword arguments:
  device -- object of type mpsse.MPSSE
  i2c_address -- 7 bit I2C device address
  register -- 8 bit register address
  retry -- number of retries
  backoff -- time between retries in seconds
  returns -- two byte array, [0] = first read, [1] = second read

  """
  count = 0
  success = False
  while count < retry:
    try:
      rd_byte = i2c_regread2b_raw(device, i2c_address, register)
      success = True
      count = retry + 1
    except RuntimeError as e:
      print 'I2C read error: %s' %e
      time.sleep(backoff)
      count = count + 1
  if not success:
    raise RuntimeError('Error: I2C read failed.') 
  else:
    return rd_byte

# I2C addresses on ROACH2 bus
ADDR_PPC_FPGA_TEMP = 0x18
ADDR_FAN1 = 0x1b
ADDR_FAN2 = 0x1f
ADDR_FAN_FPGA = 0x48
ADDR_FAN0 = 0x4b
ADDR_INLET_TEMP = 0x4c
ADDR_OUTLET_TEMP = 0x4e
ADDR_V_MON = 0x50
ADDR_C_MON = 0x51
ADDR_BOOT_EEPROM_0 = 0x54
ADDR_BOOT_EEPROM_1 = 0x55
ADDR_RTC = 0x68


