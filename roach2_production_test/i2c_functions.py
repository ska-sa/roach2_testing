# i2c_functions.py

def i2c_regwrite(device, i2c_address, register, data):
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

def i2c_regread(device, i2c_address, register):
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
  
def i2c_regread2b(device, i2c_address, register):
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


