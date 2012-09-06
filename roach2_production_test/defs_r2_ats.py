#defs_r2_ats.py
import defs_max16071, i2c_functions

# Serial number dictionary
SN_REV1 = { 
    'manufacturer' : 0x44, 
    'type'         : 0x01, 
    'revision'     : 0x01, 
    'batch'        : 0x00, 
    'board'        : 0x00 
  } 
SN_REV2 = { 
    'manufacturer' : 0x44, 
    'type'         : 0x01, 
    'revision'     : 0x02, 
    'batch'        : 0x00, 
    'board'        : 0x00 
  } 

# Manufacturer codes
MANUF = {
  'Elprom'  : 0x45,
  'Digicom' : 0x44
}

# Voltage monitor mappings to channel on max16071 (I2C addresses: 0x50, 0x51)
u16 = i2c_functions.ADDR_V_MON
u23 = i2c_functions.ADDR_C_MON
V_MON_MAP = {
  '1V0'     : [1, u16],
  '1V5'     : [2, u16],
  '1V8'     : [3, u16],
  '2V5'     : [4, u16],
  '3V3'     : [5, u16],
  '5V0'     : [6, u16],
  '12V'     : [7, u16],
  '3V3_AUX' : [8, u16],
  '5V0_AUX' : [7, u23]
}
 # Votage monitor GPIO mappings
V_MON_GPIO = {
  'MGT_1V2_PG' : 2,
  'MGT_1V0_PG' : 3,
  'ATX_PWR_OK' : 4
}

# Current monitor mappings to channel on max16071
C_MON_MAP = {
  '3V3'        : 1,
  '2V5'        : 2,
  '1V8'        : 3,
  '1V5'        : 4,
  '1V0'        : 5
}

# FTDI VID and PID
R2_VID = 0x403
R2_PID = 0x6011

# ATX power supply switch on to PG delay in seconds
PG_DELAY = 0.4
PB_DELAY = 0.2

# Linux and U-Boot boot delay in seconds
BOOT_DELAY = 60
UBOOT_DELAY = 10

# Unconfigured current thresholds
UC_NOM_C_12V = 0.8 
UC_NOM_C_5V0 = 0.8
UC_NOM_C_3V3 = 0.14
UC_NOM_C_2V5 = 1.13
UC_NOM_C_1V8 = 1.2
UC_NOM_C_1V5 = 2.4
UC_NOM_C_1V0 = 2.9
TOL_UC = 3
UC_C_THRESHOLD = {
  '12V_H'     : UC_NOM_C_12V + UC_NOM_C_12V*TOL_UC,
  '12V_L'     : UC_NOM_C_12V - UC_NOM_C_12V*TOL_UC,
  '5V0_H'     : UC_NOM_C_5V0 + UC_NOM_C_5V0*TOL_UC,
  '5V0_L'     : UC_NOM_C_5V0 - UC_NOM_C_5V0*TOL_UC,
  '3V3_H'     : UC_NOM_C_3V3 + UC_NOM_C_3V3*TOL_UC,
  '3V3_L'     : UC_NOM_C_3V3 - UC_NOM_C_3V3*TOL_UC,
  '2V5_H'     : UC_NOM_C_2V5 + UC_NOM_C_2V5*TOL_UC,
  '2V5_L'     : UC_NOM_C_2V5 - UC_NOM_C_2V5*TOL_UC,
  '1V8_H'     : UC_NOM_C_1V8 + UC_NOM_C_1V8*TOL_UC,
  '1V8_L'     : UC_NOM_C_1V8 - UC_NOM_C_1V8*TOL_UC,
  '1V5_H'     : UC_NOM_C_1V5 + UC_NOM_C_1V5*TOL_UC,
  '1V5_L'     : UC_NOM_C_1V5 - UC_NOM_C_1V5*TOL_UC,
  '1V0_H'     : UC_NOM_C_1V0 + UC_NOM_C_1V0*TOL_UC,
  '1V0_L'     : UC_NOM_C_1V0 - UC_NOM_C_1V0*TOL_UC
}

# Voltage thresholds, mods for rev1: 3V3_AUX is 12V and 12V = 0
TOL_V = 0.04
DIV = defs_max16071.V_DIV_12V
V_THRESHOLD_REV1 = {
  '5V0_H'     : 5.0 + 5.0*TOL_V,
  '5V0_L'     : 5.0 - 5.0*TOL_V,
  '3V3_H'     : 3.3 + 3.3*TOL_V,
  '3V3_L'     : 3.3 - 3.3*TOL_V,
  '2V5_H'     : 2.5 + 2.5*TOL_V,
  '2V5_L'     : 2.5 - 2.5*TOL_V,
  '1V8_H'     : 1.8 + 1.8*TOL_V,
  '1V8_L'     : 1.8 - 1.8*TOL_V,
  '1V5_H'     : 1.5 + 1.5*TOL_V,
  '1V5_L'     : 1.5 - 1.5*TOL_V,
  '1V0_H'     : 1.0 + 1.0*TOL_V,
  '1V0_L'     : 1.0 - 1.0*TOL_V,
  '3V3_AUX_H' : 12.0/DIV + (12.0/DIV)*TOL_V,
  '3V3_AUX_L' : 12.0/DIV - (12.0/DIV)*TOL_V
}
V_THRESHOLD_REV2 = {
  '12V_H'     : 12.0 + 12.0*TOL_V,
  '12V_L'     : 12.0 - 12.0*TOL_V,
  '5V0_H'     : 5.0  + 5.0*TOL_V,
  '5V0_L'     : 5.0  - 5.0*TOL_V,
  '3V3_H'     : 3.3  + 3.3*TOL_V,
  '3V3_L'     : 3.3  - 3.3*TOL_V,
  '2V5_H'     : 2.5  + 2.5*TOL_V,
  '2V5_L'     : 2.5  - 2.5*TOL_V,
  '1V8_H'     : 1.8  + 1.8*TOL_V,
  '1V8_L'     : 1.8  - 1.8*TOL_V,
  '1V5_H'     : 1.5  + 1.5*TOL_V,
  '1V5_L'     : 1.5  - 1.5*TOL_V,
  '1V0_H'     : 1.0  + 1.0*TOL_V,
  '1V0_L'     : 1.0  - 1.0*TOL_V,
  '3V3_AUX_H' : 3.4  + 3.4*TOL_V,
  '3V3_AUX_L' : 3.4  - 3.4*TOL_V,
  '5V0_AUX_H' : 5.0  + 5.0*TOL_V,
  '5V0_AUX_L' : 5.0  - 5.0*TOL_V
}

# Temperature tolerances
T_THRESHOLD = {
  'PPC_T_H'    : 60,
  'PPC_T_L'    : 10,
  'FPGA_T_H'   : 60,
  'FPGA_T_L'   : 10,
  'INLET_T_H'  : 40,
  'INLET_T_L'  : 10,
  'OUTLET_T_H' : 40,
  'OUTLET_T_L' : 10
}

# Fan tolerances
FPGA_F_SPEED = 5730
CHS0_F_SPEED = 15000
CHS1_F_SPEED = 15000
CHS2_F_SPEED = 15000

TOL_F = 0.2
F_THRESHOLD = {
  'FPGA_FAN' : FPGA_F_SPEED - FPGA_F_SPEED*TOL_F,
  'CHS0_FAN' : CHS0_F_SPEED - CHS0_F_SPEED*TOL_F,
  'CHS1_FAN' : CHS1_F_SPEED - CHS1_F_SPEED*TOL_F,
  'CHS2_FAN' : CHS2_F_SPEED - CHS2_F_SPEED*TOL_F
}

#Boot configuration for EEPROM boot (500MHz CPU, 166MHz DRAM, 83MHz bus)
CONFIG_G_BOOT = [0x87, 0x78, 0x52, 0x4e, 0x0d, 0x57, 0xa0, 0x30, 0x40, 0x08, 0x23, 0x50, 0x0d, 0x05, 0x00, 0x00]

#BSDL path
BSDL_PATH = 'support_files/bsdl'

#U-boot path
UBOOT_REV1 = '/home/nfs/roach2/boot/u-boot-r2-rev1-nofpu.bin'
UBOOT_REV1_MEMTEST = '/home/nfs/roach2/boot/u-boot-memtest-r2-rev1.bin'
UBOOT_REV2 = '/home/nfs/roach2/boot/u-boot-r2-rev2.bin'
UBOOT_REV2_MEMTEST = '/home/nfs/roach2/boot/u-boot-memtest-r2-rev2.bin'

#QDR test boffile
QDR_TST_BOF = 'qdr_tst_r2_2x_2012_May_07_1216.bof'

# Set xilinx source path
XILINX_SRC_PATH = '/opt/Xilinx/12.4/LabTools/settings64.sh'

#CPLD memory dump sample
CPLD_MD = 'c0000000: 010f0000 03000000 010f0000 03000000    ................'

#JTAG scan chain output sample
JTAG_SCAN_RENESAS ='Connected to libftdi driver.\nIR length: 64\nChain length: 11\nDevice Id: 00011000000000000110000110010111 (0x18006197)\n  Filename:     support_files/bsdl/MAX16071.BSD\nDevice Id: 00011000000000000110000110010111 (0x18006197)\n  Filename:     support_files/bsdl/MAX16071.BSD\nDevice Id: 00000000000000000000001111010011 (0x000003D3)\n  Filename:     support_files/bsdl/marvell_guess.bsd\nDevice Id: 00000000000000000000001111010011 (0x000003D3)\n  Filename:     support_files/bsdl/marvell_guess.bsd\nDevice Id: 00001011011011111110010001000111 (0x0B6FE447)\n  Filename:     support_files/bsdl/R1QDA7236.bsd\nDevice Id: 00001011011011111110010001000111 (0x0B6FE447)\n  Filename:     support_files/bsdl/R1QDA7236.bsd\nDevice Id: 00001011011011111110010001000111 (0x0B6FE447)\n  Filename:     support_files/bsdl/R1QDA7236.bsd\nDevice Id: 00001011011011111110010001000111 (0x0B6FE447)\n  Filename:     support_files/bsdl/R1QDA7236.bsd\nDevice Id: 00010110110101001010000010010011 (0x16D4A093)\n  Filename:     support_files/bsdl/XC2C256_VQ100.bsdl\nDevice Id: 01100100001010001000000010010011 (0x64288093)\n  Filename:     support_files/bsdl/xc6vsx475t_ff1759.bsd\nDevice Id: 00000100010000001111000111100001 (0x0440F1E1)\n  Filename:     support_files/bsdl/ct_wrap_440EPx_B_Full.bsd\n'

JTAG_SCAN_CYPRESS = 'Connected to libftdi driver.\nIR length: 64\nChain length: 11\nDevice Id: 00011000000000000110000110010111 (0x18006197)\n  Filename:     support_files/bsdl/MAX16071.BSD\nDevice Id: 00011000000000000110000110010111 (0x18006197)\n  Filename:     support_files/bsdl/MAX16071.BSD\nDevice Id: 00000000000000000000001111010011 (0x000003D3)\n  Filename:     support_files/bsdl/marvell_guess.bsd\nDevice Id: 00000000000000000000001111010011 (0x000003D3)\n  Filename:     support_files/bsdl/marvell_guess.bsd\nDevice Id: 00011010010001100100000001101001 (0x1A464069)\n  Filename:     support_files/bsdl/25652kv18_x36_165.bsd\nDevice Id: 00011010010001100100000001101001 (0x1A464069)\n  Filename:     support_files/bsdl/25652kv18_x36_165.bsd\nDevice Id: 00011010010001100100000001101001 (0x1A464069)\n  Filename:     support_files/bsdl/25652kv18_x36_165.bsd\nDevice Id: 00011010010001100100000001101001 (0x1A464069)\n  Filename:     support_files/bsdl/25652kv18_x36_165.bsd\nDevice Id: 00010110110101001010000010010011 (0x16D4A093)\n  Filename:     support_files/bsdl/XC2C256_VQ100.bsdl\nDevice Id: 01100100001010001000000010010011 (0x64288093)\n  Filename:     support_files/bsdl/xc6vsx475t_ff1759.bsd\nDevice Id: 00000100010000001111000111100001 (0x0440F1E1)\n  Filename:     support_files/bsdl/ct_wrap_440EPx_B_Full.bsd\n'

