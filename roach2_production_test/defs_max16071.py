#defs_max16071.py

#I2C command 
MON_1_MSB = 0x00
MON_1_LSB = 0x01
MON_2_MSB = 0x02
MON_2_LSB = 0x03
MON_3_MSB = 0x04
MON_3_LSB = 0x05
MON_4_MSB = 0x06
MON_4_LSB = 0x07
MON_5_MSB = 0x08
MON_5_LSB = 0x09
MON_6_MSB = 0x0a
MON_6_LSB = 0x0b
MON_7_MSB = 0x0c
MON_7_LSB = 0x0d
MON_8_MSB = 0x0e
MON_8_LSB = 0x0f
MON_C = 0x18
MON_CSP_MSB = 0x19
MON_CSP_LSB = 0x1a
GPIO_INPUT_STATE = 0x1e
GPIO_CONF_123 = 0x3f
GPIO_CONF_3456 = 0x40
GPIO_CONF_678 = 0x41
ADC_CONF_4321 = 0x43
ADC_CONF_8765 = 0x44
OCPT_CSC1 = 0x47
SW_EN_CONF = 0x73
CH_ARRAY = (MON_1_MSB, MON_1_LSB, MON_2_MSB, MON_2_LSB, MON_3_MSB, MON_3_LSB, MON_4_MSB, MON_4_LSB, MON_5_MSB, MON_5_LSB, MON_6_MSB, MON_6_LSB, MON_7_MSB, MON_7_LSB, MON_8_MSB, MON_8_LSB)
#Current sense gain lookup table
CURR_SNS_GAIN = 6,12,24,48
#Full-scale ADC range lookup table
ADC_RNG = 5.6,2.8,1.4,0.0
#Current sense resistors
SNS_RES_12V = 0.005
SNS_RES_5V0 = 0.002
SNS_RES_3V3 = 0.002
SNS_RES_2V5 = 0.005
SNS_RES_1V8 = 0.002
SNS_RES_1V5 = 0.001
SNS_RES_1V0 = 0.00025
SNS_RES = (SNS_RES_3V3, SNS_RES_2V5, SNS_RES_1V8, SNS_RES_1V5, SNS_RES_1V0)
#INA333 Amplifier gain resistors
GAIN_RES_3V3 = 200.0
GAIN_RES_2V5 = 499.0
GAIN_RES_1V8 = 200.0
GAIN_RES_1V5 = 100.0
GAIN_RES_1V0 = 100.0
GAIN_RES = (GAIN_RES_3V3, GAIN_RES_2V5, GAIN_RES_1V8, GAIN_RES_1V5, GAIN_RES_1V0)
#12V voltage dividor
V_DIV_12V = (19600 + 10000)/10000.0

#Configuration 

#Overcurrent secondary threshold deglitch = 4ms, Watchdog timer boots after 
#sequence completes, Early warning is undervoltage, Margin mode disabled, Enabled
SW_EN_CONF_VAL = 0x21

#Overcurrent primary threshold and current-sense gain setting = 25mV & 48V/V, 
#CSP full-scale range is 14V, Current sense enabled.
OCPT_CSC1_VAL = 0x0F

#ADC full-scale config, all to 5.6V
ADC_CONF_4321_VAL = 0x00
ADC_CONF_8765_VAL = 0x00 

