#scan_chain.py

import jtag_chain
import defs_r2_ats as defs

jch = jtag_chain.Jtag_chain(defs.BSDL_PATH)
jch.scan_chain()

