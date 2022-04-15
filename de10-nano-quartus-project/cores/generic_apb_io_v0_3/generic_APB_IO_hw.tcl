# TCL File Generated by Component Editor 19.1
# Wed Dec 16 18:58:03 CET 2020
# DO NOT MODIFY


# 
# generic_APB_IO "generic APB I/O" v0.3
# Clemens Wegener (CHAIR.AUDIO) 2020.12.16.18:58:03
# 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module generic_APB_IO
# 
set_module_property DESCRIPTION ""
set_module_property NAME generic_APB_IO
set_module_property VERSION 0.3
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP I/O
set_module_property AUTHOR "Clemens Wegener (CHAIR.AUDIO)"
set_module_property DISPLAY_NAME "generic APB I/O"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL generic_apb_io
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file generic_APB_IO.v VERILOG PATH generic_APB_IO.v TOP_LEVEL_FILE


# 
# parameters
# 


# 
# display items
# 


# 
# connection point reset
# 
add_interface reset reset end
set_interface_property reset associatedClock clock
set_interface_property reset synchronousEdges DEASSERT
set_interface_property reset ENABLED true
set_interface_property reset EXPORT_OF ""
set_interface_property reset PORT_NAME_MAP ""
set_interface_property reset CMSIS_SVD_VARIABLES ""
set_interface_property reset SVD_ADDRESS_GROUP ""

add_interface_port reset reset_n reset_n Input 1


# 
# connection point s0
# 
add_interface s0 apb end
set_interface_property s0 associatedClock clock
set_interface_property s0 associatedReset reset
set_interface_property s0 ENABLED true
set_interface_property s0 EXPORT_OF ""
set_interface_property s0 PORT_NAME_MAP ""
set_interface_property s0 CMSIS_SVD_VARIABLES ""
set_interface_property s0 SVD_ADDRESS_GROUP ""

add_interface_port s0 aps_s0_paddr paddr Input 5
add_interface_port s0 aps_s0_psel psel Input 1
add_interface_port s0 aps_s0_penable penable Input 1
add_interface_port s0 aps_s0_pwrite pwrite Input 1
add_interface_port s0 aps_s0_pwdata pwdata Input 32
add_interface_port s0 aps_s0_prdata prdata Output 32
add_interface_port s0 aps_s0_pready pready Output 1


# 
# connection point clock
# 
add_interface clock clock end
set_interface_property clock clockRate 0
set_interface_property clock ENABLED true
set_interface_property clock EXPORT_OF ""
set_interface_property clock PORT_NAME_MAP ""
set_interface_property clock CMSIS_SVD_VARIABLES ""
set_interface_property clock SVD_ADDRESS_GROUP ""

add_interface_port clock clock_clk clk Input 1


# 
# connection point IO
# 
add_interface IO conduit end
set_interface_property IO associatedClock clock
set_interface_property IO associatedReset reset
set_interface_property IO ENABLED true
set_interface_property IO EXPORT_OF ""
set_interface_property IO PORT_NAME_MAP ""
set_interface_property IO CMSIS_SVD_VARIABLES ""
set_interface_property IO SVD_ADDRESS_GROUP ""

add_interface_port IO data_in data_in Input 32
add_interface_port IO data_out data_out Output 32
add_interface_port IO strobe strobe Output 1

