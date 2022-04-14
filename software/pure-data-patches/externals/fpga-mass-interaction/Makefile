# Makefile to build class 'helloworld' for Pure Data.
# Needs Makefile.pdlibbuilder as helper makefile for platform-dependent build
# settings and rules.

# library name
lib.name = fpga-mass-interaction~

# input source file (class name == source file basename)
class.sources = fpga_mass_interaction~.c

# all extra files to be included in binary distribution of the library
datafiles = fpga_mass_interaction~-help.pd fpga_mass_interaction~-meta.pd README.md

# include Makefile.pdlibbuilder from submodule directory 'pd-lib-builder'
PDLIBBUILDER_DIR=pd-lib-builder/
include $(PDLIBBUILDER_DIR)/Makefile.pdlibbuilder
