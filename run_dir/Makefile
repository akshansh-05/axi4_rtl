# ============================================================
# Makefile for Cadence Xcelium (Run from run_dir/)
# ============================================================

XRUN_FLAGS = -64bit -sv -uvm -timescale 1ns/1ns -access +rwc -f filelist.f -l xrun.log

all: run

run:
	xrun $(XRUN_FLAGS)

gui:
	xrun $(XRUN_FLAGS) -gui

clean:
	rm -rf xcelium.d waves.shm dump.vcd xrun.log xrun.history sanity.history sanity.log *.key
