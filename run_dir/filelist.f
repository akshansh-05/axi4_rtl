# ============================================================
# Cadence Xcelium Filelist (Run from run_dir/)
# ============================================================

# Include directories
-incdir ../uvm
-incdir ../rtl/axi_master_rtl
-incdir ../rtl/axi_slave_rtl
-incdir ../axi_master_rtl
-incdir ../axi_slave_rtl

# RTL Sources (relative to run_dir/)
../rtl/axi_master_rtl/axi_dma_rd.v
../rtl/axi_master_rtl/axi_dma_wr.v
../rtl/axi_master_rtl/axi_dma.v
../rtl/axi_slave_rtl/axi_ram.v

# UVM / SystemVerilog Interfaces
../uvm/axi_if.sv
../uvm/axis_if.sv
../uvm/dma_desc_if.sv

# Top Testbench
../uvm/tb_top.sv
