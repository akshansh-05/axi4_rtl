#!/bin/csh

# ============================================================
# AXI DMA UVM Verification - Xcelium Run Script
# ============================================================

# ------------------------------------------------------------
# Load Cadence environment
# ------------------------------------------------------------

source /home2/Housekeeping/env_scripts/cshrc_IC618

# ------------------------------------------------------------
# Project / Run directories
# ------------------------------------------------------------

# Directory containing this script (run_dir)
set RUN_DIR = "$cwd"

# Project root is one directory above run_dir
set PROJECT_DIR = "$RUN_DIR:h"

set RTL_DIR = "$PROJECT_DIR/rtl"
set UVM_DIR = "$PROJECT_DIR/uvm"

set FILELIST = "$RUN_DIR/filelist.f"
set LOG_FILE = "$RUN_DIR/xrun.log"

# ------------------------------------------------------------
# Move to run directory
# ------------------------------------------------------------

cd "$RUN_DIR"

# ------------------------------------------------------------
# Display information
# ------------------------------------------------------------

echo ""
echo "============================================================"
echo "           AXI DMA UVM VERIFICATION"
echo "============================================================"
echo ""
echo "Project Directory : $PROJECT_DIR"
echo "RTL Directory     : $RTL_DIR"
echo "UVM Directory     : $UVM_DIR"
echo "Run Directory     : $RUN_DIR"
echo ""
echo "============================================================"
echo ""

# ------------------------------------------------------------
# Check filelist
# ------------------------------------------------------------

if (! -f "$FILELIST") then
    echo "[ERROR] filelist.f not found!"
    echo "Expected:"
    echo "$FILELIST"
    exit 1
endif

# ------------------------------------------------------------
# Clean previous simulation database
# ------------------------------------------------------------

echo "[INFO] Removing previous simulation database..."

rm -rf xcelium.d
rm -rf waves.shm
rm -rf dump.vcd xrun.history sanity.history sanity.log

echo ""

# ------------------------------------------------------------
# Check Xcelium
# ------------------------------------------------------------

echo "[INFO] Xcelium executable:"
which xrun

echo ""

# ------------------------------------------------------------
# Run Xcelium
# ------------------------------------------------------------

echo "[INFO] Starting Xcelium..."
echo ""

xrun \
    -64bit \
    -sv \
    -uvm \
    -timescale 1ns/1ns \
    -access +rwc \
    -f "$FILELIST" \
    -l "$LOG_FILE" \
    $argv:q

# ------------------------------------------------------------
# Simulation completed
# ------------------------------------------------------------

echo ""
echo "============================================================"
echo "             SIMULATION COMPLETED"
echo "============================================================"
echo ""
echo "Log file:"
echo "    $LOG_FILE"
echo ""
echo "Waveform:"
echo "    $RUN_DIR/waves.shm"
echo ""
echo "To open SimVision:"
echo "    simvision waves.shm &"
echo ""
echo "============================================================"
