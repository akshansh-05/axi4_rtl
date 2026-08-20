// File: tb_top.sv
// Description: Top-level Testbench instantiating DUTs and Interfaces
//              Connects AXI4 DMA (Master) and AXI4 RAM (Slave) via Interfaces

`timescale 1ns / 1ns

`include "uvm_macros.svh"
import uvm_pkg::*;

module tb_top;

    // 1. Parameter Definitions
    localparam DATA_WIDTH        = 32;
    localparam ADDR_WIDTH        = 16;
    localparam STRB_WIDTH        = (DATA_WIDTH / 8);
    localparam ID_WIDTH          = 8;
    localparam MAX_BURST_LEN     = 16;
    localparam LEN_WIDTH         = 20;
    localparam TAG_WIDTH         = 8;
    localparam DEST_WIDTH        = 8;
    localparam USER_WIDTH        = 1;
    localparam PIPELINE_OUTPUT   = 0;

    // 2. Global Signals (Clock & Reset)
    logic clk;
    logic rst;

    // Clock Generation (10ns period -> 5ns toggle)
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Reset Generation (Active-high reset for 50ns)
    initial begin
        rst = 1'b1;
        #50;
        @(posedge clk); // de-assert the reset at posedge(synchronously)
        rst = 1'b0;
    end

    // 3. Interface Instantiations

    // AXI4 Memory-Mapped Bus (connects DMA Master to AXI RAM Slave)
    axi_if #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .STRB_WIDTH(STRB_WIDTH)
    ) 
    axi_bus (
        .clk(clk),
        .rst(rst)
    );

    // DMA Descriptor Command & Status Interface
    dma_desc_if #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH (LEN_WIDTH),
        .TAG_WIDTH (TAG_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .DEST_WIDTH(DEST_WIDTH),
        .USER_WIDTH(USER_WIDTH)
    ) 
    desc_bus (
        .clk(clk),
        .rst(rst)
    );

    // AXI-Stream Write Data Input (Stream into DMA -> Written to MM RAM)
    axis_if #(
        .DATA_WIDTH(DATA_WIDTH),
        .KEEP_WIDTH(STRB_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .DEST_WIDTH(DEST_WIDTH),
        .USER_WIDTH(USER_WIDTH)
    ) 
    axis_wr_bus (
        .clk(clk),
        .rst(rst)
    );

    // AXI-Stream Read Data Output (Read from MM RAM -> Streamed out of DMA)
    axis_if #(
        .DATA_WIDTH(DATA_WIDTH),
        .KEEP_WIDTH(STRB_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .DEST_WIDTH(DEST_WIDTH),
        .USER_WIDTH(USER_WIDTH)
    ) 
    axis_rd_bus (
        .clk(clk),
        .rst(rst)
    );

    // 4. DUT 1: AXI4 DMA Engine (Master)
    axi_dma #(
        .AXI_DATA_WIDTH   (DATA_WIDTH),
        .AXI_ADDR_WIDTH   (ADDR_WIDTH),
        .AXI_STRB_WIDTH   (STRB_WIDTH),
        .AXI_ID_WIDTH     (ID_WIDTH),
        .AXI_MAX_BURST_LEN(MAX_BURST_LEN),
        .AXIS_DATA_WIDTH  (DATA_WIDTH),
        .AXIS_KEEP_ENABLE (1),
        .AXIS_KEEP_WIDTH  (STRB_WIDTH),
        .AXIS_LAST_ENABLE (1),
        .AXIS_ID_ENABLE   (1),
        .AXIS_ID_WIDTH    (ID_WIDTH),
        .AXIS_DEST_ENABLE (1),
        .AXIS_DEST_WIDTH  (DEST_WIDTH),
        .AXIS_USER_ENABLE (1),
        .AXIS_USER_WIDTH  (USER_WIDTH),
        .LEN_WIDTH        (LEN_WIDTH),
        .TAG_WIDTH        (TAG_WIDTH),
        .ENABLE_SG        (0),
        .ENABLE_UNALIGNED (1)
    ) 
    axi_dma_dut (
        .clk  (clk),
        .rst  (rst),

        // Read Descriptor Command
        .s_axis_read_desc_addr  (desc_bus.read_desc_addr),
        .s_axis_read_desc_len   (desc_bus.read_desc_len),
        .s_axis_read_desc_tag   (desc_bus.read_desc_tag),
        .s_axis_read_desc_id    (desc_bus.read_desc_id),
        .s_axis_read_desc_dest  (desc_bus.read_desc_dest),
        .s_axis_read_desc_user  (desc_bus.read_desc_user),
        .s_axis_read_desc_valid (desc_bus.read_desc_valid),
        .s_axis_read_desc_ready (desc_bus.read_desc_ready),

        // Read Descriptor Status
        .m_axis_read_desc_status_tag   (desc_bus.read_desc_status_tag),
        .m_axis_read_desc_status_error (desc_bus.read_desc_status_error),
        .m_axis_read_desc_status_valid (desc_bus.read_desc_status_valid),

        // Stream Read Data Output
        .m_axis_read_data_tdata (axis_rd_bus.tdata),
        .m_axis_read_data_tkeep (axis_rd_bus.tkeep),
        .m_axis_read_data_tvalid(axis_rd_bus.tvalid),
        .m_axis_read_data_tready(axis_rd_bus.tready),
        .m_axis_read_data_tlast (axis_rd_bus.tlast),
        .m_axis_read_data_tid   (axis_rd_bus.tid),
        .m_axis_read_data_tdest (axis_rd_bus.tdest),
        .m_axis_read_data_tuser (axis_rd_bus.tuser),

        // Write Descriptor Command
        .s_axis_write_desc_addr (desc_bus.write_desc_addr),
        .s_axis_write_desc_len  (desc_bus.write_desc_len),
        .s_axis_write_desc_tag  (desc_bus.write_desc_tag),
        .s_axis_write_desc_valid(desc_bus.write_desc_valid),
        .s_axis_write_desc_ready(desc_bus.write_desc_ready),

        // Write Descriptor Status
        .m_axis_write_desc_status_len  (desc_bus.write_desc_status_len),
        .m_axis_write_desc_status_tag  (desc_bus.write_desc_status_tag),
        .m_axis_write_desc_status_id   (desc_bus.write_desc_status_id),
        .m_axis_write_desc_status_dest (desc_bus.write_desc_status_dest),
        .m_axis_write_desc_status_user (desc_bus.write_desc_status_user),
        .m_axis_write_desc_status_error(desc_bus.write_desc_status_error),
        .m_axis_write_desc_status_valid(desc_bus.write_desc_status_valid),

        // Stream Write Data Input
        .s_axis_write_data_tdata (axis_wr_bus.tdata),
        .s_axis_write_data_tkeep (axis_wr_bus.tkeep),
        .s_axis_write_data_tvalid(axis_wr_bus.tvalid),
        .s_axis_write_data_tready(axis_wr_bus.tready),
        .s_axis_write_data_tlast (axis_wr_bus.tlast),
        .s_axis_write_data_tid   (axis_wr_bus.tid),
        .s_axis_write_data_tdest (axis_wr_bus.tdest),
        .s_axis_write_data_tuser (axis_wr_bus.tuser),

        // AXI4 Master: Write Address Channel (AW)
        .m_axi_awid   (axi_bus.awid),
        .m_axi_awaddr (axi_bus.awaddr),
        .m_axi_awlen  (axi_bus.awlen),
        .m_axi_awsize (axi_bus.awsize),
        .m_axi_awburst(axi_bus.awburst),
        .m_axi_awvalid(axi_bus.awvalid),
        .m_axi_awready(axi_bus.awready),

        // AXI4 Master: Write Data Channel (W)
        .m_axi_wdata  (axi_bus.wdata),
        .m_axi_wstrb  (axi_bus.wstrb),
        .m_axi_wlast  (axi_bus.wlast),
        .m_axi_wvalid (axi_bus.wvalid),
        .m_axi_wready (axi_bus.wready),

        // AXI4 Master: Write Response Channel (B)
        .m_axi_bid    (axi_bus.bid),
        .m_axi_bresp  (axi_bus.bresp),
        .m_axi_bvalid (axi_bus.bvalid),
        .m_axi_bready (axi_bus.bready),

        // AXI4 Master: Read Address Channel (AR)
        .m_axi_arid   (axi_bus.arid),
        .m_axi_araddr (axi_bus.araddr),
        .m_axi_arlen  (axi_bus.arlen),
        .m_axi_arsize (axi_bus.arsize),
        .m_axi_arburst(axi_bus.arburst),
        .m_axi_arvalid(axi_bus.arvalid),
        .m_axi_arready(axi_bus.arready),

        // AXI4 Master: Read Data Channel (R)
        .m_axi_rid    (axi_bus.rid),
        .m_axi_rdata  (axi_bus.rdata),
        .m_axi_rresp  (axi_bus.rresp),
        .m_axi_rlast  (axi_bus.rlast),
        .m_axi_rvalid (axi_bus.rvalid),
        .m_axi_rready (axi_bus.rready),

        // Global DMA Controls
        .read_enable  (desc_bus.read_enable),
        .write_enable (desc_bus.write_enable),
        .write_abort  (desc_bus.write_abort)
    );

    // 5. DUT 2: AXI4 RAM Memory (Slave)
    axi_ram #(
        .DATA_WIDTH     (DATA_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .STRB_WIDTH     (STRB_WIDTH),
        .ID_WIDTH       (ID_WIDTH),
        .PIPELINE_OUTPUT(PIPELINE_OUTPUT)
    ) 
    axi_ram_dut (
        .clk(clk),
        .rst(rst),

        // AXI4 Slave: Write Address Channel (AW)
        .s_axi_awid   (axi_bus.awid),
        .s_axi_awaddr (axi_bus.awaddr),
        .s_axi_awlen  (axi_bus.awlen),
        .s_axi_awsize (axi_bus.awsize),
        .s_axi_awburst(axi_bus.awburst),
        .s_axi_awvalid(axi_bus.awvalid),
        .s_axi_awready(axi_bus.awready),

        // AXI4 Slave: Write Data Channel (W)
        .s_axi_wdata  (axi_bus.wdata),
        .s_axi_wstrb  (axi_bus.wstrb),
        .s_axi_wlast  (axi_bus.wlast),
        .s_axi_wvalid (axi_bus.wvalid),
        .s_axi_wready (axi_bus.wready),

        // AXI4 Slave: Write Response Channel (B)
        .s_axi_bid    (axi_bus.bid),
        .s_axi_bresp  (axi_bus.bresp),
        .s_axi_bvalid (axi_bus.bvalid),
        .s_axi_bready (axi_bus.bready),

        // AXI4 Slave: Read Address Channel (AR)
        .s_axi_arid   (axi_bus.arid),
        .s_axi_araddr (axi_bus.araddr),
        .s_axi_arlen  (axi_bus.arlen),
        .s_axi_arsize (axi_bus.arsize),
        .s_axi_arburst(axi_bus.arburst),
        .s_axi_arvalid(axi_bus.arvalid),
        .s_axi_arready(axi_bus.arready),

        // AXI4 Slave: Read Data Channel (R)
        .s_axi_rid    (axi_bus.rid),
        .s_axi_rdata  (axi_bus.rdata),
        .s_axi_rresp  (axi_bus.rresp),
        .s_axi_rlast  (axi_bus.rlast),
        .s_axi_rvalid (axi_bus.rvalid),
        .s_axi_rready (axi_bus.rready)
    );

    // 6. UVM Config DB Registration & Simulation Start
    initial begin
        // Register virtual interfaces in UVM config database
        uvm_config_db#(virtual axi_if)::set(null, "*", "vif_axi", axi_bus);
        uvm_config_db#(virtual dma_desc_if)::set(null, "*", "vif_desc", desc_bus);
        uvm_config_db#(virtual axis_if)::set(null, "*", "vif_axis_wr", axis_wr_bus);
        uvm_config_db#(virtual axis_if)::set(null, "*", "vif_axis_rd", axis_rd_bus);

        // Optional VCD waveform dump
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_top);

        // Start UVM phase execution
        run_test();

        // End simulation
        $finish;
    end

endmodule : tb_top
