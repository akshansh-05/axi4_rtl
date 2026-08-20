// ===============================================================================
// Author:       Akshansh Chaurasia
// Supervisor:   Dr. Rakesh Palisetty
// Institution:  Shiv Nadar University
// Department:   Department of Electrical Engineering
// Project:      Minor Project
// Description:  Top-level AXI4 Direct Memory Access (DMA) engine integrating read
//               and write channels to transfer data between AXI4 Memory-Mapped
//               and AXI4-Stream interfaces.
//
// NOTE ON REMOVED ADVANCED SIGNALS:
// The unused AXI4 system signals (AWLOCK, AWCACHE, AWPROT, ARLOCK, ARCACHE, ARPROT)
// have been removed from this module port list to simplify RTL inspection and waveform
// viewing. The internal DMA engine does not utilize these attributes.
// Language:     Verilog 2001
// ===============================================================================

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * AXI4 DMA
 */
module axi_dma #
(
    // Width of AXI data bus in bits
    parameter AXI_DATA_WIDTH = 32,
    // Width of AXI address bus in bits
    parameter AXI_ADDR_WIDTH = 16,
    // Width of AXI wstrb (width of data bus in words)
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // Width of AXI ID signal
    parameter AXI_ID_WIDTH = 8,
    // Maximum AXI burst length to generate
    parameter AXI_MAX_BURST_LEN = 16,
    // Width of AXI stream interfaces in bits
    parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH,
    // Use AXI stream tkeep signal
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // AXI stream tkeep signal width (words per cycle)
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // Use AXI stream tlast signal
    parameter AXIS_LAST_ENABLE = 1,
    // Propagate AXI stream tid signal
    parameter AXIS_ID_ENABLE = 0,
    // AXI stream tid signal width
    parameter AXIS_ID_WIDTH = 8,
    // Propagate AXI stream tdest signal
    parameter AXIS_DEST_ENABLE = 0,
    // AXI stream tdest signal width
    parameter AXIS_DEST_WIDTH = 8,
    // Propagate AXI stream tuser signal
    parameter AXIS_USER_ENABLE = 1,
    // AXI stream tuser signal width
    parameter AXIS_USER_WIDTH = 1,
    // Width of length field
    parameter LEN_WIDTH = 20,
    // Width of tag field
    parameter TAG_WIDTH = 8,
    // Enable support for scatter/gather DMA
    // (multiple descriptors per AXI stream frame)
    parameter ENABLE_SG = 0,
    // Enable support for unaligned transfers
    parameter ENABLE_UNALIGNED = 0
)
(
    input  wire                       clk,
    input  wire                       rst,

    /*
     * AXI read descriptor input //done
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_read_desc_addr,
    input  wire [LEN_WIDTH-1:0]       s_axis_read_desc_len,
    input  wire [TAG_WIDTH-1:0]       s_axis_read_desc_tag,
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_read_desc_id,
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_read_desc_dest,
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_read_desc_user,
    input  wire                       s_axis_read_desc_valid,
    output wire                       s_axis_read_desc_ready,

    /*
     * AXI read descriptor status output // done
     */
    output wire [TAG_WIDTH-1:0]       m_axis_read_desc_status_tag,
    output wire [3:0]                 m_axis_read_desc_status_error,
    output wire                       m_axis_read_desc_status_valid,

    /*
     * AXI stream read data output // done 
     */
    output wire [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata,
    output wire [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep,
    output wire                       m_axis_read_data_tvalid,
    input  wire                       m_axis_read_data_tready,
    output wire                       m_axis_read_data_tlast,
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid,
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest,
    output wire [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser,

    /*
     * AXI write descriptor input
     */
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_write_desc_addr,
    input  wire [LEN_WIDTH-1:0]       s_axis_write_desc_len,
    input  wire [TAG_WIDTH-1:0]       s_axis_write_desc_tag,
    input  wire                       s_axis_write_desc_valid,
    output wire                       s_axis_write_desc_ready,

    /*
     * AXI write descriptor status output
     */
    output wire [LEN_WIDTH-1:0]       m_axis_write_desc_status_len,
    output wire [TAG_WIDTH-1:0]       m_axis_write_desc_status_tag,
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_write_desc_status_id,
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest,
    output wire [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user,
    output wire [3:0]                 m_axis_write_desc_status_error,
    output wire                       m_axis_write_desc_status_valid,

    /*
     * AXI stream write data input
     */
    input  wire [AXIS_DATA_WIDTH-1:0] s_axis_write_data_tdata,
    input  wire [AXIS_KEEP_WIDTH-1:0] s_axis_write_data_tkeep,
    input  wire                       s_axis_write_data_tvalid,
    output wire                       s_axis_write_data_tready,
    input  wire                       s_axis_write_data_tlast,
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_write_data_tid,
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_write_data_tdest,
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_write_data_tuser,

    // =========================================================================
    // 1. AXI4 Write Address Channel (AW)
    // =========================================================================
    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid,    // Write Address ID tag
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,  // Write Address
    output wire [7:0]                 m_axi_awlen,   // Burst Length (number of transfers - 1)
    output wire [2:0]                 m_axi_awsize,  // Burst Size (bytes per transfer)
    output wire [1:0]                 m_axi_awburst, // Burst Type (FIXED, INCR, WRAP)
    output wire                       m_axi_awvalid, // Write Address Valid signal
    input  wire                       m_axi_awready, // Write Address Ready signal

    // =========================================================================
    // 2. AXI4 Write Data Channel (W)
    // =========================================================================
    output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata,   // Write Data payload
    output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb,   // Write Byte Strobes (1 bit per byte)
    output wire                       m_axi_wlast,   // Write Last transfer indicator
    output wire                       m_axi_wvalid,  // Write Data Valid signal
    input  wire                       m_axi_wready,  // Write Data Ready signal

    // =========================================================================
    // 3. AXI4 Write Response Channel (B)
    // =========================================================================
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,     // Write Response ID tag (matches AWID)
    input  wire [1:0]                 m_axi_bresp,   // Write Response status (OKAY, EXOKAY, SLVERR, DECERR)
    input  wire                       m_axi_bvalid,  // Write Response Valid signal
    output wire                       m_axi_bready,  // Write Response Ready signal

    // =========================================================================
    // 4. AXI4 Read Address Channel (AR)
    // =========================================================================
    output wire [AXI_ID_WIDTH-1:0]    m_axi_arid,    // Read Address ID tag
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,  // Read Address
    output wire [7:0]                 m_axi_arlen,   // Burst Length (number of transfers - 1)
    output wire [2:0]                 m_axi_arsize,  // Burst Size (bytes per transfer)
    output wire [1:0]                 m_axi_arburst, // Burst Type (FIXED, INCR, WRAP)
    output wire                       m_axi_arvalid, // Read Address Valid signal
    input  wire                       m_axi_arready, // Read Address Ready signal

    // =========================================================================
    // 5. AXI4 Read Data Channel (R)
    // =========================================================================
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid,     // Read Response ID tag (matches ARID)
    input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata,   // Read Data payload
    input  wire [1:0]                 m_axi_rresp,   // Read Response status (OKAY, EXOKAY, SLVERR, DECERR)
    input  wire                       m_axi_rlast,   // Read Last transfer indicator
    input  wire                       m_axi_rvalid,  // Read Data Valid signal
    output wire                       m_axi_rready,  // Read Data Ready signal

    /*
     * Configuration
     */
    input  wire                       read_enable,
    input  wire                       write_enable,
    input  wire                       write_abort
);

axi_dma_rd #(
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_ENABLE(AXIS_KEEP_ENABLE),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_ID_WIDTH(AXIS_ID_WIDTH),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_DEST_WIDTH(AXIS_DEST_WIDTH),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE),
    .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
    .LEN_WIDTH(LEN_WIDTH),
    .TAG_WIDTH(TAG_WIDTH),
    .ENABLE_SG(ENABLE_SG),
    .ENABLE_UNALIGNED(ENABLE_UNALIGNED)
)
axi_dma_rd_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI read descriptor input
     */
    .s_axis_read_desc_addr(s_axis_read_desc_addr),
    .s_axis_read_desc_len(s_axis_read_desc_len),
    .s_axis_read_desc_tag(s_axis_read_desc_tag),
    .s_axis_read_desc_id(s_axis_read_desc_id),
    .s_axis_read_desc_dest(s_axis_read_desc_dest),
    .s_axis_read_desc_user(s_axis_read_desc_user),
    .s_axis_read_desc_valid(s_axis_read_desc_valid),
    .s_axis_read_desc_ready(s_axis_read_desc_ready),

    /*
     * AXI read descriptor status output
     */
    .m_axis_read_desc_status_tag(m_axis_read_desc_status_tag),
    .m_axis_read_desc_status_error(m_axis_read_desc_status_error),
    .m_axis_read_desc_status_valid(m_axis_read_desc_status_valid),

    /*
     * AXI stream read data output
     */
    .m_axis_read_data_tdata(m_axis_read_data_tdata),
    .m_axis_read_data_tkeep(m_axis_read_data_tkeep),
    .m_axis_read_data_tvalid(m_axis_read_data_tvalid),
    .m_axis_read_data_tready(m_axis_read_data_tready),
    .m_axis_read_data_tlast(m_axis_read_data_tlast),
    .m_axis_read_data_tid(m_axis_read_data_tid),
    .m_axis_read_data_tdest(m_axis_read_data_tdest),
    .m_axis_read_data_tuser(m_axis_read_data_tuser),

    /*
     * AXI master interface
     */
    .m_axi_arid(m_axi_arid),
    .m_axi_araddr(m_axi_araddr),
    .m_axi_arlen(m_axi_arlen),
    .m_axi_arsize(m_axi_arsize),
    .m_axi_arburst(m_axi_arburst),
    .m_axi_arvalid(m_axi_arvalid),
    .m_axi_arready(m_axi_arready),
    .m_axi_rid(m_axi_rid),
    .m_axi_rdata(m_axi_rdata),
    .m_axi_rresp(m_axi_rresp),
    .m_axi_rlast(m_axi_rlast),
    .m_axi_rvalid(m_axi_rvalid),
    .m_axi_rready(m_axi_rready),

    /*
     * Configuration
     */
    .enable(read_enable)
);

axi_dma_wr #(
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
    .AXI_ID_WIDTH(AXI_ID_WIDTH),
    .AXI_MAX_BURST_LEN(AXI_MAX_BURST_LEN),
    .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
    .AXIS_KEEP_ENABLE(AXIS_KEEP_ENABLE),
    .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH),
    .AXIS_LAST_ENABLE(AXIS_LAST_ENABLE),
    .AXIS_ID_ENABLE(AXIS_ID_ENABLE),
    .AXIS_ID_WIDTH(AXIS_ID_WIDTH),
    .AXIS_DEST_ENABLE(AXIS_DEST_ENABLE),
    .AXIS_DEST_WIDTH(AXIS_DEST_WIDTH),
    .AXIS_USER_ENABLE(AXIS_USER_ENABLE),
    .AXIS_USER_WIDTH(AXIS_USER_WIDTH),
    .LEN_WIDTH(LEN_WIDTH),
    .TAG_WIDTH(TAG_WIDTH),
    .ENABLE_SG(ENABLE_SG),
    .ENABLE_UNALIGNED(ENABLE_UNALIGNED)
)
axi_dma_wr_inst (
    .clk(clk),
    .rst(rst),

    /*
     * AXI write descriptor input
     */
    .s_axis_write_desc_addr(s_axis_write_desc_addr),
    .s_axis_write_desc_len(s_axis_write_desc_len),
    .s_axis_write_desc_tag(s_axis_write_desc_tag),
    .s_axis_write_desc_valid(s_axis_write_desc_valid),
    .s_axis_write_desc_ready(s_axis_write_desc_ready),

    /*
     * AXI write descriptor status output
     */
    .m_axis_write_desc_status_len(m_axis_write_desc_status_len),
    .m_axis_write_desc_status_tag(m_axis_write_desc_status_tag),
    .m_axis_write_desc_status_id(m_axis_write_desc_status_id),
    .m_axis_write_desc_status_dest(m_axis_write_desc_status_dest),
    .m_axis_write_desc_status_user(m_axis_write_desc_status_user),
    .m_axis_write_desc_status_error(m_axis_write_desc_status_error),
    .m_axis_write_desc_status_valid(m_axis_write_desc_status_valid),

    /*
     * AXI stream write data input
     */
    .s_axis_write_data_tdata(s_axis_write_data_tdata),
    .s_axis_write_data_tkeep(s_axis_write_data_tkeep),
    .s_axis_write_data_tvalid(s_axis_write_data_tvalid),
    .s_axis_write_data_tready(s_axis_write_data_tready),
    .s_axis_write_data_tlast(s_axis_write_data_tlast),
    .s_axis_write_data_tid(s_axis_write_data_tid),
    .s_axis_write_data_tdest(s_axis_write_data_tdest),
    .s_axis_write_data_tuser(s_axis_write_data_tuser),

    /*
     * AXI master interface
     */
    .m_axi_awid(m_axi_awid),
    .m_axi_awaddr(m_axi_awaddr),
    .m_axi_awlen(m_axi_awlen),
    .m_axi_awsize(m_axi_awsize),
    .m_axi_awburst(m_axi_awburst),
    .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awready(m_axi_awready),
    .m_axi_wdata(m_axi_wdata),
    .m_axi_wstrb(m_axi_wstrb),
    .m_axi_wlast(m_axi_wlast),
    .m_axi_wvalid(m_axi_wvalid),
    .m_axi_wready(m_axi_wready),
    .m_axi_bid(m_axi_bid),
    .m_axi_bresp(m_axi_bresp),
    .m_axi_bvalid(m_axi_bvalid),
    .m_axi_bready(m_axi_bready),

    /*
     * Configuration
     */
    .enable(write_enable),
    .abort(write_abort)
);

endmodule

`resetall
