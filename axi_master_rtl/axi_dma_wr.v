// Author:       Akshansh Chaurasia
// Supervisor:   Dr. Rakesh Palisetty
// Institution:  Shiv Nadar University
// Department:   Department of Electrical Engineering
// Project:      Minor Project
// Description:  AXI4 DMA Write Engine
//               Bridges an incoming AXI4-Stream data source to an AXI4 Memory-Mapped
//               master write interface according to descriptor instructions.
//
// Verification Overview:
// - Protocol Handshakes:
//     * Descriptor Channel (s_axis_write_desc_*): Valid/Ready handshake to fetch
//       start address, byte length, and transaction tag.
//     * Status Channel (m_axis_write_desc_status_*): Single-cycle pulse reporting
//       total transferred bytes, descriptor tag, sideband signals, and error codes.
//     * Stream Data Slave (s_axis_write_data_*): Receives payload data with TKEEP
//       and TLAST. Throttled via TREADY when downstream AXI or internal FIFOs backpressure.
//     * AXI4 Master AW/W/B Channels: Generates INCR write bursts, streams write beats
//       with valid byte strobes, and retires transfers upon receiving B-channel responses.
// - Protocol Constraints & Corner Cases:,                                                 
//     * 4KB Boundary Crossing: Automatically splits bursts so no AXI transfer crosses
//       a 4KB page boundary (per ARM AXI4 spec section A3.4.1).
//     * Burst Slicing: Splits large transfers into bursts of at most AXI_MAX_BURST_LEN beats.
//     * Address Alignment: When ENABLE_UNALIGNED=1, aligns unaligned byte addresses
//       to bus words via an internal barrel shifter and applies appropriate WSTRB masks.
//     * Early Stream Termination: If stream asserts TLAST before the promised burst completes,
//       the engine zero-pads remaining beats (WSTRB=0) to maintain AXI protocol compliance.
//     * Excess Stream Data: If stream supplies more data than descriptor length, extra beats
//       are cleanly drained in STATE_DROP_DATA until TLAST is reached.
//     * Response Tracking: Uses status_fifo to track in-flight bursts and associate B-channel
//       responses (OKAY, SLVERR, DECERR) with the originating descriptor.
//
// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

module axi_dma_wr #
(
    // Width of AXI memory-mapped data bus in bits (e.g., 32, 64, 128)
    parameter AXI_DATA_WIDTH = 32,
    // Width of AXI address bus in bits
    parameter AXI_ADDR_WIDTH = 16,
    // Width of AXI write strobe bus (1 bit per data byte)
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // Width of AXI ID tag (for AWID and BID)
    parameter AXI_ID_WIDTH = 8,
    // Maximum burst length in beats per generated AXI burst (1 to 256)
    parameter AXI_MAX_BURST_LEN = 16,
    // Width of AXI-Stream data interface in bits (must match AXI_DATA_WIDTH)
    parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH,
    // Enable TKEEP byte qualifiers on the stream interface
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // Width of stream TKEEP signal (1 bit per byte)
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // Enable TLAST packet boundary detection on stream interface
    parameter AXIS_LAST_ENABLE = 1,
    // Enable propagation of stream TID identifier to status
    parameter AXIS_ID_ENABLE = 0,
    parameter AXIS_ID_WIDTH = 8,
    // Enable propagation of stream TDEST routing tag to status
    parameter AXIS_DEST_ENABLE = 0,
    parameter AXIS_DEST_WIDTH = 8,
    // Enable propagation of stream TUSER sideband signal to status
    parameter AXIS_USER_ENABLE = 1,
    parameter AXIS_USER_WIDTH = 1,
    // Width of transfer length field in bytes (e.g. 20 bits allows up to 1MB)
    parameter LEN_WIDTH = 20,
    // Width of descriptor identification tag
    parameter TAG_WIDTH = 8,
    // Enable support for scatter/gather DMA (not implemented in this core)
    parameter ENABLE_SG = 0,
    // Enable barrel-shifting datapath for unaligned byte start addresses
    parameter ENABLE_UNALIGNED = 0
)
(
    input  wire                       clk,
    input  wire                       rst,

    // Write Descriptor Input Interface
    // Handshake: Valid/Ready. Latched when s_axis_write_desc_valid & s_axis_write_desc_ready.
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_write_desc_addr,   // Start address in AXI memory space
    input  wire [LEN_WIDTH-1:0]       s_axis_write_desc_len,    // Total transfer size in bytes
    input  wire [TAG_WIDTH-1:0]       s_axis_write_desc_tag,    // User tag reflected in completion status
    input  wire                       s_axis_write_desc_valid,  // Descriptor valid indicator
    output wire                       s_axis_write_desc_ready,  // Ready to accept new descriptor

    // Write Descriptor Completion Status Output Interface
    // Pulses valid for one cycle upon retirement of the final burst of a descriptor.
    output wire [LEN_WIDTH-1:0]       m_axis_write_desc_status_len,   // Actual byte count transferred
    output wire [TAG_WIDTH-1:0]       m_axis_write_desc_status_tag,   // Tag of finished descriptor
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_write_desc_status_id,    // Latch of stream TID
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest,  // Latch of stream TDEST
    output wire [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user,  // Latch of stream TUSER
    output wire [3:0]                 m_axis_write_desc_status_error, // Completion error code
    output wire                       m_axis_write_desc_status_valid, // Status valid strobe

    // AXI-Stream Slave Write Data Interface
    // Source data packet to write to memory. Backpressured via tready if downstream stalls.
    input  wire [AXIS_DATA_WIDTH-1:0] s_axis_write_data_tdata,  // Stream payload
    input  wire [AXIS_KEEP_WIDTH-1:0] s_axis_write_data_tkeep,  // Byte qualifies (valid byte enables)
    input  wire                       s_axis_write_data_tvalid, // Stream data valid
    output wire                       s_axis_write_data_tready, // Stream data ready (backpressure to source)
    input  wire                       s_axis_write_data_tlast,  // End of packet marker
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_write_data_tid,    // Stream source ID
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_write_data_tdest,  // Stream destination routing
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_write_data_tuser,  // User sideband metadata

    // AXI4 Master Write Address Channel (AW)
    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid,    // Address write ID (tied to 0)
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,  // Burst start address
    output wire [7:0]                 m_axi_awlen,   // Burst length: beat count minus 1 (0 = 1 beat)
    output wire [2:0]                 m_axi_awsize,  // Beat size: log2(bytes per beat), e.g. 3'b010 = 4B
    output wire [1:0]                 m_axi_awburst, // Burst type: 2'b01 = INCR (incremental addressing)
    output wire                       m_axi_awvalid, // Write address valid
    input  wire                       m_axi_awready, // Slave write address ready

    // AXI4 Master Write Data Channel (W)
    output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata,   // Write data beat
    output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb,   // Byte lane enables (active high per byte)
    output wire                       m_axi_wlast,   // Last beat indicator for active burst
    output wire                       m_axi_wvalid,  // Write data valid
    input  wire                       m_axi_wready,  // Slave write data ready

    // AXI4 Master Write Response Channel (B)
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,     // Response ID (matches AWID)
    input  wire [1:0]                 m_axi_bresp,   // Response status: 2'b00=OKAY, 2'b10=SLVERR, 2'b11=DECERR
    input  wire                       m_axi_bvalid,  // Response valid from memory slave
    output wire                       m_axi_bready,  // Master ready to receive write response

    // Operational Controls
    input  wire                       enable,        // Master enable (gates descriptor acceptance)
    input  wire                       abort          // Abort signal (reserved for error recovery)
);

// Bus width and sizing parameters
parameter AXI_WORD_WIDTH = AXI_STRB_WIDTH;
parameter AXI_WORD_SIZE = AXI_DATA_WIDTH/AXI_WORD_WIDTH; // Width of a single byte/word lane (usually 8)
parameter AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);       // Encoded transfer size (AWSIZE = log2(STRB_WIDTH))
parameter AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN << AXI_BURST_SIZE; // Max byte capacity per burst

parameter AXIS_KEEP_WIDTH_INT = AXIS_KEEP_ENABLE ? AXIS_KEEP_WIDTH : 1;
parameter AXIS_WORD_WIDTH = AXIS_KEEP_WIDTH_INT;
parameter AXIS_WORD_SIZE = AXIS_DATA_WIDTH/AXIS_WORD_WIDTH;

// Offset masks for isolating byte-level alignment within a word
parameter OFFSET_WIDTH = AXI_STRB_WIDTH > 1 ? $clog2(AXI_STRB_WIDTH) : 1;
parameter OFFSET_MASK = AXI_STRB_WIDTH > 1 ? {OFFSET_WIDTH{1'b1}} : 0;
parameter ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH); // Truncates unaligned low address bits
parameter CYCLE_COUNT_WIDTH = LEN_WIDTH - AXI_BURST_SIZE + 1;

// Internal FIFO depth parameters
parameter STATUS_FIFO_ADDR_WIDTH = 5; // Tracks up to 32 outstanding AXI bursts waiting for B-channel response
parameter OUTPUT_FIFO_ADDR_WIDTH = 5; // 32-entry skid buffer on W channel to decouple FSM from bus stalls

// Elaboration-time structural assertions
initial begin
    if (AXI_WORD_SIZE * AXI_STRB_WIDTH != AXI_DATA_WIDTH) begin
        $error("Error: AXI data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXIS_WORD_SIZE * AXIS_KEEP_WIDTH_INT != AXIS_DATA_WIDTH) begin
        $error("Error: AXI stream data width not evenly divisble (instance %m)");
        $finish;
    end

    if (AXI_WORD_SIZE != AXIS_WORD_SIZE) begin
        $error("Error: word size mismatch (instance %m)");
        $finish;
    end

    if (2**$clog2(AXI_WORD_WIDTH) != AXI_WORD_WIDTH) begin
        $error("Error: AXI word width must be even power of two (instance %m)");
        $finish;
    end

    if (AXI_DATA_WIDTH != AXIS_DATA_WIDTH) begin
        $error("Error: AXI interface width must match AXI stream interface width (instance %m)");
        $finish;
    end

    if (AXI_MAX_BURST_LEN < 1 || AXI_MAX_BURST_LEN > 256) begin
        $error("Error: AXI_MAX_BURST_LEN must be between 1 and 256 (instance %m)");
        $finish;
    end

    if (ENABLE_SG) begin
        $error("Error: scatter/gather is not yet implemented (instance %m)");
        $finish;
    end
end

// AXI4 Response Status Encodings (BRESP)
localparam [1:0]
    AXI_RESP_OKAY   = 2'b00, // Normal access success
    AXI_RESP_EXOKAY = 2'b01, // Exclusive access success
    AXI_RESP_SLVERR = 2'b10, // Slave error (slave unable to complete write)
    AXI_RESP_DECERR = 2'b11; // Decode error (no slave found at target address)

// Status Error Codes reported on m_axis_write_desc_status_error
localparam [3:0]
    DMA_ERROR_NONE              = 4'd0, // Successful completion
    DMA_ERROR_TIMEOUT           = 4'd1,
    DMA_ERROR_PARITY            = 4'd2,
    DMA_ERROR_AXI_RD_SLVERR     = 4'd4,
    DMA_ERROR_AXI_RD_DECERR     = 4'd5,
    DMA_ERROR_AXI_WR_SLVERR     = 4'd6, // Slave returned SLVERR on B channel
    DMA_ERROR_AXI_WR_DECERR     = 4'd7, // Interconnect returned DECERR on B channel
    DMA_ERROR_PCIE_FLR          = 4'd8,
    DMA_ERROR_PCIE_CPL_POISONED = 4'd9,
    DMA_ERROR_PCIE_CPL_STATUS_UR = 4'd10,
    DMA_ERROR_PCIE_CPL_STATUS_CA = 4'd11;

// Main FSM State Definitions
localparam [2:0]
    STATE_IDLE         = 3'd0, // Awaiting valid write descriptor and credit availability
    STATE_START        = 3'd1, // Burst calculation: 4KB boundary enforcement & AW issuing
    STATE_WRITE        = 3'd2, // Streaming data beats from AXIS to AXI W channel
    STATE_FINISH_BURST = 3'd3, // Zero-padding dummy beats if stream ends before burst completes
    STATE_DROP_DATA    = 3'd4; // Draining excess stream beats if stream exceeds descriptor length

reg [2:0] state_reg = STATE_IDLE, state_next;

// Datapath control strobes
reg transfer_in_save;
reg flush_save;
reg status_fifo_we;

integer i;
reg [OFFSET_WIDTH:0] cycle_size;

// Address and transfer progress registers
reg [AXI_ADDR_WIDTH-1:0] addr_reg = {AXI_ADDR_WIDTH{1'b0}}, addr_next;         // Active AXI burst start address
reg [LEN_WIDTH-1:0] op_word_count_reg = {LEN_WIDTH{1'b0}}, op_word_count_next; // Bytes remaining for current descriptor
reg [LEN_WIDTH-1:0] tr_word_count_reg = {LEN_WIDTH{1'b0}}, tr_word_count_next; // Bytes assigned to current AXI burst

// Alignment, strobe masking, and cycle counting registers
reg [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next;                 // Start byte offset within word
reg [AXI_STRB_WIDTH-1:0] strb_offset_mask_reg = {AXI_STRB_WIDTH{1'b1}}, strb_offset_mask_next; // First beat strobe mask
reg zero_offset_reg = 1'b1, zero_offset_next;                                          // True if word-aligned (offset==0)
reg [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next;   // Final beat byte mask
reg [LEN_WIDTH-1:0] length_reg = {LEN_WIDTH{1'b0}}, length_next;                       // Accumulated transferred byte counter
reg [CYCLE_COUNT_WIDTH-1:0] input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, input_cycle_count_next;   // Stream beats remaining
reg [CYCLE_COUNT_WIDTH-1:0] output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, output_cycle_count_next; // AXI beats remaining (AWLEN)
reg input_active_reg = 1'b0, input_active_next;           // Stream input acceptance active
reg first_cycle_reg = 1'b0, first_cycle_next;             // High during first beat of a transfer
reg input_last_cycle_reg = 1'b0, input_last_cycle_next;   // Stream last cycle reached
reg output_last_cycle_reg = 1'b0, output_last_cycle_next; // AXI last beat reached (triggers WLAST)
reg last_transfer_reg = 1'b0, last_transfer_next;         // Final burst of the current descriptor
reg [1:0] bresp_reg = AXI_RESP_OKAY, bresp_next;          // Latched write response status

// Sideband metadata storage
reg [TAG_WIDTH-1:0] tag_reg = {TAG_WIDTH{1'b0}}, tag_next;
reg [AXIS_ID_WIDTH-1:0] axis_id_reg = {AXIS_ID_WIDTH{1'b0}}, axis_id_next;
reg [AXIS_DEST_WIDTH-1:0] axis_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, axis_dest_next;
reg [AXIS_USER_WIDTH-1:0] axis_user_reg = {AXIS_USER_WIDTH{1'b0}}, axis_user_next;

// Status Tracking FIFO
// Holds metadata for issued AXI bursts to associate incoming B-channel responses with descriptors.
reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] status_fifo_wr_ptr_reg = 0;
reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] status_fifo_rd_ptr_reg = 0, status_fifo_rd_ptr_next;
reg [LEN_WIDTH-1:0] status_fifo_len[(2**STATUS_FIFO_ADDR_WIDTH)-1:0];
reg [TAG_WIDTH-1:0] status_fifo_tag[(2**STATUS_FIFO_ADDR_WIDTH)-1:0];
reg [AXIS_ID_WIDTH-1:0] status_fifo_id[(2**STATUS_FIFO_ADDR_WIDTH)-1:0];
reg [AXIS_DEST_WIDTH-1:0] status_fifo_dest[(2**STATUS_FIFO_ADDR_WIDTH)-1:0];
reg [AXIS_USER_WIDTH-1:0] status_fifo_user[(2**STATUS_FIFO_ADDR_WIDTH)-1:0];
reg status_fifo_last[(2**STATUS_FIFO_ADDR_WIDTH)-1:0];
reg [LEN_WIDTH-1:0] status_fifo_wr_len;
reg [TAG_WIDTH-1:0] status_fifo_wr_tag;
reg [AXIS_ID_WIDTH-1:0] status_fifo_wr_id;
reg [AXIS_DEST_WIDTH-1:0] status_fifo_wr_dest;
reg [AXIS_USER_WIDTH-1:0] status_fifo_wr_user;
reg status_fifo_wr_last;

// In-Flight Burst Credit Counter
// Ensures we never issue more bursts than the status_fifo can accommodate.
reg [STATUS_FIFO_ADDR_WIDTH+1-1:0] active_count_reg = 0;
reg active_count_av_reg = 1'b1; // High if status_fifo has space for more bursts
reg inc_active;
reg dec_active;

// Port registered outputs
reg s_axis_write_desc_ready_reg = 1'b0, s_axis_write_desc_ready_next;

reg [LEN_WIDTH-1:0] m_axis_write_desc_status_len_reg = {LEN_WIDTH{1'b0}}, m_axis_write_desc_status_len_next;
reg [TAG_WIDTH-1:0] m_axis_write_desc_status_tag_reg = {TAG_WIDTH{1'b0}}, m_axis_write_desc_status_tag_next;
reg [AXIS_ID_WIDTH-1:0] m_axis_write_desc_status_id_reg = {AXIS_ID_WIDTH{1'b0}}, m_axis_write_desc_status_id_next;
reg [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, m_axis_write_desc_status_dest_next;
reg [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user_reg = {AXIS_USER_WIDTH{1'b0}}, m_axis_write_desc_status_user_next;
reg [3:0] m_axis_write_desc_status_error_reg = 4'd0, m_axis_write_desc_status_error_next;
reg m_axis_write_desc_status_valid_reg = 1'b0, m_axis_write_desc_status_valid_next;

reg [AXI_ADDR_WIDTH-1:0] m_axi_awaddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_awaddr_next;
reg [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next;
reg m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next;
reg m_axi_bready_reg = 1'b0, m_axi_bready_next;

reg s_axis_write_data_tready_reg = 1'b0, s_axis_write_data_tready_next;

// Alignment Barrel-Shifter Registers
// save_axis_tdata_reg captures remaining partial bytes from the previous cycle so they
// can be concatenated and shifted with the current beat to realign to bus lanes.
reg [AXIS_DATA_WIDTH-1:0] save_axis_tdata_reg = {AXIS_DATA_WIDTH{1'b0}};
reg [AXIS_KEEP_WIDTH_INT-1:0] save_axis_tkeep_reg = {AXIS_KEEP_WIDTH_INT{1'b0}};
reg save_axis_tlast_reg = 1'b0;

reg [AXIS_DATA_WIDTH-1:0] shift_axis_tdata;
reg [AXIS_KEEP_WIDTH_INT-1:0] shift_axis_tkeep;
reg shift_axis_tvalid;
reg shift_axis_tlast;
reg shift_axis_input_tready;
reg shift_axis_extra_cycle_reg = 1'b0; // Flushes residual saved bytes on packet boundary

// Internal datapath wires driving the output skid FIFO
reg  [AXI_DATA_WIDTH-1:0] m_axi_wdata_int;
reg  [AXI_STRB_WIDTH-1:0] m_axi_wstrb_int;
reg                       m_axi_wlast_int;
reg                       m_axi_wvalid_int;
wire                      m_axi_wready_int;

// Port assignments
assign s_axis_write_desc_ready = s_axis_write_desc_ready_reg;

assign m_axis_write_desc_status_len   = m_axis_write_desc_status_len_reg;
assign m_axis_write_desc_status_tag   = m_axis_write_desc_status_tag_reg;
assign m_axis_write_desc_status_id    = m_axis_write_desc_status_id_reg;
assign m_axis_write_desc_status_dest  = m_axis_write_desc_status_dest_reg;
assign m_axis_write_desc_status_user  = m_axis_write_desc_status_user_reg;
assign m_axis_write_desc_status_error = m_axis_write_desc_status_error_reg;
assign m_axis_write_desc_status_valid = m_axis_write_desc_status_valid_reg;

assign s_axis_write_data_tready = s_axis_write_data_tready_reg;

assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awaddr  = m_axi_awaddr_reg;
assign m_axi_awlen   = m_axi_awlen_reg;
assign m_axi_awsize  = AXI_BURST_SIZE;
assign m_axi_awburst = 2'b01; // INCR burst type
assign m_axi_awvalid = m_axi_awvalid_reg;
assign m_axi_bready  = m_axi_bready_reg;

// Alignment Shifter Datapath
// Aligns incoming stream data according to the descriptor address offset.
always @* begin
    if (!ENABLE_UNALIGNED || zero_offset_reg) begin
        // Aligned transfer: straight passthrough of stream signals
        shift_axis_tdata        = s_axis_write_data_tdata;
        shift_axis_tkeep        = s_axis_write_data_tkeep;
        shift_axis_tvalid       = s_axis_write_data_tvalid;
        shift_axis_tlast        = AXIS_LAST_ENABLE && s_axis_write_data_tlast;
        shift_axis_input_tready = 1'b1;
    end else if (!AXIS_LAST_ENABLE) begin
        // Continuous unaligned stream without packet framing
        shift_axis_tdata        = {s_axis_write_data_tdata, save_axis_tdata_reg} >> ((AXIS_KEEP_WIDTH_INT-offset_reg)*AXIS_WORD_SIZE);
        shift_axis_tkeep        = {s_axis_write_data_tkeep, save_axis_tkeep_reg} >> (AXIS_KEEP_WIDTH_INT-offset_reg);
        shift_axis_tvalid       = s_axis_write_data_tvalid;
        shift_axis_tlast        = 1'b0;
        shift_axis_input_tready = 1'b1;
    end else if (shift_axis_extra_cycle_reg) begin
        // Extra cycle needed to flush unaligned residual bytes saved from previous cycle
        shift_axis_tdata        = {s_axis_write_data_tdata, save_axis_tdata_reg} >> ((AXIS_KEEP_WIDTH_INT-offset_reg)*AXIS_WORD_SIZE);
        shift_axis_tkeep        = {{AXIS_KEEP_WIDTH_INT{1'b0}}, save_axis_tkeep_reg} >> (AXIS_KEEP_WIDTH_INT-offset_reg);
        shift_axis_tvalid       = 1'b1;
        shift_axis_tlast        = save_axis_tlast_reg;
        shift_axis_input_tready = flush_save;
    end else begin
        // Normal unaligned shifting with packet framing (TLAST)
        shift_axis_tdata        = {s_axis_write_data_tdata, save_axis_tdata_reg} >> ((AXIS_KEEP_WIDTH_INT-offset_reg)*AXIS_WORD_SIZE);
        shift_axis_tkeep        = {s_axis_write_data_tkeep, save_axis_tkeep_reg} >> (AXIS_KEEP_WIDTH_INT-offset_reg);
        shift_axis_tvalid       = s_axis_write_data_tvalid;
        shift_axis_tlast        = (s_axis_write_data_tlast && ((s_axis_write_data_tkeep & ({AXIS_KEEP_WIDTH_INT{1'b1}} << (AXIS_KEEP_WIDTH_INT-offset_reg))) == 0));
        shift_axis_input_tready = !(s_axis_write_data_tlast && s_axis_write_data_tready && s_axis_write_data_tvalid);
    end
end

// Main Combinational FSM & Datapath Control Process
always @(*) begin
    state_next = STATE_IDLE;

    s_axis_write_desc_ready_next = 1'b0;

    m_axis_write_desc_status_len_next   = m_axis_write_desc_status_len_reg;
    m_axis_write_desc_status_tag_next   = m_axis_write_desc_status_tag_reg;
    m_axis_write_desc_status_id_next    = m_axis_write_desc_status_id_reg;
    m_axis_write_desc_status_dest_next  = m_axis_write_desc_status_dest_reg;
    m_axis_write_desc_status_user_next  = m_axis_write_desc_status_user_reg;
    m_axis_write_desc_status_error_next = m_axis_write_desc_status_error_reg;
    m_axis_write_desc_status_valid_next = 1'b0;

    s_axis_write_data_tready_next = 1'b0;

    m_axi_awaddr_next  = m_axi_awaddr_reg;
    m_axi_awlen_next   = m_axi_awlen_reg;
    m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready; // Hold AWVALID until AWREADY
    m_axi_wdata_int    = shift_axis_tdata;
    m_axi_wstrb_int    = shift_axis_tkeep;
    m_axi_wlast_int    = 1'b0;
    m_axi_wvalid_int   = 1'b0;
    m_axi_bready_next  = 1'b0;

    transfer_in_save   = 1'b0;
    flush_save         = 1'b0;
    status_fifo_we     = 1'b0;

    cycle_size = AXIS_KEEP_WIDTH_INT;

    addr_next               = addr_reg;
    offset_next             = offset_reg;
    strb_offset_mask_next   = strb_offset_mask_reg;
    zero_offset_next        = zero_offset_reg;
    last_cycle_offset_next  = last_cycle_offset_reg;
    length_next             = length_reg;
    op_word_count_next      = op_word_count_reg;
    tr_word_count_next      = tr_word_count_reg;
    input_cycle_count_next  = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next       = input_active_reg;
    first_cycle_next        = first_cycle_reg;
    input_last_cycle_next   = input_last_cycle_reg;
    output_last_cycle_next  = output_last_cycle_reg;
    last_transfer_next      = last_transfer_reg;

    status_fifo_rd_ptr_next = status_fifo_rd_ptr_reg;

    inc_active = 1'b0;
    dec_active = 1'b0;

    tag_next       = tag_reg;
    axis_id_next   = axis_id_reg;
    axis_dest_next = axis_dest_reg;
    axis_user_next = axis_user_reg;

    status_fifo_wr_len  = length_reg;
    status_fifo_wr_tag  = tag_reg;
    status_fifo_wr_id   = axis_id_reg;
    status_fifo_wr_dest = axis_dest_next;
    status_fifo_wr_user = axis_user_next;
    status_fifo_wr_last = 1'b0;

    // Track write response errors (SLVERR/DECERR) from the memory slave
    if (m_axi_bready && m_axi_bvalid && (m_axi_bresp == AXI_RESP_SLVERR || m_axi_bresp == AXI_RESP_DECERR)) begin
        bresp_next = m_axi_bresp;
    end else begin
        bresp_next = bresp_reg;
    end

    case (state_reg)
        // STATE_IDLE: Awaiting a new descriptor from the host
        STATE_IDLE: begin
            flush_save = 1'b1;
            // Ready to accept descriptor if engine is enabled and status FIFO has credit
            s_axis_write_desc_ready_next = enable && active_count_av_reg;

            if (ENABLE_UNALIGNED) begin
                addr_next              = s_axis_write_desc_addr;
                offset_next            = s_axis_write_desc_addr & OFFSET_MASK;
                strb_offset_mask_next  = {AXI_STRB_WIDTH{1'b1}} << (s_axis_write_desc_addr & OFFSET_MASK);
                zero_offset_next       = (s_axis_write_desc_addr & OFFSET_MASK) == 0;
                last_cycle_offset_next = offset_next + (s_axis_write_desc_len & OFFSET_MASK);
            end else begin
                // In aligned mode, low address bits are masked to word boundary
                addr_next              = s_axis_write_desc_addr & ADDR_MASK;
                offset_next            = 0;
                strb_offset_mask_next  = {AXI_STRB_WIDTH{1'b1}};
                zero_offset_next       = 1'b1;
                last_cycle_offset_next = offset_next + (s_axis_write_desc_len & OFFSET_MASK);
            end
            tag_next           = s_axis_write_desc_tag;
            op_word_count_next = s_axis_write_desc_len;
            first_cycle_next   = 1'b1;
            length_next        = 0;

            if (s_axis_write_desc_ready && s_axis_write_desc_valid) begin
                s_axis_write_desc_ready_next = 1'b0;
                state_next = STATE_START;
            end else begin
                state_next = STATE_IDLE;
            end
        end

        // STATE_START: Calculate burst size, check 4KB boundary, and issue AWVALID
        STATE_START: begin
            // Determine maximum allowable byte transfer for this burst
            if (op_word_count_reg <= AXI_MAX_BURST_SIZE - (addr_reg & OFFSET_MASK) || AXI_MAX_BURST_SIZE >= 4096) begin
                // Remaining length fits within a single AXI burst
                if (((addr_reg & 12'hfff) + (op_word_count_reg & 12'hfff)) >> 12 != 0 || op_word_count_reg >> 12 != 0) begin
                    // Crosses 4KB page boundary: clamp burst length to end exactly at 4KB edge
                    tr_word_count_next = 13'h1000 - (addr_reg & 12'hfff);
                end else begin
                    tr_word_count_next = op_word_count_reg;
                end
            end else begin
                // Transfer exceeds maximum burst size: slice to AXI_MAX_BURST_SIZE
                if (((addr_reg & 12'hfff) + AXI_MAX_BURST_SIZE) >> 12 != 0) begin
                    // Crosses 4KB page boundary before max burst size: clamp at 4KB edge
                    tr_word_count_next = 13'h1000 - (addr_reg & 12'hfff);
                end else begin
                    tr_word_count_next = AXI_MAX_BURST_SIZE - (addr_reg & OFFSET_MASK);
                end
            end

            // Calculate stream beat count and AXI beat count (AWLEN)
            input_cycle_count_next = (tr_word_count_next - 1) >> $clog2(AXIS_KEEP_WIDTH_INT);
            input_last_cycle_next  = input_cycle_count_next == 0;
            if (ENABLE_UNALIGNED) begin
                output_cycle_count_next = (tr_word_count_next + (addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
            end else begin
                output_cycle_count_next = (tr_word_count_next - 1) >> AXI_BURST_SIZE;
            end
            output_last_cycle_next = output_cycle_count_next == 0;
            last_transfer_next     = tr_word_count_next == op_word_count_reg;
            input_active_next      = 1'b1;

            if (ENABLE_UNALIGNED) begin
                if (!first_cycle_reg && last_transfer_next) begin
                    if (offset_reg >= last_cycle_offset_reg && last_cycle_offset_reg > 0) begin
                        // Final beat served entirely by stored partial word from previous cycle
                        input_active_next      = input_cycle_count_next > 0;
                        input_cycle_count_next = input_cycle_count_next - 1;
                    end
                end
            end

            // Issue write address handshake on AW channel
            if (!m_axi_awvalid_reg && active_count_av_reg) begin
                m_axi_awaddr_next  = addr_reg;
                m_axi_awlen_next   = output_cycle_count_next; // AWLEN = number of beats - 1
                m_axi_awvalid_next = s_axis_write_data_tvalid || !first_cycle_reg;

                if (m_axi_awvalid_next) begin
                    addr_next          = addr_reg + tr_word_count_next;
                    op_word_count_next = op_word_count_reg - tr_word_count_next;

                    s_axis_write_data_tready_next = m_axi_wready_int && input_active_next;

                    inc_active = 1'b1; // Reserve credit in status FIFO

                    state_next = STATE_WRITE;
                end else begin
                    state_next = STATE_START;
                end
            end else begin
                state_next = STATE_START;
            end
        end

        // STATE_WRITE: Stream data beats into the W-channel output FIFO
        STATE_WRITE: begin
            s_axis_write_data_tready_next = m_axi_wready_int && (last_transfer_reg || input_active_reg) && shift_axis_input_tready;

            if ((s_axis_write_data_tready && shift_axis_tvalid) || (!input_active_reg && !last_transfer_reg) || !shift_axis_input_tready) begin
                if (s_axis_write_data_tready && s_axis_write_data_tvalid) begin
                    transfer_in_save = 1'b1;

                    axis_id_next   = s_axis_write_data_tid;
                    axis_dest_next = s_axis_write_data_tdest;
                    axis_user_next = s_axis_write_data_tuser;
                end

                // Update byte counters and beat cycle countdowns
                if (first_cycle_reg) begin
                    length_next = length_reg + (AXIS_KEEP_WIDTH_INT - offset_reg);
                end else begin
                    length_next = length_reg + AXIS_KEEP_WIDTH_INT;
                end
                if (input_active_reg) begin
                    input_cycle_count_next = input_cycle_count_reg - 1;
                    input_active_next      = input_cycle_count_reg > 0;
                end
                input_last_cycle_next   = input_cycle_count_next == 0;
                output_cycle_count_next = output_cycle_count_reg - 1;
                output_last_cycle_next  = output_cycle_count_next == 0;
                first_cycle_next        = 1'b0;
                strb_offset_mask_next   = {AXI_STRB_WIDTH{1'b1}};

                m_axi_wdata_int  = shift_axis_tdata;
                m_axi_wstrb_int  = strb_offset_mask_reg;
                m_axi_wvalid_int = 1'b1;

                if (AXIS_LAST_ENABLE && s_axis_write_data_tlast) begin
                    // Stream packet ended
                    input_active_next = 1'b0;
                    s_axis_write_data_tready_next = 1'b0;
                end

                if (AXIS_LAST_ENABLE && shift_axis_tlast) begin
                    // Packet boundary detected on shifted stream datapath
                    if (AXIS_KEEP_ENABLE) begin
                        cycle_size = AXIS_KEEP_WIDTH_INT;
                        for (i = AXIS_KEEP_WIDTH_INT-1; i >= 0; i = i - 1) begin
                            if (~shift_axis_tkeep & strb_offset_mask_reg & (1 << i)) begin
                                cycle_size = i;
                            end
                        end
                    end else begin
                        cycle_size = AXIS_KEEP_WIDTH_INT;
                    end

                    if (output_last_cycle_reg) begin
                        m_axi_wlast_int = 1'b1;

                        // Final beat of burst matches final beat of stream
                        if (last_transfer_reg && last_cycle_offset_reg > 0) begin
                            if (AXIS_KEEP_ENABLE && !(shift_axis_tkeep & ~({AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg)))) begin
                                m_axi_wstrb_int = strb_offset_mask_reg & shift_axis_tkeep;
                                if (first_cycle_reg) begin
                                    length_next = length_reg + (cycle_size - offset_reg);
                                end else begin
                                    length_next = length_reg + cycle_size;
                                end
                            end else begin
                                m_axi_wstrb_int = strb_offset_mask_reg & {AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg);
                                if (first_cycle_reg) begin
                                    length_next = length_reg + (last_cycle_offset_reg - offset_reg);
                                end else begin
                                    length_next = length_reg + last_cycle_offset_reg;
                                end
                            end
                        end else begin
                            if (AXIS_KEEP_ENABLE) begin
                                m_axi_wstrb_int = strb_offset_mask_reg & shift_axis_tkeep;
                                if (first_cycle_reg) begin
                                    length_next = length_reg + (cycle_size - offset_reg);
                                end else begin
                                    length_next = length_reg + cycle_size;
                                end
                            end
                        end

                        // Enqueue completion entry into status FIFO for B-channel retirement
                        status_fifo_we      = 1'b1;
                        status_fifo_wr_len  = length_next;
                        status_fifo_wr_tag  = tag_reg;
                        status_fifo_wr_id   = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b1;

                        s_axis_write_data_tready_next = 1'b0;
                        s_axis_write_desc_ready_next  = enable && active_count_av_reg;
                        state_next                    = STATE_IDLE;
                    end else begin
                        // Stream ended early, but AXI burst still has beats pending.
                        // Must transition to STATE_FINISH_BURST to pad out remaining beats.
                        if (AXIS_KEEP_ENABLE) begin
                            m_axi_wstrb_int = strb_offset_mask_reg & shift_axis_tkeep;
                            if (first_cycle_reg) begin
                                length_next = length_reg + (cycle_size - offset_reg);
                            end else begin
                                length_next = length_reg + cycle_size;
                            end
                        end

                        status_fifo_we      = 1'b1;
                        status_fifo_wr_len  = length_next;
                        status_fifo_wr_tag  = tag_reg;
                        status_fifo_wr_id   = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b1;

                        s_axis_write_data_tready_next = 1'b0;
                        state_next                    = STATE_FINISH_BURST;
                    end

                end else if (output_last_cycle_reg) begin
                    // End of current AXI burst (WLAST asserted)
                    m_axi_wlast_int = 1'b1;

                    if (op_word_count_reg > 0) begin
                        // Descriptor still has remaining bytes: record burst in status FIFO and loop back to START
                        status_fifo_we      = 1'b1;
                        status_fifo_wr_len  = length_next;
                        status_fifo_wr_tag  = tag_reg;
                        status_fifo_wr_id   = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b0; // Not final burst of descriptor

                        s_axis_write_data_tready_next = 1'b0;
                        state_next                    = STATE_START;
                    end else begin
                        // Transfer length satisfied: mask final beat strobes
                        if (last_cycle_offset_reg > 0) begin
                            m_axi_wstrb_int = strb_offset_mask_reg & {AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - last_cycle_offset_reg);
                            if (first_cycle_reg) begin
                                length_next = length_reg + (last_cycle_offset_reg - offset_reg);
                            end else begin
                                length_next = length_reg + last_cycle_offset_reg;
                            end
                        end

                        status_fifo_we      = 1'b1;
                        status_fifo_wr_len  = length_next;
                        status_fifo_wr_tag  = tag_reg;
                        status_fifo_wr_id   = axis_id_next;
                        status_fifo_wr_dest = axis_dest_next;
                        status_fifo_wr_user = axis_user_next;
                        status_fifo_wr_last = 1'b1; // Final burst of descriptor

                        if (AXIS_LAST_ENABLE) begin
                            // If stream has not asserted TLAST yet, discard excess beats
                            s_axis_write_data_tready_next = shift_axis_input_tready;
                            state_next                    = STATE_DROP_DATA;
                        end else begin
                            s_axis_write_data_tready_next = 1'b0;
                            s_axis_write_desc_ready_next  = enable && active_count_av_reg;
                            state_next                    = STATE_IDLE;
                        end
                    end
                end else begin
                    s_axis_write_data_tready_next = m_axi_wready_int && (last_transfer_reg || input_active_next) && shift_axis_input_tready;
                    state_next = STATE_WRITE;
                end
            end else begin
                state_next = STATE_WRITE;
            end
        end

        // STATE_FINISH_BURST: Zero-pad remaining AXI burst beats
        // AXI protocol rule: Slave expects exactly AWLEN + 1 beats for every AW accepted.
        // If stream ended early, we emit dummy beats with WSTRB=0 to safely satisfy the burst.
        STATE_FINISH_BURST: begin
            if (m_axi_wready_int) begin
                if (input_active_reg) begin
                    input_cycle_count_next = input_cycle_count_reg - 1;
                    input_active_next      = input_cycle_count_reg > 0;
                end
                input_last_cycle_next   = input_cycle_count_next == 0;
                output_cycle_count_next = output_cycle_count_reg - 1;
                output_last_cycle_next  = output_cycle_count_next == 0;

                m_axi_wdata_int  = {AXI_DATA_WIDTH{1'b0}};
                m_axi_wstrb_int  = {AXI_STRB_WIDTH{1'b0}}; // Inactive strobes prevent memory corruption
                m_axi_wvalid_int = 1'b1;

                if (output_last_cycle_reg) begin
                    m_axi_wlast_int = 1'b1; // Final beat of dummy padded burst

                    s_axis_write_data_tready_next = 1'b0;
                    s_axis_write_desc_ready_next  = enable && active_count_av_reg;
                    state_next                    = STATE_IDLE;
                end else begin
                    state_next = STATE_FINISH_BURST;
                end
            end else begin
                state_next = STATE_FINISH_BURST;
            end
        end

        // STATE_DROP_DATA: Drain leftover stream beats until TLAST
        // When descriptor length is smaller than the input stream frame, drain remaining beats.
        STATE_DROP_DATA: begin
            s_axis_write_data_tready_next = shift_axis_input_tready;

            if (shift_axis_tvalid) begin
                if (s_axis_write_data_tready && s_axis_write_data_tvalid) begin
                    transfer_in_save = 1'b1;
                end

                if (shift_axis_tlast) begin
                    s_axis_write_data_tready_next = 1'b0;
                    s_axis_write_desc_ready_next  = enable && active_count_av_reg;
                    state_next                    = STATE_IDLE;
                end else begin
                    state_next = STATE_DROP_DATA;
                end
            end else begin
                state_next = STATE_DROP_DATA;
            end
        end
    endcase

    // Write Response (B Channel) Processing & Descriptor Completion
    // When BVALID & BREADY handshake occurs, pop the matching burst from status FIFO.
    if (status_fifo_rd_ptr_reg != status_fifo_wr_ptr_reg) begin
        if (m_axi_bready && m_axi_bvalid) begin
            m_axis_write_desc_status_len_next  = status_fifo_len[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_tag_next  = status_fifo_tag[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_id_next   = status_fifo_id[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_dest_next = status_fifo_dest[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            m_axis_write_desc_status_user_next = status_fifo_user[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];

            // Map AXI BRESP errors to DMA descriptor status error codes
            if (bresp_next == AXI_RESP_SLVERR) begin
                m_axis_write_desc_status_error_next = DMA_ERROR_AXI_WR_SLVERR;
            end else if (bresp_next == AXI_RESP_DECERR) begin
                m_axis_write_desc_status_error_next = DMA_ERROR_AXI_WR_DECERR;
            end else begin
                m_axis_write_desc_status_error_next = DMA_ERROR_NONE;
            end

            // Only pulse status valid when the final burst of a descriptor finishes
            m_axis_write_desc_status_valid_next = status_fifo_last[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]];
            status_fifo_rd_ptr_next             = status_fifo_rd_ptr_reg + 1;
            m_axi_bready_next                   = 1'b0;

            if (status_fifo_last[status_fifo_rd_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]]) begin
                bresp_next = AXI_RESP_OKAY; // Reset error tracker for next descriptor
            end

            dec_active = 1'b1; // Free credit in status FIFO
        end else begin
            // Ready to accept write response
            m_axi_bready_next = 1'b1;
        end
    end
end

// Sequential State & Register Updates
always @(posedge clk) begin
    state_reg <= state_next;

    s_axis_write_desc_ready_reg <= s_axis_write_desc_ready_next;

    m_axis_write_desc_status_len_reg   <= m_axis_write_desc_status_len_next;
    m_axis_write_desc_status_tag_reg   <= m_axis_write_desc_status_tag_next;
    m_axis_write_desc_status_id_reg    <= m_axis_write_desc_status_id_next;
    m_axis_write_desc_status_dest_reg  <= m_axis_write_desc_status_dest_next;
    m_axis_write_desc_status_user_reg  <= m_axis_write_desc_status_user_next;
    m_axis_write_desc_status_error_reg <= m_axis_write_desc_status_error_next;
    m_axis_write_desc_status_valid_reg <= m_axis_write_desc_status_valid_next;

    s_axis_write_data_tready_reg <= s_axis_write_data_tready_next;

    m_axi_awaddr_reg  <= m_axi_awaddr_next;
    m_axi_awlen_reg   <= m_axi_awlen_next;
    m_axi_awvalid_reg <= m_axi_awvalid_next;
    m_axi_bready_reg  <= m_axi_bready_next;

    addr_reg               <= addr_next;
    offset_reg             <= offset_next;
    strb_offset_mask_reg   <= strb_offset_mask_next;
    zero_offset_reg        <= zero_offset_next;
    last_cycle_offset_reg  <= last_cycle_offset_next;
    length_reg             <= length_next;
    op_word_count_reg      <= op_word_count_next;
    tr_word_count_reg      <= tr_word_count_next;
    input_cycle_count_reg  <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg       <= input_active_next;
    first_cycle_reg        <= first_cycle_next;
    input_last_cycle_reg   <= input_last_cycle_next;
    output_last_cycle_reg  <= output_last_cycle_next;
    last_transfer_reg      <= last_transfer_next;
    bresp_reg              <= bresp_next;

    tag_reg       <= tag_next;
    axis_id_reg   <= axis_id_next;
    axis_dest_reg <= axis_dest_next;
    axis_user_reg <= axis_user_next;

    // Alignment Datapath Save Registers
    if (flush_save) begin
        save_axis_tkeep_reg        <= {AXIS_KEEP_WIDTH_INT{1'b0}};
        save_axis_tlast_reg        <= 1'b0;
        shift_axis_extra_cycle_reg <= 1'b0;
    end else if (transfer_in_save) begin
        save_axis_tdata_reg        <= s_axis_write_data_tdata;
        save_axis_tkeep_reg        <= AXIS_KEEP_ENABLE ? s_axis_write_data_tkeep : {AXIS_KEEP_WIDTH_INT{1'b1}};
        save_axis_tlast_reg        <= s_axis_write_data_tlast;
        shift_axis_extra_cycle_reg <= s_axis_write_data_tlast & ((s_axis_write_data_tkeep >> (AXIS_KEEP_WIDTH_INT-offset_reg)) != 0);
    end

    // Status Tracking FIFO write
    if (status_fifo_we) begin
        status_fifo_len[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]]  <= status_fifo_wr_len;
        status_fifo_tag[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]]  <= status_fifo_wr_tag;
        status_fifo_id[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]]   <= status_fifo_wr_id;
        status_fifo_dest[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_dest;
        status_fifo_user[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_user;
        status_fifo_last[status_fifo_wr_ptr_reg[STATUS_FIFO_ADDR_WIDTH-1:0]] <= status_fifo_wr_last;
        status_fifo_wr_ptr_reg <= status_fifo_wr_ptr_reg + 1;
    end
    status_fifo_rd_ptr_reg <= status_fifo_rd_ptr_next;

    // Credit Counter: Prevents issuing more bursts than status FIFO can store
    if (active_count_reg < 2**STATUS_FIFO_ADDR_WIDTH && inc_active && !dec_active) begin
        active_count_reg    <= active_count_reg + 1;
        active_count_av_reg <= active_count_reg < (2**STATUS_FIFO_ADDR_WIDTH-1);
    end else if (active_count_reg > 0 && !inc_active && dec_active) begin
        active_count_reg    <= active_count_reg - 1;
        active_count_av_reg <= 1'b1;
    end else begin
        active_count_av_reg <= active_count_reg < 2**STATUS_FIFO_ADDR_WIDTH;
    end

    // Synchronous Reset
    if (rst) begin
        state_reg <= STATE_IDLE;

        s_axis_write_desc_ready_reg        <= 1'b0;
        m_axis_write_desc_status_valid_reg <= 1'b0;

        s_axis_write_data_tready_reg <= 1'b0;

        m_axi_awvalid_reg <= 1'b0;
        m_axi_bready_reg  <= 1'b0;

        bresp_reg <= AXI_RESP_OKAY;

        save_axis_tlast_reg        <= 1'b0;
        shift_axis_extra_cycle_reg <= 1'b0;

        status_fifo_wr_ptr_reg <= 0;
        status_fifo_rd_ptr_reg <= 0;

        active_count_reg    <= 0;
        active_count_av_reg <= 1'b1;
    end
end

// Output W-Channel Skid Buffer / Distributed RAM FIFO
// Decouples internal FSM pipelining from external AXI WREADY backpressure.
// Throttles internal generation when reaching half-full watermark.
reg [AXI_DATA_WIDTH-1:0] m_axi_wdata_reg  = {AXI_DATA_WIDTH{1'b0}};
reg [AXI_STRB_WIDTH-1:0] m_axi_wstrb_reg  = {AXI_STRB_WIDTH{1'b0}};
reg                      m_axi_wlast_reg  = 1'b0;
reg                      m_axi_wvalid_reg = 1'b0;

reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_wr_ptr_reg = 0;
reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_rd_ptr_reg = 0;
reg out_fifo_half_full_reg = 1'b0;

wire out_fifo_full  = out_fifo_wr_ptr_reg == (out_fifo_rd_ptr_reg ^ {1'b1, {OUTPUT_FIFO_ADDR_WIDTH{1'b0}}});
wire out_fifo_empty = out_fifo_wr_ptr_reg == out_fifo_rd_ptr_reg;

(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXI_DATA_WIDTH-1:0] out_fifo_wdata[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXI_STRB_WIDTH-1:0] out_fifo_wstrb[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg                      out_fifo_wlast[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];

assign m_axi_wready_int = !out_fifo_half_full_reg;

assign m_axi_wdata  = m_axi_wdata_reg;
assign m_axi_wstrb  = m_axi_wstrb_reg;
assign m_axi_wvalid = m_axi_wvalid_reg;
assign m_axi_wlast  = m_axi_wlast_reg;

always @(posedge clk) begin
    m_axi_wvalid_reg <= m_axi_wvalid_reg && !m_axi_wready;

    out_fifo_half_full_reg <= $unsigned(out_fifo_wr_ptr_reg - out_fifo_rd_ptr_reg) >= 2**(OUTPUT_FIFO_ADDR_WIDTH-1);

    // Enqueue beat from internal datapath into FIFO
    if (!out_fifo_full && m_axi_wvalid_int) begin
        out_fifo_wdata[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axi_wdata_int;
        out_fifo_wstrb[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axi_wstrb_int;
        out_fifo_wlast[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axi_wlast_int;
        out_fifo_wr_ptr_reg <= out_fifo_wr_ptr_reg + 1;
    end

    // Dequeue beat from FIFO onto external AXI W channel
    if (!out_fifo_empty && (!m_axi_wvalid_reg || m_axi_wready)) begin
        m_axi_wdata_reg     <= out_fifo_wdata[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axi_wstrb_reg     <= out_fifo_wstrb[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axi_wlast_reg     <= out_fifo_wlast[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axi_wvalid_reg    <= 1'b1;
        out_fifo_rd_ptr_reg <= out_fifo_rd_ptr_reg + 1;
    end

    if (rst) begin
        out_fifo_wr_ptr_reg <= 0;
        out_fifo_rd_ptr_reg <= 0;
        m_axi_wvalid_reg    <= 1'b0;
    end
end

endmodule

`resetall
