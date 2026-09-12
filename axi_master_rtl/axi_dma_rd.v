// Author:       Akshansh Chaurasia
// Supervisor:   Dr. Rakesh Palisetty
// Institution:  Shiv Nadar University
// Department:   Department of Electrical Engineering
// Project:      Minor Project
// Description:  AXI4 DMA Read Engine
//               Reads data from an AXI4 Memory-Mapped master interface based on
//               descriptor requests and packages it into an AXI4-Stream master output.
//
// Verification Overview:
// - Architecture (Split-Transaction Dual FSM):
//     AXI read transactions decouple address requests from data responses.
//     This core reflects that with two independent state machines:
//     1. axi_state FSM (Read Address Generator):
//        * Accepts read descriptors from s_axis_read_desc_*.
//        * Slices large transfers into bursts <= AXI_MAX_BURST_LEN.
//        * Enforces ARM AXI4 4KB page boundary rule (A3.4.1), clipping bursts at 4KB edges.
//        * Generates AR-channel requests (ARADDR, ARLEN, ARVALID) and pushes transfer
//          parameters into the internal axis_cmd_* pipeline.
//     2. axis_state FSM (Read Data & Stream Output Engine):
//        * Receives transfer metadata from the axis_cmd_* pipeline.
//        * Consumes data beats from the AXI R channel (RDATA, RRESP, RLAST).
//        * Performs byte alignment barrel shifting for unaligned transfers.
//        * Formats beats into AXI-Stream (TDATA, TKEEP, TLAST).
//        * Monitors RRESP for SLVERR/DECERR errors and issues a completion pulse
//          on m_axis_read_desc_status_* upon the final beat.
// - Protocol Handshakes:
//     * Descriptor Input (s_axis_read_desc_*): Valid/Ready handshake.
//     * Read Status Output (m_axis_read_desc_status_*): 1-cycle valid pulse at packet completion.
//     * AXI4 AR & R Channels: Issues INCR bursts on AR; consumes RDATA with RREADY backpressure.
//     * Stream Data Master (m_axis_read_data_*): Driven via a 32-entry skid FIFO buffer.
//
// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

module axi_dma_rd #
(
    // Width of AXI memory-mapped data bus in bits (e.g. 32, 64, 128)
    parameter AXI_DATA_WIDTH = 32,
    // Width of AXI address bus in bits
    parameter AXI_ADDR_WIDTH = 16,
    // Byte-lane strobe width (1 bit per byte of AXI data)
    parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
    // Width of AXI transaction ID tag (ARID and RID)
    parameter AXI_ID_WIDTH = 8,
    // Maximum burst length in beats per generated AXI transaction (1 to 256)
    parameter AXI_MAX_BURST_LEN = 16,
    // Width of AXI-Stream master data bus in bits (must match AXI_DATA_WIDTH)
    parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH,
    // Enable TKEEP byte qualifiers on the stream interface
    parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
    // Stream TKEEP width (1 bit per byte)
    parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
    // Enable TLAST packet boundary generation on stream interface
    parameter AXIS_LAST_ENABLE = 1,
    // Propagate descriptor ID to stream TID
    parameter AXIS_ID_ENABLE = 0,
    parameter AXIS_ID_WIDTH = 8,
    // Propagate descriptor destination tag to stream TDEST
    parameter AXIS_DEST_ENABLE = 0,
    parameter AXIS_DEST_WIDTH = 8,
    // Propagate descriptor user metadata to stream TUSER
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

    // Read Descriptor Input Interface
    // Handshake: Valid/Ready. Latched when s_axis_read_desc_valid & s_axis_read_desc_ready.
    input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_read_desc_addr,   // Source address in AXI memory space
    input  wire [LEN_WIDTH-1:0]       s_axis_read_desc_len,    // Total transfer length in bytes
    input  wire [TAG_WIDTH-1:0]       s_axis_read_desc_tag,    // User tag reflected in completion status
    input  wire [AXIS_ID_WIDTH-1:0]   s_axis_read_desc_id,     // TID to attach to stream packets
    input  wire [AXIS_DEST_WIDTH-1:0] s_axis_read_desc_dest,   // TDEST to attach to stream packets
    input  wire [AXIS_USER_WIDTH-1:0] s_axis_read_desc_user,   // TUSER to attach to stream packets
    input  wire                       s_axis_read_desc_valid,  // Descriptor valid indicator
    output wire                       s_axis_read_desc_ready,  // Ready to accept new read descriptor

    // Read Descriptor Completion Status Output Interface
    // Pulses valid for one cycle upon retirement of the final stream beat of a descriptor.
    output wire [TAG_WIDTH-1:0]       m_axis_read_desc_status_tag,   // Tag of finished descriptor
    output wire [3:0]                 m_axis_read_desc_status_error, // Error status: 0=OK, 4=SLVERR, 5=DECERR
    output wire                       m_axis_read_desc_status_valid, // Status valid pulse

    // AXI-Stream Master Read Data Output Interface
    // Streams data read from memory to the downstream destination.
    output wire [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata,  // Read data payload
    output wire [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep,  // Byte qualifies (valid byte mask)
    output wire                       m_axis_read_data_tvalid, // Stream data valid
    input  wire                       m_axis_read_data_tready, // Downstream ready (backpressure)
    output wire                       m_axis_read_data_tlast,  // End of packet marker
    output wire [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid,    // Stream source ID
    output wire [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest,  // Stream destination tag
    output wire [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser,  // User sideband metadata

    // AXI4 Master Read Address Channel (AR)
    output wire [AXI_ID_WIDTH-1:0]    m_axi_arid,    // Read Address ID (fixed to 0)
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,  // Burst start address
    output wire [7:0]                 m_axi_arlen,   // Burst length: beat count minus 1 (0 = 1 beat)
    output wire [2:0]                 m_axi_arsize,  // Beat size: log2(bytes per beat), e.g. 3'b010 = 4B
    output wire [1:0]                 m_axi_arburst, // Burst type: 2'b01 = INCR (incremental addressing)
    output wire                       m_axi_arvalid, // Read address valid
    input  wire                       m_axi_arready, // Slave read address ready

    // AXI4 Master Read Data Channel (R)
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid,     // Read response ID (matches ARID)
    input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata,   // Read data payload from memory
    input  wire [1:0]                 m_axi_rresp,   // Read response: 2'b00=OKAY, 2'b10=SLVERR, 2'b11=DECERR
    input  wire                       m_axi_rlast,   // Last beat indicator for active AXI burst
    input  wire                       m_axi_rvalid,  // Read data valid from memory slave
    output wire                       m_axi_rready,  // Master ready to accept read beat

    // Operational Control
    input  wire                       enable         // Master enable (gates descriptor acceptance)
);

// Bus width and sizing parameters
parameter AXI_WORD_WIDTH = AXI_STRB_WIDTH;
parameter AXI_WORD_SIZE = AXI_DATA_WIDTH/AXI_WORD_WIDTH; // Width of a single byte/word lane (usually 8)
parameter AXI_BURST_SIZE = $clog2(AXI_STRB_WIDTH);       // Encoded transfer size (ARSIZE = log2(STRB_WIDTH))
parameter AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN << AXI_BURST_SIZE; // Max byte capacity per burst

parameter AXIS_KEEP_WIDTH_INT = AXIS_KEEP_ENABLE ? AXIS_KEEP_WIDTH : 1;
parameter AXIS_WORD_WIDTH = AXIS_KEEP_WIDTH_INT;
parameter AXIS_WORD_SIZE = AXIS_DATA_WIDTH/AXIS_WORD_WIDTH;

// Offset masks for isolating byte-level alignment within a word
parameter OFFSET_WIDTH = AXI_STRB_WIDTH > 1 ? $clog2(AXI_STRB_WIDTH) : 1;
parameter OFFSET_MASK = AXI_STRB_WIDTH > 1 ? {OFFSET_WIDTH{1'b1}} : 0;
parameter ADDR_MASK = {AXI_ADDR_WIDTH{1'b1}} << $clog2(AXI_STRB_WIDTH); // Truncates unaligned low address bits
parameter CYCLE_COUNT_WIDTH = LEN_WIDTH - AXI_BURST_SIZE + 1;

// Output FIFO buffer depth parameter
parameter OUTPUT_FIFO_ADDR_WIDTH = 5; // 32-entry skid buffer on AXIS output to decouple from stalls

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

// AXI4 Response Status Encodings (RRESP)
localparam [1:0]
    AXI_RESP_OKAY   = 2'b00, // Normal access success
    AXI_RESP_EXOKAY = 2'b01, // Exclusive access success
    AXI_RESP_SLVERR = 2'b10, // Slave error (target memory failed access)
    AXI_RESP_DECERR = 2'b11; // Decode error (no slave mapped at target address)

// Status Error Codes reported on m_axis_read_desc_status_error
localparam [3:0]
    DMA_ERROR_NONE              = 4'd0, // Successful completion
    DMA_ERROR_TIMEOUT           = 4'd1,
    DMA_ERROR_PARITY            = 4'd2,
    DMA_ERROR_AXI_RD_SLVERR     = 4'd4, // Slave returned SLVERR on R channel
    DMA_ERROR_AXI_RD_DECERR     = 4'd5, // Interconnect returned DECERR on R channel
    DMA_ERROR_AXI_WR_SLVERR     = 4'd6,
    DMA_ERROR_AXI_WR_DECERR     = 4'd7,
    DMA_ERROR_PCIE_FLR          = 4'd8,
    DMA_ERROR_PCIE_CPL_POISONED = 4'd9,
    DMA_ERROR_PCIE_CPL_STATUS_UR = 4'd10,
    DMA_ERROR_PCIE_CPL_STATUS_CA = 4'd11;

// FSM 1: AXI Read Address (AR) Generator States
localparam [0:0]
    AXI_STATE_IDLE  = 1'd0, // Awaiting read descriptor, computing burst sizing
    AXI_STATE_START = 1'd1; // Issuing AXI AR bursts (enforces 4KB boundary)

reg [0:0] axi_state_reg = AXI_STATE_IDLE, axi_state_next;

// FSM 2: AXI-Stream Read Data (R) Assembler States
localparam [0:0]
    AXIS_STATE_IDLE = 1'd0, // Awaiting transfer parameters from axi_state
    AXIS_STATE_READ = 1'd1; // Receiving R beats, shifting, and streaming to AXIS

reg [0:0] axis_state_reg = AXIS_STATE_IDLE, axis_state_next;

// Datapath control strobes
reg transfer_in_save;
reg axis_cmd_ready;

// Address and transfer progress registers for AR generation
reg [AXI_ADDR_WIDTH-1:0] addr_reg = {AXI_ADDR_WIDTH{1'b0}}, addr_next;         // Active burst address
reg [LEN_WIDTH-1:0] op_word_count_reg = {LEN_WIDTH{1'b0}}, op_word_count_next; // Total bytes remaining
reg [LEN_WIDTH-1:0] tr_word_count_reg = {LEN_WIDTH{1'b0}}, tr_word_count_next; // Bytes in current burst

// Internal Command Pipeline: Passes transfer parameters from axi_state to axis_state
reg [OFFSET_WIDTH-1:0] axis_cmd_offset_reg = {OFFSET_WIDTH{1'b0}}, axis_cmd_offset_next;
reg [OFFSET_WIDTH-1:0] axis_cmd_last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, axis_cmd_last_cycle_offset_next;
reg [CYCLE_COUNT_WIDTH-1:0] axis_cmd_input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, axis_cmd_input_cycle_count_next;
reg [CYCLE_COUNT_WIDTH-1:0] axis_cmd_output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, axis_cmd_output_cycle_count_next;
reg axis_cmd_bubble_cycle_reg = 1'b0, axis_cmd_bubble_cycle_next; // Initial alignment delay cycle
reg [TAG_WIDTH-1:0] axis_cmd_tag_reg = {TAG_WIDTH{1'b0}}, axis_cmd_tag_next;
reg [AXIS_ID_WIDTH-1:0] axis_cmd_axis_id_reg = {AXIS_ID_WIDTH{1'b0}}, axis_cmd_axis_id_next;
reg [AXIS_DEST_WIDTH-1:0] axis_cmd_axis_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, axis_cmd_axis_dest_next;
reg [AXIS_USER_WIDTH-1:0] axis_cmd_axis_user_reg = {AXIS_USER_WIDTH{1'b0}}, axis_cmd_axis_user_next;
reg axis_cmd_valid_reg = 1'b0, axis_cmd_valid_next; // Handshake: valid to axis_state

// Stream generation tracking registers
reg [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next;
reg [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next;
reg [CYCLE_COUNT_WIDTH-1:0] input_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, input_cycle_count_next;   // AXI R beats countdown
reg [CYCLE_COUNT_WIDTH-1:0] output_cycle_count_reg = {CYCLE_COUNT_WIDTH{1'b0}}, output_cycle_count_next; // AXIS stream beats countdown
reg input_active_reg = 1'b0, input_active_next;
reg output_active_reg = 1'b0, output_active_next;
reg bubble_cycle_reg = 1'b0, bubble_cycle_next;           // Discards first cycle output during unaligned start
reg first_cycle_reg = 1'b0, first_cycle_next;
reg output_last_cycle_reg = 1'b0, output_last_cycle_next; // Triggers TLAST and completion status
reg [1:0] rresp_reg = AXI_RESP_OKAY, rresp_next;          // Latches RRESP error status

// Sideband metadata storage
reg [TAG_WIDTH-1:0] tag_reg = {TAG_WIDTH{1'b0}}, tag_next;
reg [AXIS_ID_WIDTH-1:0] axis_id_reg = {AXIS_ID_WIDTH{1'b0}}, axis_id_next;
reg [AXIS_DEST_WIDTH-1:0] axis_dest_reg = {AXIS_DEST_WIDTH{1'b0}}, axis_dest_next;
reg [AXIS_USER_WIDTH-1:0] axis_user_reg = {AXIS_USER_WIDTH{1'b0}}, axis_user_next;

// Port registered outputs
reg s_axis_read_desc_ready_reg = 1'b0, s_axis_read_desc_ready_next;

reg [TAG_WIDTH-1:0] m_axis_read_desc_status_tag_reg = {TAG_WIDTH{1'b0}}, m_axis_read_desc_status_tag_next;
reg [3:0] m_axis_read_desc_status_error_reg = 4'd0, m_axis_read_desc_status_error_next;
reg m_axis_read_desc_status_valid_reg = 1'b0, m_axis_read_desc_status_valid_next;

reg [AXI_ADDR_WIDTH-1:0] m_axi_araddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_araddr_next;
reg [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next;
reg m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next;
reg m_axi_rready_reg = 1'b0, m_axi_rready_next;

// Alignment Barrel-Shifter Datapath
// Captures previous cycle's read data in save_axi_rdata_reg and shifts it together
// with the incoming beat to align requested unaligned bytes to stream beat byte lanes.
reg [AXI_DATA_WIDTH-1:0] save_axi_rdata_reg = {AXI_DATA_WIDTH{1'b0}};

wire [AXI_DATA_WIDTH-1:0] shift_axi_rdata = {m_axi_rdata, save_axi_rdata_reg} >> ((AXI_STRB_WIDTH-offset_reg)*AXI_WORD_SIZE);

// Internal stream datapath wires driving the output skid FIFO
reg  [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata_int;
reg  [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep_int;
reg                        m_axis_read_data_tvalid_int;
wire                       m_axis_read_data_tready_int;
reg                        m_axis_read_data_tlast_int;
reg  [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid_int;
reg  [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest_int;
reg  [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser_int;

// Port assignments
assign s_axis_read_desc_ready = s_axis_read_desc_ready_reg;

assign m_axis_read_desc_status_tag   = m_axis_read_desc_status_tag_reg;
assign m_axis_read_desc_status_error = m_axis_read_desc_status_error_reg;
assign m_axis_read_desc_status_valid = m_axis_read_desc_status_valid_reg;

assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_araddr  = m_axi_araddr_reg;
assign m_axi_arlen   = m_axi_arlen_reg;
assign m_axi_arsize  = AXI_BURST_SIZE;
assign m_axi_arburst = 2'b01; // INCR burst type
assign m_axi_arvalid = m_axi_arvalid_reg;
assign m_axi_rready  = m_axi_rready_reg;

// FSM 1: AXI Read Address Channel (AR) Generator
// Slices the descriptor into legal AXI read bursts and issues AR handshakes.
always @* begin
    axi_state_next = AXI_STATE_IDLE;

    s_axis_read_desc_ready_next = 1'b0;

    m_axi_araddr_next  = m_axi_araddr_reg;
    m_axi_arlen_next   = m_axi_arlen_reg;
    m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready; // Hold ARVALID until ARREADY

    addr_next          = addr_reg;
    op_word_count_next = op_word_count_reg;
    tr_word_count_next = tr_word_count_reg;

    axis_cmd_offset_next             = axis_cmd_offset_reg;
    axis_cmd_last_cycle_offset_next  = axis_cmd_last_cycle_offset_reg;
    axis_cmd_input_cycle_count_next  = axis_cmd_input_cycle_count_reg;
    axis_cmd_output_cycle_count_next = axis_cmd_output_cycle_count_reg;
    axis_cmd_bubble_cycle_next       = axis_cmd_bubble_cycle_reg;
    axis_cmd_tag_next                = axis_cmd_tag_reg;
    axis_cmd_axis_id_next            = axis_cmd_axis_id_reg;
    axis_cmd_axis_dest_next          = axis_cmd_axis_dest_reg;
    axis_cmd_axis_user_next          = axis_cmd_axis_user_reg;
    axis_cmd_valid_next              = axis_cmd_valid_reg && !axis_cmd_ready;

    case (axi_state_reg)
        // AXI_STATE_IDLE: Await new descriptor and available command pipeline slot
        AXI_STATE_IDLE: begin
            s_axis_read_desc_ready_next = !axis_cmd_valid_reg && enable;

            if (s_axis_read_desc_ready && s_axis_read_desc_valid) begin
                if (ENABLE_UNALIGNED) begin
                    addr_next                       = s_axis_read_desc_addr;
                    axis_cmd_offset_next            = AXI_STRB_WIDTH > 1 ? AXI_STRB_WIDTH - (s_axis_read_desc_addr & OFFSET_MASK) : 0;
                    axis_cmd_bubble_cycle_next      = axis_cmd_offset_next > 0;
                    axis_cmd_last_cycle_offset_next = s_axis_read_desc_len & OFFSET_MASK;
                end else begin
                    // Aligned mode: mask address to word boundary
                    addr_next                       = s_axis_read_desc_addr & ADDR_MASK;
                    axis_cmd_offset_next            = 0;
                    axis_cmd_bubble_cycle_next      = 1'b0;
                    axis_cmd_last_cycle_offset_next = s_axis_read_desc_len & OFFSET_MASK;
                end
                axis_cmd_tag_next  = s_axis_read_desc_tag;
                op_word_count_next = s_axis_read_desc_len;

                axis_cmd_axis_id_next   = s_axis_read_desc_id;
                axis_cmd_axis_dest_next = s_axis_read_desc_dest;
                axis_cmd_axis_user_next = s_axis_read_desc_user;

                // Compute total input beats (AXI R beats) and output beats (AXIS beats)
                if (ENABLE_UNALIGNED) begin
                    axis_cmd_input_cycle_count_next = (op_word_count_next + (s_axis_read_desc_addr & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                end else begin
                    axis_cmd_input_cycle_count_next = (op_word_count_next - 1) >> AXI_BURST_SIZE;
                end
                axis_cmd_output_cycle_count_next = (op_word_count_next - 1) >> AXI_BURST_SIZE;

                axis_cmd_valid_next = 1'b1; // Send command parameters to stream data FSM

                s_axis_read_desc_ready_next = 1'b0;
                axi_state_next = AXI_STATE_START;
            end else begin
                axi_state_next = AXI_STATE_IDLE;
            end
        end

        // AXI_STATE_START: Calculate burst size, check 4KB boundary, and assert ARVALID
        AXI_STATE_START: begin
            if (!m_axi_arvalid) begin
                if (op_word_count_reg <= AXI_MAX_BURST_SIZE - (addr_reg & OFFSET_MASK) || AXI_MAX_BURST_SIZE >= 4096) begin
                    // Transfer fits in single burst
                    if (((addr_reg & 12'hfff) + (op_word_count_reg & 12'hfff)) >> 12 != 0 || op_word_count_reg >> 12 != 0) begin
                        // 4KB page boundary crossed: clamp burst to end at 4KB edge
                        tr_word_count_next = 13'h1000 - (addr_reg & 12'hfff);
                    end else begin
                        tr_word_count_next = op_word_count_reg;
                    end
                end else begin
                    // Transfer requires multiple bursts: clamp at AXI_MAX_BURST_SIZE or 4KB edge
                    if (((addr_reg & 12'hfff) + AXI_MAX_BURST_SIZE) >> 12 != 0) begin
                        tr_word_count_next = 13'h1000 - (addr_reg & 12'hfff);
                    end else begin
                        tr_word_count_next = AXI_MAX_BURST_SIZE - (addr_reg & OFFSET_MASK);
                    end
                end

                m_axi_araddr_next = addr_reg;
                if (ENABLE_UNALIGNED) begin
                    m_axi_arlen_next = (tr_word_count_next + (addr_reg & OFFSET_MASK) - 1) >> AXI_BURST_SIZE;
                end else begin
                    m_axi_arlen_next = (tr_word_count_next - 1) >> AXI_BURST_SIZE;
                end
                m_axi_arvalid_next = 1'b1; // Issue AR read address request

                addr_next          = addr_reg + tr_word_count_next;
                op_word_count_next = op_word_count_reg - tr_word_count_next;

                if (op_word_count_next > 0) begin
                    // Additional bursts required for this descriptor
                    axi_state_next = AXI_STATE_START;
                end else begin
                    // All bursts for this descriptor issued; return to IDLE
                    s_axis_read_desc_ready_next = !axis_cmd_valid_reg && enable;
                    axi_state_next = AXI_STATE_IDLE;
                end
            end else begin
                axi_state_next = AXI_STATE_START;
            end
        end
    endcase
end

// FSM 2: AXI-Stream Read Data (R) Assembler
// Consumes beats from R channel, performs alignment barrel shifting, and drives AXIS.
always @* begin
    axis_state_next = AXIS_STATE_IDLE;

    m_axis_read_desc_status_tag_next   = m_axis_read_desc_status_tag_reg;
    m_axis_read_desc_status_error_next = m_axis_read_desc_status_error_reg;
    m_axis_read_desc_status_valid_next = 1'b0;

    m_axis_read_data_tdata_int  = shift_axi_rdata;
    m_axis_read_data_tkeep_int  = {AXIS_KEEP_WIDTH{1'b1}};
    m_axis_read_data_tlast_int  = 1'b0;
    m_axis_read_data_tvalid_int = 1'b0;
    m_axis_read_data_tid_int    = axis_id_reg;
    m_axis_read_data_tdest_int  = axis_dest_reg;
    m_axis_read_data_tuser_int  = axis_user_reg;

    m_axi_rready_next = 1'b0;

    transfer_in_save = 1'b0;
    axis_cmd_ready   = 1'b0;

    offset_next             = offset_reg;
    last_cycle_offset_next  = last_cycle_offset_reg;
    input_cycle_count_next  = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next       = input_active_reg;
    output_active_next      = output_active_reg;
    bubble_cycle_next       = bubble_cycle_reg;
    first_cycle_next        = first_cycle_reg;
    output_last_cycle_next  = output_last_cycle_reg;

    tag_next       = tag_reg;
    axis_id_next   = axis_id_reg;
    axis_dest_next = axis_dest_reg;
    axis_user_next = axis_user_reg;

    // Latch read errors (SLVERR/DECERR) returned by slave on RRESP
    if (m_axi_rready && m_axi_rvalid && (m_axi_rresp == AXI_RESP_SLVERR || m_axi_rresp == AXI_RESP_DECERR)) begin
        rresp_next = m_axi_rresp;
    end else begin
        rresp_next = rresp_reg;
    end

    case (axis_state_reg)
        // AXIS_STATE_IDLE: Await transfer command parameters from axi_state
        AXIS_STATE_IDLE: begin
            m_axi_rready_next = 1'b0;

            if (ENABLE_UNALIGNED) begin
                offset_next = axis_cmd_offset_reg;
            end else begin
                offset_next = 0;
            end
            last_cycle_offset_next  = axis_cmd_last_cycle_offset_reg;
            input_cycle_count_next  = axis_cmd_input_cycle_count_reg;
            output_cycle_count_next = axis_cmd_output_cycle_count_reg;
            bubble_cycle_next       = axis_cmd_bubble_cycle_reg;
            tag_next                = axis_cmd_tag_reg;
            axis_id_next            = axis_cmd_axis_id_reg;
            axis_dest_next          = axis_cmd_axis_dest_reg;
            axis_user_next          = axis_cmd_axis_user_reg;

            output_last_cycle_next = output_cycle_count_next == 0;
            input_active_next      = 1'b1;
            output_active_next     = 1'b1;
            first_cycle_next       = 1'b1;

            if (axis_cmd_valid_reg) begin
                axis_cmd_ready    = 1'b1; // Acknowledge command from axi_state
                m_axi_rready_next = m_axis_read_data_tready_int;
                axis_state_next   = AXIS_STATE_READ;
            end
        end

        // AXIS_STATE_READ: Collect AXI R beats, shift, and package into stream beats
        AXIS_STATE_READ: begin
            m_axi_rready_next = m_axis_read_data_tready_int && input_active_reg;

            if ((m_axi_rready && m_axi_rvalid) || !input_active_reg) begin
                transfer_in_save = m_axi_rready && m_axi_rvalid;

                // Handle first cycle bubble during unaligned transfers:
                // Pre-fills save_axi_rdata_reg with first memory word without emitting an AXIS beat.
                if (ENABLE_UNALIGNED && first_cycle_reg && bubble_cycle_reg) begin
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next      = input_cycle_count_reg > 0;
                    end
                    bubble_cycle_next = 1'b0;
                    first_cycle_next  = 1'b0;

                    m_axi_rready_next = m_axis_read_data_tready_int && input_active_next;
                    axis_state_next   = AXIS_STATE_READ;
                end else begin
                    // Decrement remaining beat countdowns
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next      = input_cycle_count_reg > 0;
                    end
                    if (output_active_reg) begin
                        output_cycle_count_next = output_cycle_count_reg - 1;
                        output_active_next      = output_cycle_count_reg > 0;
                    end
                    output_last_cycle_next = output_cycle_count_next == 0;
                    bubble_cycle_next      = 1'b0;
                    first_cycle_next       = 1'b0;

                    // Output aligned read data beat to the internal stream datapath
                    m_axis_read_data_tdata_int  = shift_axi_rdata;
                    m_axis_read_data_tkeep_int  = {AXIS_KEEP_WIDTH_INT{1'b1}};
                    m_axis_read_data_tvalid_int = 1'b1;

                    if (output_last_cycle_reg) begin
                        // Final stream beat of the descriptor: apply TKEEP byte mask & assert TLAST
                        if (last_cycle_offset_reg > 0) begin
                            m_axis_read_data_tkeep_int = {AXIS_KEEP_WIDTH_INT{1'b1}} >> (AXIS_KEEP_WIDTH_INT - last_cycle_offset_reg);
                        end
                        m_axis_read_data_tlast_int = 1'b1;

                        // Emit completion status on m_axis_read_desc_status_*
                        m_axis_read_desc_status_tag_next = tag_reg;
                        if (rresp_next == AXI_RESP_SLVERR) begin
                            m_axis_read_desc_status_error_next = DMA_ERROR_AXI_RD_SLVERR;
                        end else if (rresp_next == AXI_RESP_DECERR) begin
                            m_axis_read_desc_status_error_next = DMA_ERROR_AXI_RD_DECERR;
                        end else begin
                            m_axis_read_desc_status_error_next = DMA_ERROR_NONE;
                        end
                        m_axis_read_desc_status_valid_next = 1'b1;

                        rresp_next        = AXI_RESP_OKAY; // Reset error status
                        m_axi_rready_next = 1'b0;
                        axis_state_next   = AXIS_STATE_IDLE;
                    end else begin
                        // Transfer continues
                        m_axi_rready_next = m_axis_read_data_tready_int && input_active_next;
                        axis_state_next   = AXIS_STATE_READ;
                    end
                end
            end else begin
                axis_state_next = AXIS_STATE_READ;
            end
        end
    endcase
end

// Sequential State & Register Updates
always @(posedge clk) begin
    axi_state_reg  <= axi_state_next;
    axis_state_reg <= axis_state_next;

    s_axis_read_desc_ready_reg <= s_axis_read_desc_ready_next;

    m_axis_read_desc_status_tag_reg   <= m_axis_read_desc_status_tag_next;
    m_axis_read_desc_status_error_reg <= m_axis_read_desc_status_error_next;
    m_axis_read_desc_status_valid_reg <= m_axis_read_desc_status_valid_next;

    m_axi_araddr_reg  <= m_axi_araddr_next;
    m_axi_arlen_reg   <= m_axi_arlen_next;
    m_axi_arvalid_reg <= m_axi_arvalid_next;
    m_axi_rready_reg  <= m_axi_rready_next;

    addr_reg           <= addr_next;
    op_word_count_reg  <= op_word_count_next;
    tr_word_count_reg  <= tr_word_count_next;

    axis_cmd_offset_reg             <= axis_cmd_offset_next;
    axis_cmd_last_cycle_offset_reg  <= axis_cmd_last_cycle_offset_next;
    axis_cmd_input_cycle_count_reg  <= axis_cmd_input_cycle_count_next;
    axis_cmd_output_cycle_count_reg <= axis_cmd_output_cycle_count_next;
    axis_cmd_bubble_cycle_reg       <= axis_cmd_bubble_cycle_next;
    axis_cmd_tag_reg                <= axis_cmd_tag_next;
    axis_cmd_axis_id_reg            <= axis_cmd_axis_id_next;
    axis_cmd_axis_dest_reg          <= axis_cmd_axis_dest_next;
    axis_cmd_axis_user_reg          <= axis_cmd_axis_user_next;
    axis_cmd_valid_reg              <= axis_cmd_valid_next;

    offset_reg             <= offset_next;
    last_cycle_offset_reg  <= last_cycle_offset_next;
    input_cycle_count_reg  <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg       <= input_active_next;
    output_active_reg      <= output_active_next;
    bubble_cycle_reg       <= bubble_cycle_next;
    first_cycle_reg        <= first_cycle_next;
    output_last_cycle_reg  <= output_last_cycle_next;
    rresp_reg              <= rresp_next;

    tag_reg       <= tag_next;
    axis_id_reg   <= axis_id_next;
    axis_dest_reg <= axis_dest_next;
    axis_user_reg <= axis_user_next;

    // Alignment Shifter Save Register: stores last read word for next cycle shifting
    if (transfer_in_save) begin
        save_axi_rdata_reg <= m_axi_rdata;
    end

    // Synchronous Reset
    if (rst) begin
        axi_state_reg  <= AXI_STATE_IDLE;
        axis_state_reg <= AXIS_STATE_IDLE;

        axis_cmd_valid_reg <= 1'b0;

        s_axis_read_desc_ready_reg <= 1'b0;

        m_axis_read_desc_status_valid_reg <= 1'b0;
        m_axi_arvalid_reg                 <= 1'b0;
        m_axi_rready_reg                  <= 1'b0;

        rresp_reg <= AXI_RESP_OKAY;
    end
end

// Output Stream Skid Buffer / Distributed RAM FIFO
// Decouples read datapath from downstream m_axis_read_data_tready backpressure.
// Generates internal backpressure (m_axis_read_data_tready_int) at half-full threshold.
reg [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata_reg  = {AXIS_DATA_WIDTH{1'b0}};
reg [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep_reg  = {AXIS_KEEP_WIDTH{1'b0}};
reg                       m_axis_read_data_tvalid_reg = 1'b0;
reg                       m_axis_read_data_tlast_reg  = 1'b0;
reg [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid_reg    = {AXIS_ID_WIDTH{1'b0}};
reg [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest_reg  = {AXIS_DEST_WIDTH{1'b0}};
reg [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser_reg  = {AXIS_USER_WIDTH{1'b0}};

reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_wr_ptr_reg = 0;
reg [OUTPUT_FIFO_ADDR_WIDTH+1-1:0] out_fifo_rd_ptr_reg = 0;
reg out_fifo_half_full_reg = 1'b0;

wire out_fifo_full  = out_fifo_wr_ptr_reg == (out_fifo_rd_ptr_reg ^ {1'b1, {OUTPUT_FIFO_ADDR_WIDTH{1'b0}}});
wire out_fifo_empty = out_fifo_wr_ptr_reg == out_fifo_rd_ptr_reg;

(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_DATA_WIDTH-1:0] out_fifo_tdata[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_KEEP_WIDTH-1:0] out_fifo_tkeep[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg                       out_fifo_tlast[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_ID_WIDTH-1:0]   out_fifo_tid[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_DEST_WIDTH-1:0] out_fifo_tdest[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];
(* ram_style = "distributed", ramstyle = "no_rw_check, mlab" *)
reg [AXIS_USER_WIDTH-1:0] out_fifo_tuser[2**OUTPUT_FIFO_ADDR_WIDTH-1:0];

assign m_axis_read_data_tready_int = !out_fifo_half_full_reg;

assign m_axis_read_data_tdata  = m_axis_read_data_tdata_reg;
assign m_axis_read_data_tkeep  = AXIS_KEEP_ENABLE ? m_axis_read_data_tkeep_reg : {AXIS_KEEP_WIDTH{1'b1}};
assign m_axis_read_data_tvalid = m_axis_read_data_tvalid_reg;
assign m_axis_read_data_tlast  = AXIS_LAST_ENABLE ? m_axis_read_data_tlast_reg : 1'b1;
assign m_axis_read_data_tid    = AXIS_ID_ENABLE   ? m_axis_read_data_tid_reg   : {AXIS_ID_WIDTH{1'b0}};
assign m_axis_read_data_tdest  = AXIS_DEST_ENABLE ? m_axis_read_data_tdest_reg : {AXIS_DEST_WIDTH{1'b0}};
assign m_axis_read_data_tuser  = AXIS_USER_ENABLE ? m_axis_read_data_tuser_reg : {AXIS_USER_WIDTH{1'b0}};

always @(posedge clk) begin
    m_axis_read_data_tvalid_reg <= m_axis_read_data_tvalid_reg && !m_axis_read_data_tready;

    out_fifo_half_full_reg <= $unsigned(out_fifo_wr_ptr_reg - out_fifo_rd_ptr_reg) >= 2**(OUTPUT_FIFO_ADDR_WIDTH-1);

    // Enqueue beat from internal stream datapath into FIFO
    if (!out_fifo_full && m_axis_read_data_tvalid_int) begin
        out_fifo_tdata[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tdata_int;
        out_fifo_tkeep[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tkeep_int;
        out_fifo_tlast[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tlast_int;
        out_fifo_tid[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]]   <= m_axis_read_data_tid_int;
        out_fifo_tdest[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tdest_int;
        out_fifo_tuser[out_fifo_wr_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]] <= m_axis_read_data_tuser_int;
        out_fifo_wr_ptr_reg <= out_fifo_wr_ptr_reg + 1;
    end

    // Dequeue beat from FIFO onto external AXI-Stream interface
    if (!out_fifo_empty && (!m_axis_read_data_tvalid_reg || m_axis_read_data_tready)) begin
        m_axis_read_data_tdata_reg  <= out_fifo_tdata[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tkeep_reg  <= out_fifo_tkeep[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tvalid_reg <= 1'b1;
        m_axis_read_data_tlast_reg  <= out_fifo_tlast[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tid_reg    <= out_fifo_tid[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tdest_reg  <= out_fifo_tdest[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        m_axis_read_data_tuser_reg  <= out_fifo_tuser[out_fifo_rd_ptr_reg[OUTPUT_FIFO_ADDR_WIDTH-1:0]];
        out_fifo_rd_ptr_reg <= out_fifo_rd_ptr_reg + 1;
    end

    if (rst) begin
        out_fifo_wr_ptr_reg <= 0;
        out_fifo_rd_ptr_reg <= 0;
        m_axis_read_data_tvalid_reg <= 1'b0;
    end
end

endmodule

`resetall
