// File: axi_if.sv
// Description: SystemVerilog Interface for AXI4 Memory-Mapped Bus
//              (Without modports and clocking blocks)

interface axi_if #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 16,
    parameter ID_WIDTH   = 8,
    parameter STRB_WIDTH = (DATA_WIDTH / 8)
)(
    input logic clk,
    input logic rst
);

    // 1. Write Address Channel (AW)
    logic [ID_WIDTH-1:0]    awid;
    logic [ADDR_WIDTH-1:0]  awaddr;
    logic [7:0]             awlen;      // Burst length: 0 to 255 (representing 1 to 256 beats)
    logic [2:0]             awsize;     // Bytes per beat: 3'b000=1B, 3'b001=2B, 3'b010=4B, etc.
    logic [1:0]             awburst;    // 2'b00=FIXED, 2'b01=INCR, 2'b10=WRAP
    logic                   awvalid;
    logic                   awready;

    // 2. Write Data Channel (W)
    logic [DATA_WIDTH-1:0]  wdata;
    logic [STRB_WIDTH-1:0]  wstrb;      // 1 bit per byte
    logic                   wlast;      // Indicates last beat of write burst
    logic                   wvalid;
    logic                   wready;

    // 3. Write Response Channel (B)
    logic [ID_WIDTH-1:0]    bid;
    logic [1:0]             bresp;      // 2'b00=OKAY, 2'b01=EXOKAY, 2'b10=SLVERR, 2'b11=DECERR
    logic                   bvalid;
    logic                   bready;

    // 4. Read Address Channel (AR)
    logic [ID_WIDTH-1:0]    arid;
    logic [ADDR_WIDTH-1:0]  araddr;
    logic [7:0]             arlen;      // Burst length: 0 to 255
    logic [2:0]             arsize;     // Bytes per beat
    logic [1:0]             arburst;    // 2'b00=FIXED, 2'b01=INCR, 2'b10=WRAP
    logic                   arvalid;
    logic                   arready;

    // 5. Read Data Channel (R)
    logic [ID_WIDTH-1:0]    rid;
    logic [DATA_WIDTH-1:0]  rdata;
    logic [1:0]             rresp;      // Response status (OKAY, EXOKAY, SLVERR, DECERR)
    logic                   rlast;      // Indicates last beat of read burst
    logic                   rvalid;
    logic                   rready;

endinterface : axi_if
