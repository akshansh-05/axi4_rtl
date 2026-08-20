// File: axis_if.sv
// Description: Generic AXI4-Stream Protocol Interface
//              (Without modports and clocking blocks)

interface axis_if #(
    parameter DATA_WIDTH = 32,
    parameter KEEP_WIDTH = (DATA_WIDTH / 8),
    parameter ID_WIDTH   = 8,
    parameter DEST_WIDTH = 8,
    parameter USER_WIDTH = 1
)(
    input logic clk,
    input logic rst
);

    logic [DATA_WIDTH-1:0]  tdata;
    logic [KEEP_WIDTH-1:0]  tkeep;      // 1 bit per byte strobe
    logic                   tvalid;
    logic                   tready;
    logic                   tlast;      // Packet boundary indicator
    logic [ID_WIDTH-1:0]    tid;        // Stream identifier tag
    logic [DEST_WIDTH-1:0]  tdest;      // Stream routing destination
    logic [USER_WIDTH-1:0]  tuser;      // User sideband data

endinterface : axis_if
