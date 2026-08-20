// File: dma_desc_if.sv
// Description: Descriptor Command & Status Interface for AXI DMA Core
//              (Without modports and clocking blocks)

interface dma_desc_if #(
    parameter ADDR_WIDTH = 16,
    parameter LEN_WIDTH  = 20,
    parameter TAG_WIDTH  = 8,
    parameter ID_WIDTH   = 8,
    parameter DEST_WIDTH = 8,
    parameter USER_WIDTH = 1
)(
    input logic clk,
    input logic rst
);

    // 1. Global Controls
    logic                   read_enable;
    logic                   write_enable;
    logic                   write_abort;

    // 2. Read Descriptor Command Channel
    logic [ADDR_WIDTH-1:0]  read_desc_addr;
    logic [LEN_WIDTH-1:0]   read_desc_len;
    logic [TAG_WIDTH-1:0]   read_desc_tag;
    logic [ID_WIDTH-1:0]    read_desc_id;
    logic [DEST_WIDTH-1:0]  read_desc_dest;
    logic [USER_WIDTH-1:0]  read_desc_user;
    logic                   read_desc_valid;
    logic                   read_desc_ready;

    // 3. Read Descriptor Status Channel
    logic [TAG_WIDTH-1:0]   read_desc_status_tag;
    logic [3:0]             read_desc_status_error;     // DMA_ERROR codes (0=None, 4=SLVERR, 5=DECERR)
    logic                   read_desc_status_valid;

    // 4. Write Descriptor Command Channel
    logic [ADDR_WIDTH-1:0]  write_desc_addr;
    logic [LEN_WIDTH-1:0]   write_desc_len;
    logic [TAG_WIDTH-1:0]   write_desc_tag;
    logic                   write_desc_valid;
    logic                   write_desc_ready;

    // 5. Write Descriptor Status Channel
    logic [LEN_WIDTH-1:0]   write_desc_status_len;      // Actual byte length written
    logic [TAG_WIDTH-1:0]   write_desc_status_tag;
    logic [ID_WIDTH-1:0]    write_desc_status_id;
    logic [DEST_WIDTH-1:0]  write_desc_status_dest;
    logic [USER_WIDTH-1:0]  write_desc_status_user;
    logic [3:0]             write_desc_status_error;    // DMA_ERROR codes (0=None, 6=SLVERR, 7=DECERR)
    logic                   write_desc_status_valid;

endinterface : dma_desc_if
