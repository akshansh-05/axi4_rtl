# AXI4 DMA Subsystem: Scoreboard Architecture & Code-Level Verification Guide

## 1. High-Level Theoretical Overview & Complete DMA Pipeline Dataflow

### 1.1 The Fundamental Verification Challenge
In high-performance System-on-Chip (SoC) architectures, the Direct Memory Access (DMA) Subsystem acts as a protocol and data transformer:
* **MM2S (Memory-to-Stream):** Converts **address-based memory bursts** (AMBA AXI4-Full) into **address-less byte streams** (AMBA AXI4-Stream).
* **S2MM (Stream-to-Memory):** Converts **address-less byte streams** into **address-based memory bursts**.

A traditional transaction-level scoreboard relying on standard 1-to-1 input-to-output FIFO queues fails because:
1. **Dimensional Disparity:** Memory bursts arrive in 32-bit aligned chunks with partial byte offsets and padding, whereas streaming packets represent contiguous, packed byte payloads.
2. **Elastic Pipelining & Latency Skew:** Internal skid buffers and FIFOs introduce dynamic multi-cycle latency between handshakes on opposing interfaces.
3. **Temporal Skew:** Streaming packets can arrive before or after host descriptor submission (Data-First vs. Descriptor-First timing skew).
4. **Dynamic Burst Slicing & Corner Cases:** Hardware clamps bursts at 4KB page boundaries, zero-pads bursts upon premature stream termination, and drops excess beats upon buffer overruns.

To overcome these challenges, `dma_scoreboard.sv` implements two decoupled, concurrently executing **Predictive Algorithmic Reference Models** that mirror hardware transformations in software.

```
+---------------------------------------------------------------------------------------------------------+
|                                      dma_scoreboard Architecture                                        |
|                                                                                                         |
|  [MM2S Pipeline]:                                                                                       |
|  1. Descriptor Cmd -----> Register Transfer Window: [start_addr, start_addr + xfer_len - 1]             |
|                                    |                                                                    |
|  2. Memory Read Bursts -> Windowing Algorithm: Extract requested bytes into expected_bytes[$]          |
|                                    |                                                                    |
|  3. Egress Stream Beats -> TKEEP Extraction: Collect stream payload into actual_bytes[$]                |
|                                    |                                                                    |
|  4. Pair-Wise Drain ----> Real-time Lockstep Byte Comparator: assert(expected == actual)               |
|                                    |                                                                    |
|  5. Completion Pulse ---> Temporal & In-Order Check -> Audit zero leftovers -> Retire Descriptor        |
|                                                                                                         |
|  [S2MM Pipeline]:                                                                                       |
|  1. Descriptor Cmd -----> Register Destination Buffer: [dest_addr, dest_addr + buffer_len - 1]          |
|                                    |                                                                    |
|  2. Ingress Stream Beats -> Golden Source: Unpack TKEEP-qualified stream bytes into expected_bytes[$]   |
|                                    |                                                                    |
|  3. Memory Write Bursts -> WSTRB Extraction: Collect memory-committed bytes into actual_bytes[$]       |
|                                    |                                                                    |
|  4. Pair-Wise Drain ----> Real-time Lockstep Byte Comparator: assert(expected == actual)               |
|                                    |                                                                    |
|  5. Completion Pulse ---> Verify status_len == expected_written -> Audit excess drop -> Retire         |
+---------------------------------------------------------------------------------------------------------+
```

### 1.2 MM2S Read Verification Concept
* **Hardware Dataflow:** The host issues a read descriptor (`addr`, `len`, `tag`). The DMA's AXI AR master generates memory read bursts. If the start address is unaligned (e.g., `0x1001`), the AXI burst starts at the 32-bit aligned boundary (`0x1000`). Memory returns 32-bit words (`RDATA`). The DMA's barrel shifter realigns the bytes, strips off discarded prefix bytes using `bubble_cycle_reg`, packs bytes, and emits an AXI-Stream packet on `m_axis_read_data_*` with `TLAST` at the final byte. Finally, a 1-cycle status pulse is emitted on `m_axis_read_desc_status_*`.
* **How Expected Values Are Generated (The Windowing Slicing Model):**  
  When `axi_rd_monitor` sees AXI read data words on the bus, the scoreboard calculates the exact physical byte address for every byte lane:
  $$\text{byte\_addr} = (\text{addr} \ \& \ \sim 3) + (i \times 4) + b$$
  It extracts **only** bytes falling strictly within the requested transfer window $[start\_addr, \ start\_addr + xfer\_len - 1]$ and pushes them into `desc.expected_bytes[$]`. Prefix bytes (before $start\_addr$) and suffix padding bytes are automatically sliced out.
* **How Actual Values Are Generated:**  
  When `axis_rd_monitor` captures the egress stream packet from `m_axis_read_data_*`, the scoreboard extracts every valid byte where `TKEEP[lane] == 1` and pushes them into `desc.actual_bytes[$]`.
* **How Verification Closes:**  
  The pair-wise comparator drains both queues in lockstep. When the status pulse arrives, the scoreboard confirms that the stream finished *before* status asserted, that exactly one packet was produced, that zero orphan bytes remain, and that the descriptor error status is `DMA_ERR_NONE`.

### 1.3 S2MM Write Verification Concept
* **Hardware Dataflow:** The host issues a write descriptor (`dest_addr`, `buffer_len`, `tag`). The peripheral streams ingress data into `s_axis_write_data_*`. The DMA captures bytes through its write barrel shifter and 2-cycle skid buffer FIFO (`out_fifo`), generates AXI `AW` bursts, and commits write beats with sparse `WSTRB` qualifiers (masking out prefix bytes before `dest_addr[1:0]`).  
  * *Corner 1 (Early Stream Termination):* If the stream ends early ($L_{stream} < L_{buffer}$), the DMA transitions to `STATE_FINISH_BURST` and zero-pads remaining committed beats with `WSTRB = 4'b0000`.
  * *Corner 2 (Buffer Overrun):* If the stream exceeds buffer capacity ($L_{stream} > L_{buffer}$), the DMA enters `STATE_DROP_DATA`, keeps `TREADY = 1`, and drains excess beats without modifying memory.
* **How Expected Values Are Generated (The Stream Golden Source):**  
  In S2MM, the **stream packet is the golden reference input**. When `axis_wr_monitor` snoops the ingress packet, the scoreboard unpacks all `TKEEP`-qualified bytes into `desc.expected_bytes[$]`. It immediately tags the transfer as nominal, early termination, or excess data.
* **How Actual Values Are Generated (Sparse WSTRB Extraction):**  
  When `axi_wr_monitor` snoops memory write bursts on `AW`/`W`/`B`, the scoreboard loops through every beat $i$ and byte lane $b$. **Only when $WSTRB[i][b] == 1$** does it push the byte into `desc.actual_bytes[$]`. In early stream termination, the DUT emits dummy zero-padded beats with `WSTRB = 4'b0000`; because the scoreboard filters by active strobes, zero bytes are collected for padding beats, preventing false mismatches.
* **How Verification Closes:**  
  The pair-wise comparator validates memory bytes against stream bytes. When write status arrives, it asserts that `status_len` matches actual committed bytes, confirms that excess stream bytes were dropped cleanly, and deletes the descriptor.

## 2. Tracking Data Structures & Dual-Lookup Architecture (Lines 53–156)

The scoreboard defines two dedicated transaction tracking classes:

### 2.1 MM2S Descriptor Tracking Class (`mm2s_desc_txn`, Lines 54–84)
```systemverilog
class mm2s_desc_txn #(parameter ADDR_WIDTH = 16, parameter LEN_WIDTH = 20, parameter TAG_WIDTH = 8);
    bit [ADDR_WIDTH-1:0] start_addr;
    bit [LEN_WIDTH-1:0]  xfer_len;
    bit [TAG_WIDTH-1:0]  tag;
    byte                 expected_bytes[$];          // Golden bytes sliced from AXI-MM read bursts
    byte                 actual_bytes[$];            // Egress stream bytes observed on AXIS
    int unsigned         total_bytes_expected;
    int unsigned         bytes_matched;
    int unsigned         packet_count;               // Stream packets observed (must be exactly 1)
    int unsigned         stream_pkt_bytes;           // Total bytes in the observed stream packet
    bit [ADDR_WIDTH-1:0] expected_next_burst_addr;   // Next expected AXI read burst address
    bit                  cmd_seen;
    bit                  stream_done;
    bit                  status_done;
    dma_error_e          status_error;
endclass
```

### 2.2 S2MM Descriptor Tracking Class (`s2mm_desc_txn`, Lines 87–123)
```systemverilog
class s2mm_desc_txn #(parameter ADDR_WIDTH = 16, parameter LEN_WIDTH = 20, parameter TAG_WIDTH = 8);
    bit [ADDR_WIDTH-1:0] dest_addr;
    bit [LEN_WIDTH-1:0]  buffer_len;
    bit [TAG_WIDTH-1:0]  tag;
    byte                 expected_bytes[$];          // Golden bytes assembled from ingress stream
    byte                 actual_bytes[$];            // Bytes captured from AXI write bursts (WSTRB=1)
    int unsigned         total_stream_bytes;
    int unsigned         total_written_bytes;
    int unsigned         bytes_matched;
    bit [ADDR_WIDTH-1:0] expected_next_awaddr;       // Progression check across multi-bursts
    bit                  cmd_seen;
    bit                  stream_done;                // TLAST observed
    bit                  all_bursts_done;
    bit                  status_done;
    dma_error_e          status_error;
    bit [LEN_WIDTH-1:0]  status_len;
    bit                  is_early_termination;
    bit                  is_excess_data;
endclass
```

### 2.3 Dual-Lookup Storage Architecture (Lines 151–156)
To support both sequential stream pipelining and out-of-order status retirement, the scoreboard maintains two parallel representations of active descriptors:
```systemverilog
mm2s_txn_type  mm2s_queue[$];                          // In-order FIFO queue
mm2s_txn_type  mm2s_by_tag[bit[TAG_WIDTH-1:0]];       // Tag-indexed associative array

s2mm_txn_type  s2mm_queue[$];                          // In-order FIFO queue
s2mm_txn_type  s2mm_by_tag[bit[TAG_WIDTH-1:0]];       // Tag-indexed associative array
axis_item_type unmatched_axis_pkts[$];                // Early stream buffer for data-first skew
```
* **Why both a queue and an associative array?**  
  Data beats progress sequentially through the hardware FIFO (`*_queue[0]`), but status completion tokens and out-of-order descriptors are looked up instantly by transaction tag (`*_by_tag[tag]`).

## 3. Multi-Port TLM Analysis Port Ingestion (Lines 13–51, 138–194)

The scoreboard connects to all 6 agents (8 analysis ports total) via `uvm_analysis_imp_decl` macros:
```systemverilog
`uvm_analysis_imp_decl(_dma_rd_cmd)     // Read Descriptor Monitor -> cmd_ap
`uvm_analysis_imp_decl(_dma_rd_status)  // Read Descriptor Monitor -> status_ap
`uvm_analysis_imp_decl(_axis_rd)        // Egress Stream Monitor   -> ap
`uvm_analysis_imp_decl(_axi_rd)         // Memory Read Monitor     -> ap

`uvm_analysis_imp_decl(_dma_wr_cmd)     // Write Descriptor Monitor -> cmd_ap
`uvm_analysis_imp_decl(_dma_wr_status)  // Write Descriptor Monitor -> status_ap
`uvm_analysis_imp_decl(_axis_wr)        // Ingress Stream Monitor  -> ap
`uvm_analysis_imp_decl(_axi_wr)         // Memory Write Monitor    -> ap
```
These macros map incoming monitor transactions into dedicated `write_*` functions without function name collisions.

## 4. MM2S Code Execution Flow (Tracing a Read Transfer)

### 4.1 Command Registration: `write_dma_rd_cmd()` (Lines 197–213)
When `dma_rd_desc_agent` drives a read descriptor and the DUT asserts `ready`, `write_dma_rd_cmd()` is called:
```systemverilog
virtual function void write_dma_rd_cmd(dma_desc_item_type t);
    mm2s_txn_type desc = new();
    desc.start_addr               = t.addr;
    desc.xfer_len                 = t.len;
    desc.tag                      = t.tag;
    desc.total_bytes_expected     = t.len;
    desc.expected_next_burst_addr = t.addr;
    desc.cmd_seen                 = 1'b1;

    mm2s_queue.push_back(desc);
    mm2s_by_tag[t.tag] = desc;
    mm2s_cmd_count++;
endfunction
```
* **Purpose:** Allocates a tracking object, establishes the start address and byte length, and registers it in both `mm2s_queue` and `mm2s_by_tag`.

### 4.2 Memory Read Slicing & Windowing: `write_axi_rd()` (Lines 216–246)
When the DMA read master fetches data from memory across `AR` and `R` channels, `axi_rd_monitor` broadcasts an `axi_seq_item`. This invokes `write_axi_rd()`:
```systemverilog
virtual function void write_axi_rd(axi_item_type t);
    desc = mm2s_queue[0];

    // 1. Check sequential address progression
    if (t.addr !== desc.expected_next_burst_addr) begin
        `uvm_error("SCB_MM2S_BURST_ADDR", ...)
        mm2s_addr_errors++;
    end
    desc.expected_next_burst_addr = (t.addr & ~(STRB_WIDTH - 1)) + ((t.len + 1) * STRB_WIDTH);

    // 2. Unpack 32-bit read data words into individual byte stream (Windowing Algorithm)
    for (int i = 0; i <= t.len; i++) begin
        for (int b = 0; b < STRB_WIDTH; b++) begin
            int byte_addr = (t.addr & ~(STRB_WIDTH - 1)) + (i * STRB_WIDTH) + b;
            // Only collect bytes within descriptor boundary [start_addr, start_addr + xfer_len - 1]
            if (byte_addr >= desc.start_addr && byte_addr < (desc.start_addr + desc.xfer_len)) begin
                byte byte_val = (t.data[i] >> (8 * b)) & 8'hFF;
                desc.expected_bytes.push_back(byte_val);
            end
        end
    end

    compare_mm2s_data(desc);
endfunction
```
* **Logic Explained:**  
  1. Verifies that the DUT's AXI burst address matches `expected_next_burst_addr`. Catches any 4KB split address calculation bugs in RTL.
  2. The nested loops evaluate every byte lane $b$ of every beat $i$.
  3. If `byte_addr` falls between $start\_addr$ and $start\_addr + xfer\_len - 1$, it is a valid transfer byte and gets pushed into `desc.expected_bytes[$]`. Prefix bytes (e.g., byte 0 of an unaligned address `0x1001`) are automatically ignored.
  4. Calls `compare_mm2s_data(desc)` to compare against any stream bytes already received.

### 4.3 Egress Stream Collection: `write_axis_rd()` (Lines 249–293)
When the DMA read engine emits an AXI-Stream packet, `axis_rd_monitor` broadcasts an `axis_seq_item`. This invokes `write_axis_rd()`:
```systemverilog
virtual function void write_axis_rd(axis_item_type t);
    desc = mm2s_queue[0];

    // 1. Extract payload bytes using TKEEP lane qualifiers
    for (int beat = 0; beat < t.data.size(); beat++) begin
        for (int lane = 0; lane < STRB_WIDTH; lane++) begin
            if (t.keep[beat][lane]) begin
                byte act_byte = (t.data[beat] >> (8 * lane)) & 8'hFF;
                desc.actual_bytes.push_back(act_byte);
                pkt_bytes++;
            end
        end
    end

    // 2. Protocol Framing Checks
    if (desc.packet_count > 0) begin
        `uvm_error("SCB_MM2S_EXTRA_PKT", "Multiple stream packets produced for 1 descriptor!")
        mm2s_pkt_errors++;
    end
    if (pkt_bytes != desc.xfer_len) begin
        `uvm_error("SCB_MM2S_PKT_LEN", "TLAST arrived at wrong byte count!")
        mm2s_pkt_errors++;
    end

    desc.packet_count++;
    desc.stream_pkt_bytes = pkt_bytes;
    desc.stream_done      = 1'b1;

    compare_mm2s_data(desc);
    if (desc.status_done) check_and_retire_mm2s(desc);
endfunction
```
* **Logic Explained:**  
  1. Inspects `t.keep[beat][lane]`. Only pushed bytes where `TKEEP` is high. This accurately models fractional tail transfers (e.g., `TKEEP = 4'b0011` on the final beat).
  2. Asserts that `packet_count == 0` (verifies 1-to-1 packet-to-descriptor framing; catches spurious intermediate `TLAST` assertions).
  3. Asserts that `pkt_bytes == desc.xfer_len` (verifies that `TLAST` fired on the exact final byte).
  4. Marks `desc.stream_done = 1'b1` and calls `compare_mm2s_data(desc)`.

### 4.4 Pair-Wise Real-Time Comparator: `compare_mm2s_data()` (Lines 296–311)
```systemverilog
virtual function void compare_mm2s_data(mm2s_txn_type desc);
    while (desc.expected_bytes.size() > 0 && desc.actual_bytes.size() > 0) begin
        byte exp_b = desc.expected_bytes.pop_front();
        byte act_b = desc.actual_bytes.pop_front();
        mm2s_bytes_checked++;

        if (exp_b === act_b) begin
            mm2s_byte_matches++;
            desc.bytes_matched++;
        end else begin
            mm2s_byte_mismatches++;
            `uvm_error("SCB_MM2S_DATA_MISMATCH", $sformatf("MM2S Data Mismatch at byte %0d...", desc.bytes_matched))
        end
    end
endfunction
```
* **Logic Explained:**  
  Uses a `while` loop that pops from `expected_bytes` and `actual_bytes` simultaneously. It executes incrementally whenever new memory read beats or stream beats arrive. If a mismatch occurs, it prints the exact byte index and logs a `uvm_error`.

### 4.5 Status Retirement & Cleanliness: `write_dma_rd_status()` & `check_and_retire_mm2s()` (Lines 314–396)
When the DMA emits the 1-cycle completion pulse on `m_axis_read_desc_status_*`:
```systemverilog
virtual function void write_dma_rd_status(dma_desc_item_type t);
    desc = mm2s_by_tag[t.status_tag];
    desc.status_done  = 1'b1;
    desc.status_error = t.status_error;

    // 1. In-order queue head check
    if (mm2s_queue.size() > 0 && mm2s_queue[0].tag !== t.status_tag)
        `uvm_error("SCB_MM2S_OOO", "Status arrived out of order!")

    // 2. Temporal assertion: Status MUST NOT arrive before egress stream packet finishes!
    if (!desc.stream_done)
        `uvm_error("SCB_MM2S_EARLY_STATUS", "Status pulsed while stream packet is still draining!")

    if (desc.stream_done) check_and_retire_mm2s(desc);
endfunction
```
Then, `check_and_retire_mm2s()` (Lines 349–396) enforces the final audit:
1. Drains any remaining bytes in `compare_mm2s_data(desc)`.
2. Enforces `desc.bytes_matched == desc.xfer_len`.
3. Asserts `desc.expected_bytes.size() == 0` (zero orphan expected bytes).
4. Asserts `desc.actual_bytes.size() == 0` (zero orphan actual bytes).
5. Asserts `desc.status_error == DMA_ERR_NONE`.
6. Removes `desc` from `mm2s_queue` and `mm2s_by_tag`.
7. Prints `SCB_MM2S_PASS`.

## 5. S2MM Code Execution Flow (Tracing a Write Transfer)

### 5.1 Write Command Registration: `write_dma_wr_cmd()` (Lines 399–439)
```systemverilog
virtual function void write_dma_wr_cmd(dma_desc_item_type t);
    // 1. Duplicate tag check
    if (s2mm_by_tag.exists(t.tag)) `uvm_error("SCB_S2MM_DUP_TAG", ...)

    // 2. Legality checks
    if (t.len == 0) `uvm_error("SCB_S2MM_ZERO_LEN", ...)
    if ((t.addr + t.len) > (1 << ADDR_WIDTH)) `uvm_error("SCB_S2MM_ADDR_WRAP", ...)

    desc = new();
    desc.dest_addr            = t.addr;
    desc.buffer_len           = t.len;
    desc.tag                  = t.tag;
    desc.expected_next_awaddr = t.addr;
    desc.cmd_seen             = 1'b1;

    // 3. Data-First Skew Pairing (Test S2MM_10)
    if (unmatched_axis_pkts.size() > 0) begin
        axis_item_type early_pkt = unmatched_axis_pkts.pop_front();
        bind_stream_to_desc(desc, early_pkt);
    end

    s2mm_queue.push_back(desc);
    s2mm_by_tag[t.tag] = desc;
endfunction
```
* **Logic Explained:**  
  Checks for duplicate active tags, bounds-checks the destination buffer within memory, and initializes `desc`.  
  * *Data-First Timing Skew Handling:* In test `dma_wr_skew_test`, the streaming packet arrives *before* host software issues the descriptor. If an unmatched stream packet is already waiting in `unmatched_axis_pkts[$]`, it pairs immediately via `bind_stream_to_desc()`.

### 5.2 Ingress Stream Ingestion: `write_axis_wr()` & `bind_stream_to_desc()` (Lines 442–513)
When `axis_wr_agent` drives an ingress stream packet into the DMA:
```systemverilog
virtual function void bind_stream_to_desc(s2mm_txn_type desc, axis_item_type t);
    for (int beat = 0; beat < t.data.size(); beat++) begin
        for (int lane = 0; lane < STRB_WIDTH; lane++) begin
            if (t.keep[beat][lane]) begin
                byte act_byte = (t.data[beat] >> (8 * lane)) & 8'hFF;
                desc.expected_bytes.push_back(act_byte);
                pkt_bytes++;
            end
        end
    end
    desc.total_stream_bytes = pkt_bytes;
    desc.stream_done        = 1'b1;

    // Classify scenario
    if (pkt_bytes < desc.buffer_len)
        desc.is_early_termination = 1'b1; // Scenario S2MM_07
    else if (pkt_bytes > desc.buffer_len)
        desc.is_excess_data = 1'b1;       // Scenario S2MM_08
endfunction
```
* **Logic Explained:**  
  Unpacks incoming stream bytes into `desc.expected_bytes[$]`. Compares `pkt_bytes` against `buffer_len`:
  * If $pkt\_bytes < buffer\_len$, sets `is_early_termination = 1` (DUT must zero-pad).
  * If $pkt\_bytes > buffer\_len$, sets `is_excess_data = 1` (DUT must drop excess data in `STATE_DROP_DATA`).

### 5.3 Memory Write Burst Progression: `write_axi_wr()` (Lines 516–605)
When the DMA write master emits write bursts on the AXI bus (`AW`, `W`, `B`):
```systemverilog
virtual function void write_axi_wr(axi_item_type t);
    desc = correlate_s2mm_descriptor(t.addr);

    // 1. Check AWADDR sequential progression
    if (t.addr !== desc.expected_next_awaddr) `uvm_error("SCB_S2MM_BURST_ADDR", ...)

    // 2. 4KB boundary crossing check (AXI4 Section A3.4.1)
    if (((t.addr & 12'hFFF) + ((t.len + 1) * STRB_WIDTH)) > 4096)
        `uvm_error("SCB_S2MM_4KB_CROSS", "Write burst crossed 4KB boundary!")

    // 3. Burst length check (Clamped to 16 beats)
    if (t.len > 15) `uvm_error("SCB_S2MM_AWLEN_OVERFLOW", "AWLEN exceeds 16 beats!")

    // 4. Response check
    if (t.bresp !== AXI_RESP_OKAY) `uvm_error("SCB_S2MM_BRESP_ERR", "Non-OKAY response!")

    desc.expected_next_awaddr = t.addr + ((t.len + 1) * STRB_WIDTH);

    // 5. Extract bytes qualified by WSTRB
    for (int i = 0; i <= t.len; i++) begin
        for (int b = 0; b < STRB_WIDTH; b++) begin
            if (t.strb[i][b]) begin
                byte act_byte = (t.data[i] >> (8 * b)) & 8'hFF;
                desc.actual_bytes.push_back(act_byte);
                desc.total_written_bytes++;
            end
        end
    end

    compare_s2mm_data(desc);
    if (desc.status_done) check_and_retire_s2mm(desc);
endfunction
```
* **Logic Explained:**  
  1. Correlates the AXI burst with the active descriptor by destination address window (Lines 528–556).
  2. Enforces hardware protocol rules: sequential `AWADDR` progression, 4KB page boundary non-crossing, burst length $\le 16$ beats, and `BRESP == OKAY`.
  3. Loops through beats and byte lanes: **only if `t.strb[i][b] == 1`** is the byte collected into `desc.actual_bytes[$]`.
  4. Calls `compare_s2mm_data(desc)` to compare against stream bytes in real time.

### 5.4 S2MM Retirement & Corner Case Verification: `check_and_retire_s2mm()` (Lines 656–725)
When `write_dma_wr_status()` receives the write status pulse, it invokes `check_and_retire_s2mm()`:
```systemverilog
virtual function void check_and_retire_s2mm(s2mm_txn_type desc);
    // 1. Calculate expected written bytes
    int unsigned exp_written_bytes = (desc.is_early_termination) ?
                                      desc.total_stream_bytes : desc.buffer_len;

    // Do not retire until all bursts, stream, and status are done
    if (!desc.cmd_seen || !desc.status_done || !desc.stream_done ||
        (desc.total_written_bytes < exp_written_bytes)) return;

    compare_s2mm_data(desc);

    // 2. Verify bytes matched
    if (desc.bytes_matched != exp_written_bytes)
        `uvm_error("SCB_S2MM_LEN_MISMATCH", ...)

    // 3. Verify status reported transfer length
    if (desc.status_len != exp_written_bytes)
        `uvm_error("SCB_S2MM_STAT_LEN_MISMATCH", ...)

    // 4. Verify clean drop of excess stream data (Test S2MM_08)
    if (desc.is_excess_data) begin
        int unsigned dropped_bytes = desc.expected_bytes.size();
        `uvm_info("SCB_S2MM_DROP_CHECK", $sformatf("Verified %0d excess bytes cleanly dropped...", dropped_bytes), UVM_LOW)
        desc.expected_bytes.delete(); // Clear dropped bytes cleanly
    end

    // 5. Audit zero leftover orphan bytes
    if (desc.expected_bytes.size() != 0) `uvm_error("SCB_S2MM_ORPHAN_EXP", ...)
    if (desc.actual_bytes.size() != 0)   `uvm_error("SCB_S2MM_ORPHAN_ACT", ...)

    // 6. Delete from tracking structures
    s2mm_queue.delete(found_idx);
    s2mm_by_tag.delete(desc.tag);
    `uvm_info("SCB_S2MM_PASS", ...)
endfunction
```
* **Logic Explained:**  
  * For normal transfers: `exp_written_bytes = buffer_len`.
  * For early termination transfers: `exp_written_bytes = total_stream_bytes`.
  * Verifies that `status_len` matches `exp_written_bytes`.
  * In excess data scenarios ($L_{stream} > L_{buffer}$), verifies that the remaining unwritten stream bytes equal the excess payload, confirms they were dropped without modifying memory, and deletes them.
  * Confirms zero orphan bytes remain and deletes the descriptor.

## 6. End-of-Test Cleanliness Check & Final Verdict (Lines 728–789)

### 6.1 `check_phase()`: Zero-Leak / Hang Detection (Lines 728–746)
At the end of simulation, UVM calls `check_phase()`:
```systemverilog
virtual function void check_phase(uvm_phase phase);
    mm2s_hang_count = mm2s_queue.size();
    s2mm_hang_count = s2mm_queue.size();

    if (mm2s_hang_count != 0)
        `uvm_error("SCB_MM2S_HANG", $sformatf("%0d MM2S descriptor(s) never completed!", mm2s_hang_count))

    if (s2mm_hang_count != 0)
        `uvm_error("SCB_S2MM_HANG", $sformatf("%0d S2MM descriptor(s) never completed!", s2mm_hang_count))

    if (unmatched_axis_pkts.size() != 0)
        `uvm_error("SCB_S2MM_ORPHAN_PKT", "Ingress stream packets never paired with descriptor!")
endfunction
```
* **Why this is critical:** If the DMA hardware hangs mid-burst or drops a completion status pulse, data comparison might pass for previous beats, but `mm2s_queue.size()` will not be zero. This check guarantees that **no descriptor or packet was abandoned**.

### 6.2 `report_phase()`: Authoritative Scorecard Summary (Lines 749–789)
```systemverilog
virtual function void report_phase(uvm_phase phase);
    mm2s_errors = (mm2s_byte_mismatches > 0) || (mm2s_status_mismatches > 0) ||
                  (mm2s_pkt_errors > 0)      || (mm2s_addr_errors > 0)        ||
                  (mm2s_hang_count > 0)      || (mm2s_queue.size() > 0);

    s2mm_errors = (s2mm_byte_mismatches > 0) || (s2mm_status_mismatches > 0) ||
                  (s2mm_pkt_errors > 0)      || (s2mm_addr_errors > 0)        ||
                  (s2mm_hang_count > 0)      || (s2mm_queue.size() > 0)       ||
                  (unmatched_axis_pkts.size() > 0);

    if (!mm2s_errors) `uvm_info("SCB_MM2S", "VERDICT: [PASSED] MM2S transfer verified successfully.", UVM_LOW)
    else              `uvm_error("SCB_MM2S", "VERDICT: [FAILED] MM2S errors detected.")

    if (!s2mm_errors) `uvm_info("SCB_S2MM", "VERDICT: [PASSED] S2MM transfer verified successfully.", UVM_LOW)
    else              `uvm_error("SCB_S2MM", "VERDICT: [FAILED] S2MM errors detected.")
endfunction
```
Only when:
1. `bytes_checked == bytes_matched`
2. `byte_mismatches == 0`
3. `addr_errors == 0` (4KB boundary & progression checks passed)
4. `pkt_errors == 0` (1-to-1 packet framing passed)
5. `status_mismatches == 0` (error codes and lengths matched)
6. `hang_count == 0` (all descriptors completed)

...does the scoreboard print the final **`VERDICT: [PASSED]`**, allowing Cadence Xcelium to complete with exit code 0.

## 7. Summary Checklist of All Checkers in `dma_scoreboard.sv`

| Checker Name | Line # | What Hardware Bug It Catches in RTL |
| :--- | :---: | :--- |
| `SCB_MM2S_BURST_ADDR` | 228 | AXI read burst address progression error or 4KB boundary splitting math bug. |
| `SCB_MM2S_EXTRA_PKT` | 273 | Spurious intermediate `TLAST` asserted by the DMA read engine before transfer completes. |
| `SCB_MM2S_PKT_LEN` | 279 | Stream packet terminated early or late relative to requested descriptor byte count. |
| `SCB_MM2S_DATA_MISMATCH`| 308 | Data corruption, barrel shifter bit-shift error, or bubble cycle discard failure. |
| `SCB_MM2S_OOO` | 330 | Read descriptor completion status tag returned out of order. |
| `SCB_MM2S_EARLY_STATUS`| 337 | Status pulse asserted before egress stream packet has completely drained out of FIFO. |
| `SCB_S2MM_DUP_TAG` | 405 | Hardware accepted a command tag that is already pending completion. |
| `SCB_S2MM_BURST_ADDR` | 560 | S2MM AXI write burst address progression mismatch across multi-bursts. |
| `SCB_S2MM_4KB_CROSS` | 566 | S2MM write burst crossed a 4KB boundary without splitting (AXI4 protocol violation). |
| `SCB_S2MM_AWLEN_OVERFLOW`| 572 | S2MM write burst length exceeded hardware maximum clamp limit of 16 beats. |
| `SCB_S2MM_DATA_MISMATCH`| 621 | Memory write data mismatch against ingress stream, or sparse `WSTRB` strobe gap error. |
| `SCB_S2MM_STAT_LEN_MISMATCH`| 684| `status_len` pulse reported incorrect written byte count on early termination or buffer overrun. |
| `SCB_S2MM_DROP_CHECK` | 697 | Verified that excess stream beats were cleanly dropped in `STATE_DROP_DATA` without corrupting memory. |
| `SCB_*_HANG` | 735, 739 | Unfinished transfers or dropped status pulses at simulation end. |
