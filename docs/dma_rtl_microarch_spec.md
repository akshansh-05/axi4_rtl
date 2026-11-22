# AXI4 DMA Subsystem: Hardware Architecture & RTL Microarchitecture

## 1. Executive Hardware Architectural Overview

Direct Memory Access (DMA) controllers serve as the primary high-bandwidth bridge between high-speed streaming I/O peripherals (e.g., Ethernet MACs, PCIe endpoints, image sensors) and multi-channel memory-mapped interconnects in System-on-Chip (SoC) architectures. 

The AMBA AXI4 DMA Subsystem implemented in this project provides a high-throughput, full-duplex, parameterizable bridge between the **AMBA AXI4-Stream Protocol (ARM IHI0051B)** and the **AMBA AXI4-Full Memory-Mapped Protocol (ARM IHI0022H)**.

The top-level hardware module ([`axi_dma.v`](file:///c:/Users/admin/OneDrive/Desktop/axi_vip_project/axi4_rtl/axi_master_rtl/axi_dma.v)) integrates two decoupled, concurrently operating hardware engines:
1. **MM2S Read Engine ([`axi_dma_rd.v`](file:///c:/Users/admin/OneDrive/Desktop/axi_vip_project/axi4_rtl/axi_master_rtl/axi_dma_rd.v)):** Manages Memory-to-Stream data movements by issuing AXI4 Read Address (AR) bursts, gathering Read Data (R) beats, shifting and packing bytes, and serializing framed packets onto the AXI4-Stream master interface.
2. **S2MM Write Engine ([`axi_dma_wr.v`](file:///c:/Users/admin/OneDrive/Desktop/axi_vip_project/axi4_rtl/axi_master_rtl/axi_dma_wr.v)):** Manages Stream-to-Memory data movements by capturing AXI4-Stream ingress packets, buffering beats through an internal skid FIFO, aligning bytes via sparse write strobes (`WSTRB`), and committing memory writes via AXI4 Write Address (AW), Write Data (W), and Write Response (B) channels.

```
                           +-------------------------------------------------------+
                           |               AXI4 DMA Subsystem (axi_dma)            |
                           |                                                       |
  [Host Read Descriptors]  |  +--------------------+       +--------------------+  |  [Host Write Descriptors]
  s_axis_read_desc_* ----->|  |                    |       |                    |  |<----- s_axis_write_desc_*
  m_axis_read_desc_stat <--|  |  MM2S Read Engine  |       |  S2MM Write Engine |  |-----> m_axis_write_desc_stat
                           |  |   (axi_dma_rd)     |       |   (axi_dma_wr)     |  |
  [Egress Stream Master]   |  |                    |       |                    |  |  [Ingress Stream Slave]
  m_axis_read_data_* <-----|  |  * Dual 2-State    |       |  * 5-State Master  |  |<----- s_axis_write_data_*
                           |  |    FSMs            |       |    FSM             |  |
  [AXI4 Read Master]       |  |  * Read Barrel     |       |  * 2-Cycle Skid    |  |  [AXI4 Write Master]
  m_axi_ar* -------------->|  |    Shifter         |       |    Buffer FIFO     |  |------> m_axi_aw* / m_axi_w*
  m_axi_r* <---------------|  |  * 4KB Clamp Adder |       |  * Write Shifter   |  |<------ m_axi_b*
                           |  +--------------------+       +--------------------+  |
                           +-------------------------------------------------------+
```

### 1.1 Architectural Parameters

The subsystem is parameterizable at elaboration time to match diverse bus widths and platform requirements:

| Parameter Name | Default Value | Microarchitectural Significance |
| :--- | :---: | :--- |
| `AXI_DATA_WIDTH` | 32 | Memory data bus width in bits (32, 64, 128, 512). |
| `AXI_ADDR_WIDTH` | 16 | Memory address width in bits (up to 64-bit addressing). |
| `AXI_STRB_WIDTH` | 4 | Byte strobe width: `AXI_DATA_WIDTH / 8`. |
| `AXI_MAX_BURST_LEN` | 16 | Maximum beats per generated AXI burst (1 to 256; clamped to 16 in this IP). |
| `AXIS_DATA_WIDTH` | 32 | Streaming data bus width (synchronized with memory datapath). |
| `LEN_WIDTH` | 20 | Transfer length counter width, supporting transfer sizes up to 1 MB ($2^{20}\text{ bytes}$). |
| `TAG_WIDTH` | 8 | Descriptor tracking tag width, supporting up to 256 in-flight tags. |
| `ENABLE_UNALIGNED` | 1 | Enables internal multi-stage barrel shifters for unaligned byte transfers. |

## 2. MM2S Read Engine Microarchitecture (`axi_dma_rd.v`)

The MM2S Read Engine coordinates fetching data words from arbitrary memory addresses and presenting a packed, contiguous byte stream on the AXI-Stream interface.

### 2.1 Dual Independent 2-State Finite State Machines

A critical architectural feature of [`axi_dma_rd.v`](file:///c:/Users/admin/OneDrive/Desktop/axi_vip_project/axi4_rtl/axi_master_rtl/axi_dma_rd.v) is the total decoupling of AXI Read Address (`AR`) generation from AXI-Stream Read Data (`R`) consumption. This is accomplished using **two independent 2-state FSMs** communicating via an internal command parameter pipeline:

```
+------------------------------------+         +------------------------------------+
|  FSM 1: AXI AR Generator           |         |  FSM 2: AXIS Data Assembler        |
|  (axi_state_reg)                   |         |  (axis_state_reg)                  |
|                                    |         |                                    |
|   +----------------------------+   |         |   +----------------------------+   |
|   | AXI_STATE_IDLE (1'd0)      |   | Command |   | AXIS_STATE_IDLE (1'd0)     |   |
|   | Await descriptor, compute  |   | Pipeline|   | Await transfer parameters  |   |
|   | transfer sizing            |   | FIFO    |   | from axi_state             |   |
|   +--------------+-------------+   |-------->|   +--------------+-------------+   |
|                  |                 |         |                  |                 |
|                  v                 |         |                  v                 |
|   +----------------------------+   |         |   +----------------------------+   |
|   | AXI_STATE_START (1'd1)     |   |         |   | AXIS_STATE_READ (1'd1)     |   |
|   | 4KB clamp calculation,     |   |         |   | Gather R beats, barrel-    |   |
|   | assert ARVALID for bursts  |   |         |   | shift, generate TKEEP/TLAST|   |
|   +----------------------------+   |         |   +----------------------------+   |
+------------------------------------+         +------------------------------------+
```

#### FSM 1: AXI Address Generation FSM (`axi_state_reg`)
* **`AXI_STATE_IDLE` (1'd0):** The engine monitors `s_axis_read_desc_valid`. When asserted and space exists in the internal command pipeline (`axis_cmd_ready`), the engine latches `addr_reg`, `op_word_count_reg`, and metadata tags. It computes initial start offsets and transitions to `AXI_STATE_START`.
* **`AXI_STATE_START` (1'd1):** The engine calculates the safe burst length for the current AXI transfer by evaluating the 4KB boundary clamp. It drives `m_axi_araddr`, `m_axi_arlen`, and asserts `m_axi_arvalid`. 
  * If the handshake completes (`m_axi_arvalid && m_axi_arready`) and remaining bytes exist (`op_word_count_reg > tr_word_count_reg`), the FSM remains in `AXI_STATE_START` to issue subsequent bursts.
  * Once the entire transfer length has been issued on the AR channel, it transitions back to `AXI_STATE_IDLE`.

#### FSM 2: AXI-Stream Data Assembler FSM (`axis_state_reg`)
* **`AXIS_STATE_IDLE` (1'd0):** Sits idle awaiting transfer parameters from FSM 1 via `axis_cmd_valid_reg`. Once valid parameters are handshaken, it loads byte offsets, beat countdown counters, and transitions to `AXIS_STATE_READ`.
* **`AXIS_STATE_READ` (1'd1):** Actively receives memory read data beats (`m_axi_rvalid && m_axi_rready`), drives the barrel shifter, packs stream beats, manages the initial unaligned discard cycle (`bubble_cycle_reg`), generates byte qualifiers (`TKEEP`), and asserts `TLAST` and status on the final beat before returning to `AXIS_STATE_IDLE`.

### 2.2 Microarchitectural Building Blocks in `axi_dma_rd.v`

#### 1. Internal Command Parameter Pipeline
FSM 1 passes transfer attributes to FSM 2 through a registered command interface:
* `axis_cmd_offset_reg`: Starting byte offset within the first 32-bit word (`addr[1:0]`).
* `axis_cmd_last_cycle_offset_reg`: Ending byte offset within the final word.
* `axis_cmd_input_cycle_count_reg`: Total AXI read beats expected from the memory slave.
* `axis_cmd_output_cycle_count_reg`: Total AXI-Stream beats to be emitted.
* `axis_cmd_bubble_cycle_reg`: Flag indicating that the first word contains only discarded prefix bytes.
* `axis_cmd_valid_reg` / `axis_cmd_ready`: Inter-FSM flow control handshake.

This pipeline allows the AR address generator to race ahead and pre-fetch subsequent descriptor bursts while the data assembler is still streaming previous data words to the peripheral.

#### 2. Multi-Stage Barrel Shifter (`ENABLE_UNALIGNED = 1`)
When reading from an unaligned address (e.g., address `0x1001`), the AXI memory slave returns 32-bit aligned words starting at `0x1000`. The Read Barrel Shifter shifts the data right by `offset_reg` bytes:
* **Holding Register (`save_axis_tdata_reg`):** Caches the upper unaligned bytes of word $k$ to concatenate with the lower bytes of word $k+1$.
* **Combinational Crossbar:** Merges `save_axis_tdata_reg` and incoming `m_axi_rdata`:
  ```verilog
  m_axis_read_data_tdata = (m_axi_rdata << (offset * 8)) | (save_axis_tdata_reg >> ((4 - offset) * 8));
  ```
* **Bubble Cycle Discard Logic (`bubble_cycle_reg`):** If an unaligned transfer begins with offset $> 0$, the first incoming memory beat contains fewer than 4 valid bytes. If these bytes cannot fill a complete stream beat, `bubble_cycle_reg` suppresses `m_axis_read_data_tvalid` for 1 cycle while latching the data into `save_axis_tdata_reg`.

#### 3. 4KB Page Boundary Clamp Combinational Logic
Per ARM AMBA AXI4 Section A3.4.1, bursts must never cross a 4KB boundary ($0x1000$). The address generation logic calculates:
```verilog
wire [11:0] bytes_to_boundary = 13'd4096 - addr_reg[11:0];
wire [7:0]  max_burst_beats   = bytes_to_boundary >> 2;
```
The burst length is clamped to the minimum of three constraints:
$$ARLEN = \min\left(15, \ \text{max\_burst\_beats} - 1, \ \left\lfloor \frac{\text{op\_word\_count\_reg} - 1}{4} \right\rfloor \right)$$

#### 4. Fractional `TKEEP` Mask Generator
On the final output cycle (`output_last_cycle_reg = 1`), combinational priority logic translates the remaining byte count (`last_cycle_offset_reg`) into valid byte qualifiers:
* 1 byte remaining: `TKEEP = 4'b0001`
* 2 bytes remaining: `TKEEP = 4'b0011`
* 3 bytes remaining: `TKEEP = 4'b0111`
* 4 bytes remaining: `TKEEP = 4'b1111`

## 3. S2MM Write Engine Microarchitecture (`axi_dma_wr.v`)

The S2MM Write Engine coordinates receiving unstructured AXI-Stream beats, segmenting them into legal memory bursts, generating sparse byte strobes, and committing them to physical memory.

### 3.1 The 5-State Master FSM Architecture

The write datapath is governed by a unified 5-state finite state machine (`state_reg` in [`axi_dma_wr.v`](file:///c:/Users/admin/OneDrive/Desktop/axi_vip_project/axi4_rtl/axi_master_rtl/axi_dma_wr.v)). In addition to nominal streaming, it incorporates dedicated hardware states for early termination and buffer overrun safety:

```
                            +--------------------------+
                            |     STATE_IDLE (3'd0)    |<-------------------+
                            |  Await write descriptor  |                    |
                            +-------------+------------+                    |
                                          |                                 |
                                          v                                 |
                            +--------------------------+                    |
                            |    STATE_START (3'd1)    |<----+              |
                            |  Compute 4KB clamp & AW  |     | Multi-burst  |
                            +-------------+------------+     | loop         |
                                          |                  |              |
                                          v                  |              |
                            +--------------------------+     |              |
  Early Stream              |    STATE_WRITE (3'd2)    |-----+              |
  Termination               |  Stream beats into FIFO  |                    |
  +-------------------------+-------------+------------+                    |
  |                                       |                                 |
  |                                       | Excess Stream                   |
  |                                       | Overrun                         |
  v                                       v                                 |
+--------------------------+    +--------------------------+                |
| STATE_FINISH_BURST (3'd3)|    |  STATE_DROP_DATA (3'd4)  |                |
| Zero-pad dummy beats     |    |  Drain excess beats      |                |
| (WSTRB = 0000)           |    |  until TLAST             |                |
+-------------+------------+    +-------------+------------+                |
              |                               |                             |
              +-------------------------------+-----------------------------+
```

#### State Definitions & Functional Behavior

1. **`STATE_IDLE` (3'd0):**
   - Monitors `s_axis_write_desc_valid`.
   - Checks credit availability in the `status_fifo` to guarantee that memory response tracking will not overflow.
   - Latches `addr_reg`, total buffer capacity `op_word_count_reg`, and metadata tag.
   - Transitions to `STATE_START`.

2. **`STATE_START` (3'd1):**
   - Evaluates remaining bytes against the 4KB page boundary (`4096 - addr_reg[11:0]`).
   - Clamps the burst length to a maximum of 16 beats.
   - Drives `m_axi_awaddr`, `m_axi_awlen`, and asserts `m_axi_awvalid`.
   - Upon `m_axi_awready` handshake, records burst parameters into `status_fifo` and transitions to `STATE_WRITE`.

3. **`STATE_WRITE` (3'd2):**
   - Directs incoming stream beats (`s_axis_write_data_*`) through the write barrel shifter and into the 2-cycle W-channel skid buffer.
   - Generates sparse `WSTRB` qualifiers on beat 0 and the final beat.
   - Decrements `op_word_count_reg` and `output_cycle_count_reg`.
   - **Nominal Transition:** When the active burst completes, it loops back to `STATE_START` (if more bursts are needed) or `STATE_IDLE` (if transfer is complete).
   - **Corner Scenario 1 Transition:** If the stream source asserts `TLAST` prematurely while an AXI burst is in-flight, it transitions to `STATE_FINISH_BURST`.
   - **Corner Scenario 2 Transition:** If the stream payload exceeds descriptor buffer capacity (`op_word_count == 0` without `TLAST`), it transitions to `STATE_DROP_DATA`.

4. **`STATE_FINISH_BURST` (3'd3) [Corner Scenario: Early Stream Termination]:**
   - Under AXI4 protocol rules, once an `AWLEN` burst length has been committed on the AW channel, the master *must* deliver exactly $AWLEN + 1$ write beats.
   - If an ingress stream asserts `TLAST` early, the DMA cannot simply abort the burst without causing bus deadlock.
   - **Hardware Mechanism:** The DMA transitions to `STATE_FINISH_BURST` and injects dummy padding beats into the W-channel output FIFO with write strobes forced to zero (`m_axi_wstrb = 4'b0000`).
   - This completes the committed AXI burst legally without modifying memory words.

5. **`STATE_DROP_DATA` (3'd4) [Corner Scenario: Buffer Overrun & Packet Draining]:**
   - If an incoming streaming packet is larger than the programmed descriptor buffer length ($L_{stream} > L_{buffer}$), continuing to write would corrupt adjacent memory buffers.
   - **Hardware Mechanism:** The DMA terminates memory write generation immediately at $L_{buffer}$ bytes. It enters `STATE_DROP_DATA`, holding memory write `WVALID` low while holding stream `s_axis_write_data_tready = 1`.
   - It drains and discards all excess stream beats until `TLAST` handshakes, ensuring the streaming FIFO is clean and ready for the next packet.

### 3.2 Microarchitectural Building Blocks in `axi_dma_wr.v`

#### 1. 2-Cycle Distributed RAM Skid Buffer (`out_fifo`)
Located in [`axi_dma_wr.v` lines 916-965](file:///c:/Users/admin/OneDrive/Desktop/axi_vip_project/axi4_rtl/axi_master_rtl/axi_dma_wr.v#L916-L965), this block isolates the streaming interface from AXI memory backpressure:
* **Storage Array:** Dual-port distributed RAM holding `out_fifo_wdata`, `out_fifo_wstrb`, and `out_fifo_wlast`.
* **Flow Control Watermarking:** When the difference between write and read pointers reaches $2^{\text{ADDR\_WIDTH}-1}$, `out_fifo_half_full_reg` asserts, throttling upstream stream acceptance.
* **Pipeline Latency Signature:** Because data is registered on FIFO write and re-registered on FIFO read onto `m_axi_wdata_reg`, handshakes on `s_axis_write_data_*` manifest on the physical memory bus with an exact **2-clock-cycle phase latency**.

#### 2. Write Barrel Shifter (`ENABLE_UNALIGNED = 1`)
Facilitates unaligned write transfers by steering stream bytes to arbitrary destination byte offsets:
* Shifts ingress stream bytes left by `offset_reg` ($dest\_addr[1:0]$).
* Merges residues across cycles using `save_axis_tdata_reg`.
* Aligns stream byte 0 to physical memory lane $dest\_addr[1:0]$.

#### 3. Sparse Write Strobe (`WSTRB`) Combinational Synthesis
The write controller synthesizes individual byte enable bits for every beat:
* **Beat 0 (Unaligned Prefix Mask):**
  $$WSTRB[0][b] = 1 \iff b \ge (AWADDR \ \& \ 3)$$
* **Intermediate Beats:** Full strobe assertion (`WSTRB = 4'b1111`).
* **Final Beat (Partial Tail Mask):**
  $$WSTRB[N-1][b] = 1 \iff b \le \left( (AWADDR + L_{xfer} - 1) \ \& \ 3 \right)$$

#### 4. Status Tracking FIFO (`status_fifo`)
A multi-entry hardware ring buffer storing descriptor parameters for issued write bursts:
* Stores `status_fifo_len`, `status_fifo_tag`, and error flags.
* Enables decoupling between write data transmission and write response collection (`BVALID`).
* Pulses `m_axis_write_desc_status_valid` only after the memory slave confirms write commit via `BVALID && BREADY`.

## 4. Hardware Microarchitecture Summary Matrix

The following matrix summarizes the sequential storage and combinational datapaths across both DMA sub-engines:

| Hardware Block | Engine | Circuit Type | RTL Implementation Details |
| :--- | :---: | :--- | :--- |
| **AR Generator FSM** | MM2S | 2-State Sequential | `AXI_STATE_IDLE`, `AXI_STATE_START`; issues AR bursts. |
| **AXIS Assembler FSM**| MM2S | 2-State Sequential | `AXIS_STATE_IDLE`, `AXIS_STATE_READ`; gathers R beats, drives AXIS. |
| **Command Pipeline FIFO**| MM2S | Registered FIFO | Decouples AR address prefetching from AXIS stream draining. |
| **Read Barrel Shifter** | MM2S | Multi-stage Crossbar| Shifts 32-bit memory words by 0-3 bytes; uses `save_axis_tdata_reg`. |
| **Bubble Discard Logic**| MM2S | Sequential Strobe | `bubble_cycle_reg` suppresses stream valid on unaligned cycle 0. |
| **Read 4KB Boundary Clamp**| MM2S | Combinational Adder | `bytes_to_boundary = 4096 - addr[11:0]`; clamps `ARLEN <= 15`. |
| **Write Master FSM** | S2MM | 5-State Sequential | `IDLE`, `START`, `WRITE`, `FINISH_BURST` (padding), `DROP_DATA` (excess). |
| **W-Channel Skid Buffer**| S2MM | Distributed RAM FIFO| 2-cycle latency pipeline; half-full watermark backpressure. |
| **Write Barrel Shifter**| S2MM | Multi-stage Crossbar| Steers stream bytes left by `dest_addr[1:0]`. |
| **Sparse WSTRB Generator**| S2MM | Combinational Mask | Synthesizes prefix mask (beat 0) and tail mask (final beat). |
| **Early Term Padder** | S2MM | Sequential Datapath | Injects zero-strobe beats (`WSTRB = 4'b0000`) in `STATE_FINISH_BURST`. |
| **Excess Data Drainer** | S2MM | Flow Control Gate | Keeps `TREADY = 1`, suppresses memory writes in `STATE_DROP_DATA`. |
| **Status Response FIFO**| S2MM | Ring Buffer FIFO | Correlates asynchronous `BVALID` responses with descriptor tags. |
