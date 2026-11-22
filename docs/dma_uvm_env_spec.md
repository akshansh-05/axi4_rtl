# AXI4 DMA Subsystem: UVM Verification Environment Specification

## 1. Executive Verification Overview

The primary verification objective for the **AMBA AXI4 Direct Memory Access (DMA) Subsystem** is to ensure 100% protocol compliance, functional correctness, and corner-case resilience across full-duplex, multi-channel data transfers.

The design under test (`axi_dma`) bridges high-speed packet-oriented streaming peripherals (**AMBA AXI4-Stream**) and memory-mapped interconnects (**AMBA AXI4-Full**). Verification must account for complex dynamic behaviors:
* **Unaligned Byte Steering:** Arbitrary byte alignments on both memory addresses and stream packets requiring multi-stage barrel shifting.
* **4KB Page Boundary Splitting:** Automatic hardware burst segmentation per ARM AMBA Section A3.4.1.
* **Asynchronous Multi-Channel Traffic:** Decoupled MM2S (Memory-to-Stream) read operations and S2MM (Stream-to-Memory) write operations executing concurrently.
* **Safety & Corner-Case Scenarios:** Premature stream termination (zero-padded bursts) and buffer capacity overruns (packet draining without memory corruption).

To verify these capabilities rigorously, an industrial-grade **IEEE 1800.2 UVM (Universal Verification Methodology)** verification environment was developed. This environment integrates **6 modular UVM agents** sourced from **3 specialized Verification IP (VIP)** suites, a **predictive dual-path scoreboard**, and a **4-tier functional coverage model**.

```
+---------------------------------------------------------------------------------------------------------+
|                                        UVM Verification Environment                                      |
|                                            (dma_subsystem_env)                                          |
|                                                                                                         |
|  +---------------------------+                                             +-------------------------+  |
|  |     dma_desc_vip          |                                             |        axis_vip         |  |
|  |  +---------------------+  |                                             |  +-------------------+  |  |
|  |  |  dma_rd_desc_agent  |  |                                             |  |   axis_rd_agent   |  |  |
|  |  |   (Active Master)   |  |                                             |  |   (Active Slave)  |  |  |
|  |  +----------+----------+  |                                             |  +---------+---------+  |  |
|  +-------------|-------------+                                             +------------|------------+  |
|                |                                                                        |               |
|                v (s_axis_read_desc_*)                                                   ^ (m_axis_rd_*) |
|  +--------------------------------------------------------------------------------------|------------+  |
|  |                                     DESIGN UNDER TEST (axi_dma)                      |            |  |
|  |                                                                                      |            |  |
|  |    MM2S Read Engine (axi_dma_rd) ----------------------------------------------------+            |  |
|  |       * AXI AR Burst Generator    * Read Barrel Shifter    * 4KB Boundary Clamp                   |  |
|  |                                                                                                   |  |
|  |    S2MM Write Engine (axi_dma_wr) <--------------------------------------------------+            |  |
|  |       * 2-Cycle Skid Buffer FIFO  * Write Barrel Shifter   * 5-State Master FSM      |            |  |
|  +--------------------------------------------------------------------------------------|------------+  |
|                ^ (m_axi_ar* / m_axi_r*)                                                 | (s_axis_wr) |
|                |                                                                        |               |
|  +-------------|-------------+       +-----------------------------+       +------------|------------+  |
|  |        axi_mm_vip         |       |      Central Analysis       |       |        axis_vip         |  |
|  |  +----------+----------+  |       |  +-----------------------+  |       |  +---------+---------+  |  |
|  |  |    axi_rd_agent     |  |       |  |     dma_coverage      |  |       |  |   axis_wr_agent   |  |  |
|  |  | (Slave/Monitor)     |  |------>|  |  (4-Tier Model)       |  |<------|  |   (Active Master) |  |  |
|  |  +---------------------+  |       |  +-----------------------+  |       |  +-------------------+  |  |
|  |  |    axi_wr_agent     |  |       |  |    dma_scoreboard     |  |       |  | dma_wr_desc_agent |  |  |
|  |  | (Slave/Monitor)     |  |------>|  | (Predictive Checker)  |  |<------|  |   (Active Master) |  |  |
|  |  +---------------------+  |       |  +-----------------------+  |       |  +-------------------+  |  |
|  +---------------------------+       +-----------------------------+       +-------------------------+  |
+---------------------------------------------------------------------------------------------------------+
```

## 2. The 3 Verification IP (VIP) Suites

The testbench architecture leverages three independent, reusable VIP packages, each responsible for a distinct protocol boundary:

### 2.1 AXI4-Full Memory-Mapped VIP (`axi_mm_vip`)
Encapsulates the standard ARM AMBA AXI4 protocol (IHI0022H). It interfaces to the master read and write channels of the DMA.
* **Component Inventory:** `axi_wr_agent`, `axi_rd_agent`, `axi_sequencer`, `axi_seq_item`, `axi_agent_config`.
* **Verification Scope:** 
  * Full channel decoupling: Handles Address Write (AW), Write Data (W), and Write Response (B) concurrently with Address Read (AR) and Read Data (R).
  * Slave Memory Modeling: Houses an internal associative memory array that returns requested read data and commits written bytes with sparse strobes (`WSTRB`).
  * Reactive Flow Control: Dynamically injects randomized slave backpressure by controlling `ARREADY`, `AWREADY`, and `WREADY` handshake delays.

### 2.2 DMA Descriptor VIP (`dma_desc_vip`)
Models host-level software interaction with the DMA hardware descriptor queues.
* **Component Inventory:** `dma_rd_desc_agent`, `dma_wr_desc_agent`, `dma_desc_sequencer`, `dma_desc_seq_item`, `dma_desc_agent_config`.
* **Verification Scope:**
  * Issues transfer command descriptors containing source/destination addresses, byte lengths ($1\text{ B}$ to $1\text{ MB}$), tags, IDs, and user attributes.
  * Synchronizes command valid/ready handshakes.
  * Monitors asynchronous completion status pulses (`m_axis_*_desc_status_*`), verifying that status tokens match issued tags and report valid transferred lengths.

### 2.3 AXI4-Stream VIP (`axis_vip`)
Encapsulates high-speed streaming peripheral transactions per ARM AMBA AXI4-Stream (IHI0051B).
* **Component Inventory:** `axis_wr_agent`, `axis_rd_agent`, `axis_sequencer`, `axis_seq_item`, `axis_agent_config`.
* **Verification Scope:**
  * Ingress Stream Generation (Write Path): Emits structured or randomized data packets with byte qualifiers (`TKEEP`) and framing delimiters (`TLAST`).
  * Egress Stream Reception (Read Path): Captures serialized byte streams emitted by the DMA read engine.
  * Backpressure Injection: Drives `TREADY` with zero latency, fixed stalls, or randomized burst backpressure to test DUT skid-buffering resilience.

## 3. The 6 UVM Verification Agents

The top-level environment class (`dma_subsystem_env`) instantiates exactly six agents to govern and monitor every pin of the DUT:

| Agent Name | VIP Family | Primary Interface / Pins | Operating Role & Default Mode | Primary Verification Function |
| :--- | :--- | :--- | :---: | :--- |
| **`axi_wr_agent`** | `axi_mm_vip` | `m_axi_aw*`, `m_axi_w*`, `m_axi_b*` | Dual-Mode (Active Slave / Passive Monitor) | Captures memory write bursts, validates `AWLEN`/`WSTRB`, and models memory write responses. |
| **`axi_rd_agent`** | `axi_mm_vip` | `m_axi_ar*`, `m_axi_r*` | Dual-Mode (Active Slave / Passive Monitor) | Captures memory read requests, validates `ARLEN`/`ARADDR`, and supplies memory read data. |
| **`dma_rd_desc_agent`** | `dma_desc_vip` | `s_axis_read_desc_*`, `m_axis_read_desc_status_*` | Active Master | Drives read transfer requests into the MM2S engine; monitors completion status. |
| **`dma_wr_desc_agent`** | `dma_desc_vip` | `s_axis_write_desc_*`, `m_axis_write_desc_status_*` | Active Master | Drives write buffer allocations into the S2MM engine; monitors completion status. |
| **`axis_wr_agent`** | `axis_vip` | `s_axis_write_data_*` | Active Master (Stream Producer) | Drives packetized ingress stream data into the S2MM write datapath. |
| **`axis_rd_agent`** | `axis_vip` | `m_axis_read_data_*` | Active Slave (Stream Consumer) | Sinks egress stream data from the MM2S read datapath; injects `TREADY` backpressure. |

### 3.1 Agent Internal Structure
Following standard UVM methodology, each agent encapsulates:
* **Sequencer:** Holds and arbitrates sequence item transactions.
* **Driver:** Converts high-level sequence item transactions into cycle-accurate pin toggles on the virtual interface.
* **Monitor:** Passively samples pin transitions, reconstructs protocol transactions, performs protocol assertion checks, and broadcasts transactions via an analysis port (`uvm_analysis_port`).

## 4. Environment Configuration & Operational Modes

Agent roles and behaviors are configured centrally during the `build_phase` of `dma_base_test` using configuration objects passed through `uvm_config_db`.

The environment supports two major operational topologies:

```
                           +-----------------------------------------------+
                           |          Testbench Operating Topologies       |
                           +-----------------------------------------------+
                                                  |
                 +--------------------------------+--------------------------------+
                 |                                                                 |
                 v                                                                 v
+------------------------------------+           +------------------------------------+
|     1. DMA Standalone Mode         |           |       2. Subsystem Mode            |
|        (+DMA_STANDALONE)           |           |          (+SUBSYSTEM)              |
+------------------------------------+           +------------------------------------+
| * Focus: Focused DUT IP testing.   |           | * Focus: Full chip SoC integration.|
| * AXI-MM VIPs: ACTIVE SLAVES.      |           | * RTL Memory: axi_ram active on bus|
| * The VIP acts as memory, serving  |           | * AXI-MM VIPs: PASSIVE MONITORS.   |
|   read data & absorbing writes.    |           | * VIP monitors memory transactions |
| * Fast simulation, fine control    |           |   executed against physical RTL    |
|   over slave ready delays.         |           |   memory block.                    |
+------------------------------------+           +------------------------------------+
```

### 4.1 Configuration Knobs (`dma_base_test`)
* `is_standalone_dma`: Selects between Standalone mode (`1'b1`) and Subsystem mode (`1'b0`).
* `ready_always_high`: In `axis_rd_agent`, configures whether the downstream stream receiver is ideal (zero backpressure) or applies randomized wait states.
* `is_active`: Configures whether an agent's sequencer and driver are instantiated (`UVM_ACTIVE`) or only its monitor is enabled (`UVM_PASSIVE`).

## 5. Architectural Role of the Scoreboard (`dma_scoreboard`)

Rather than performing simple point-to-point comparisons, `dma_scoreboard` serves as an **end-to-end predictive self-checking reference model**:

```
+-------------------------------------------------------------------------------------------------+
|                                    dma_scoreboard Architecture                                  |
|                                                                                                 |
|  MM2S Path:                                                                                     |
|  [dma_rd_desc] ---> Record Command (Addr, Len, Tag)                                             |
|                             |                                                                   |
|  [axi_rd_mon]  ---> Predict Egress Stream Beats (Barrel Shift, Offset Alignment, TKEEP Masks)    |
|                             |                                                                   |
|  [axis_rd_mon] ---> Compare Actual vs. Predicted Beats Byte-for-Byte                            |
|                                                                                                 |
|  S2MM Path:                                                                                     |
|  [dma_wr_desc] ---> Record Buffer Allocation (Addr, Capacity, Tag)                              |
|                             |                                                                   |
|  [axis_wr_mon] ---> Buffer Stream Bytes; Predict AXI Memory Bursts (AWADDR, AWLEN, WDATA, WSTRB)|
|                             |                                                                   |
|  [axi_wr_mon]  ---> Compare Actual vs. Predicted Memory Writes Byte-for-Byte                     |
|                                                                                                 |
|  Completion:                                                                                    |
|  [desc_status] ---> Correlate Asynchronous Status Tokens with Issued Descriptors                |
+-------------------------------------------------------------------------------------------------+
```

### 5.1 Scoreboard Verification Responsibilities
1. **MM2S Data Integrity:** Tracks memory read beats captured by `axi_rd_agent`, calculates expected unaligned byte barrel shifting and `TKEEP` qualifier generation, and matches them against egress packets captured by `axis_rd_agent`.
2. **S2MM Data Integrity:** Collects ingress stream beats from `axis_wr_agent`, tracks the 2-cycle skid buffer delay, predicts memory burst segmentation and sparse `WSTRB` generation, and compares against actual memory write transactions observed by `axi_wr_agent`.
3. **Corner-Case Tracking:**
   * **Early Termination:** Detects premature `TLAST` and confirms that the DUT zero-pads remaining committed burst beats with `WSTRB = 4'b0000` without triggering false data mismatches.
   * **Buffer Overrun:** Detects when incoming stream length exceeds allocated descriptor buffer capacity and verifies that excess beats are discarded cleanly while memory remains intact.
4. **Zero-Leakage Audit:** In the UVM `check_phase`, verifies that all descriptor queues, transaction shadow buffers, and status tables are completely empty, proving zero data loss or hung transfers.

## 6. Architectural Role of the Coverage Model (`dma_coverage`)

The functional coverage model (`dma_coverage`) implements a multi-tier verification metric architecture to ensure that every corner case, protocol combination, and transfer geometry has been stimulated and verified:

```
+----------------------------------------------------------------------------------------+
|                               4-Tier Functional Coverage Model                         |
+----------------------------------------------------------------------------------------+
|  Tier 1: Protocol Handshake Coverage                                                   |
|  * AXI-MM Address / Data / Response Valid-Ready Handshake Latencies                    |
|  * AXI-Stream TVALID / TREADY Stall Delays and Backpressure Hold Times                 |
+----------------------------------------------------------------------------------------+
|  Tier 2: Transfer Geometry Coverage                                                    |
|  * Burst Lengths: Single beat (1) through maximum burst (16/256)                       |
|  * Transfer Sizes: 1 Byte to 64 KB                                                     |
|  * Memory Alignments: Aligned (offset 0) and unaligned offsets (1, 2, 3)               |
|  * 4KB Page Boundaries: Near-boundary starts, exact crossings, multi-burst splits       |
+----------------------------------------------------------------------------------------+
|  Tier 3: Corner-Case & Architectural Feature Coverage                                  |
|  * Early Stream Termination (TLAST before descriptor length)                           |
|  * Excess Stream Payload (Stream length > descriptor buffer capacity)                  |
|  * Back-to-Back Descriptor Queuing (Zero-bubble command re-arming)                     |
|  * Heavy Receiver Backpressure (TREADY deassertion bursts)                             |
+----------------------------------------------------------------------------------------+
|  Tier 4: Cross-Coverage Metrics                                                        |
|  * Transfer Length x Address Offset x Slave Backpressure                               |
|  * Burst Type x 4KB Boundary Proximity x Stream Backpressure                           |
+----------------------------------------------------------------------------------------+
```

* Monitors analysis ports from all 6 agents simultaneously.
* Fully decoupled from scoreboard checking logic to prevent coverage sampling from masking functional bugs.
* Generates unified metric databases merged and inspected via **Cadence IMC (Integrated Metrics Center)**.

## 7. Verification Dataflow & Execution Lifecycle

During a test run, the environment components collaborate across UVM phases:

1. **`build_phase`:** Instantiates the 6 agents, `dma_coverage`, and `dma_scoreboard`. Fetches virtual interfaces from `uvm_config_db`.
2. **`connect_phase`:** Binds agent monitor analysis ports to the analysis exports of `dma_coverage` and `dma_scoreboard`.
3. **`run_phase`:** Test sequences execute on the sequencers:
   * Descriptor sequences arm the DMA channels.
   * Stream sequences drive ingress packets or consume egress packets.
   * Monitors passively observe physical bus activity and stream transactions into the scoreboard and coverage model concurrently.
4. **`check_phase`:** The scoreboard asserts that all expected transactions have arrived, queues are empty, and zero mismatches occurred.
5. **`report_phase`:** Summarizes final packet counts, scoreboard matches, and functional coverage percentages.

## 8. Summary of Verification Architecture Documentation

This specification provides the high-level structural framework of the DMA verification environment. For detailed component-level specifications, refer to the companion documents:

* **Hardware Microarchitecture:** `docs/dma_rtl_microarch_spec.md`
* **Testcase Regression Suite:** `uvm/tests/dma_testcases_doc.md`
* **Scoreboard Predictive Algorithms:** `uvm/env/dma_scoreboard_doc.md`
* **VIP Internals & Protocols:** `uvm/vip/VERIFICATION_PLATFORM_AND_VIP_ARCHITECTURE.md`
* **ASIC Synthesis Results:** `asic/axi_dma_synthesis_report.md`
