// Copyright 2017-2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19.03.2017
// Description: Ariane Top-level module


module ariane import ariane_pkg::*; #(
  parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
  parameter bit IsRVFI = bit'(0),
  parameter type rvfi_probes_t = logic,
  parameter int unsigned AxiAddrWidth = ariane_axi::AddrWidth,
  parameter int unsigned AxiDataWidth = ariane_axi::DataWidth,
  parameter int unsigned AxiIdWidth   = ariane_axi::IdWidth,
  parameter type axi_ar_chan_t = ariane_axi::ar_chan_t,
  parameter type axi_aw_chan_t = ariane_axi::aw_chan_t,
  parameter type axi_w_chan_t  = ariane_axi::w_chan_t,
  parameter type noc_req_t = ariane_axi::req_t,
  parameter type noc_resp_t = ariane_axi::resp_t
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [riscv::VLEN-1:0]       boot_addr_i,  // reset boot address
  input  logic [riscv::XLEN-1:0]       hart_id_i,    // hart id in a multicore environment (reflected in a CSR)

  // Interrupt inputs
  input  logic [1:0]                   irq_i,        // level sensitive IR lines, mip & sip (async)
  input  logic                         ipi_i,        // inter-processor interrupts (async)
  // Timer facilities
  input  logic                         time_irq_i,   // timer interrupt in (async)
  input  logic                         debug_req_i,  // debug request (async)
`ifdef ARIANE_ACCELERATOR_PORT
  // CSR to accelerator
  output logic                            en_ld_st_translation_o,

  // MMU interface with accelerator
  input  exception_t                      acc_mmu_misaligned_ex_i,
  input  logic                            acc_mmu_req_i,        // request address translation
  input  logic [riscv::VLEN-1:0]          acc_mmu_vaddr_i,      // virtual address in
  input  logic                            acc_mmu_is_store_i,   // the translation is requested by a store
  // if we need to walk the page table we can't grant in the same cycle
  // Cycle 0
  output logic                            acc_mmu_dtlb_hit_o,   // sent in the same cycle as the request if translation hits in the DTLB
  output logic [riscv::PPNW-1:0]          acc_mmu_dtlb_ppn_o,   // ppn (send same cycle as hit)
  // Cycle 1
  output logic                            acc_mmu_valid_o,      // translation is valid
  output logic [riscv::PLEN-1:0]          acc_mmu_paddr_o,      // translated address
  output exception_t                      acc_mmu_exception_o,  // address translation threw an exception
`endif
  // RISC-V formal interface port (`rvfi`):
  // Can be left open when formal tracing is not needed.
  output rvfi_probes_t rvfi_probes_o,
  // memory side
  output noc_req_t                     noc_req_o,
  input  noc_resp_t                    noc_resp_i
);

  cvxif_pkg::cvxif_req_t  cvxif_req;
  cvxif_pkg::cvxif_resp_t cvxif_resp;

  cva6 #(
    .CVA6Cfg ( CVA6Cfg ),
    .IsRVFI ( IsRVFI ),
    .rvfi_probes_t ( rvfi_probes_t ),
    .axi_ar_chan_t (axi_ar_chan_t),
    .axi_aw_chan_t (axi_aw_chan_t),
    .axi_w_chan_t (axi_w_chan_t),
    .noc_req_t (noc_req_t),
    .noc_resp_t (noc_resp_t)
  ) i_cva6 (
    .clk_i                ( clk_i                     ),
    .rst_ni               ( rst_ni                    ),
    .boot_addr_i          ( boot_addr_i               ),
    .hart_id_i            ( hart_id_i                 ),
    .irq_i                ( irq_i                     ),
    .ipi_i                ( ipi_i                     ),
    .time_irq_i           ( time_irq_i                ),
    .debug_req_i          ( debug_req_i               ),
    .clic_irq_valid_i     ( 1'b0                      ),
    .clic_irq_id_i        ( '0                        ),
    .clic_irq_level_i     ( '0                        ),
    .clic_irq_priv_i      ( '0                        ),
    .clic_irq_shv_i       ( 1'b0                      ),
    .clic_irq_ready_o     (                           ),
    .clic_kill_req_i      ( 1'b0                      ),
    .clic_kill_ack_o      (                           ),
    .rvfi_probes_o        ( rvfi_probes_o             ),
`ifdef ARIANE_ACCELERATOR_PORT
    .en_ld_st_translation_o (en_ld_st_translation_o ),
    .acc_mmu_misaligned_ex_i(mmu_misaligned_ex_i),
    .acc_mmu_req_i          (mmu_req_i          ),
    .acc_mmu_vaddr_i        (mmu_vaddr_i        ),
    .acc_mmu_is_store_i     (mmu_is_store_i     ),
    .acc_mmu_dtlb_hit_o     (mmu_dtlb_hit_o     ),
    .acc_mmu_dtlb_ppn_o     (mmu_dtlb_ppn_o     ),
    .acc_mmu_valid_o        (mmu_valid_o        ),
    .acc_mmu_paddr_o        (mmu_paddr_o        ),
    .acc_mmu_exception_o    (mmu_exception_o    ),
`endif

    .cvxif_req_o          ( cvxif_req                 ),
    .cvxif_resp_i         ( cvxif_resp                ),
    .noc_req_o            ( noc_req_o                 ),
    .noc_resp_i           ( noc_resp_i                )
  );

  if (CVA6Cfg.CvxifEn) begin : gen_example_coprocessor
    cvxif_example_coprocessor i_cvxif_coprocessor (
      .clk_i                ( clk_i                          ),
      .rst_ni               ( rst_ni                         ),
      .cvxif_req_i          ( cvxif_req                      ),
      .cvxif_resp_o         ( cvxif_resp                     )
    );
  end

endmodule // ariane
