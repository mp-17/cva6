// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Authors: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
//          Nils Wistoff <nwistoff@iis.ee.ethz.ch>

// Package defining the accelerator interface as used by Ara + CVA6

package acc_pkg;

  // ----------------------
  // Accelerator MMU Interface
  // ----------------------

  // Accelerator MMU interface
  typedef struct packed {
    logic                   acc_mmu_misaligned_ex;
    logic                   acc_mmu_req;
    logic [riscv::VLEN-1:0] acc_mmu_vaddr;
    logic                   acc_mmu_is_store;
  } acc_mmu_req_t;

  typedef struct packed {
    logic                   acc_mmu_dtlb_hit;
    logic [riscv::PPNW-1:0] acc_mmu_dtlb_ppn;
    logic                   acc_mmu_valid;
    logic [riscv::PLEN-1:0] acc_mmu_paddr;
    exception_t             acc_mmu_exception;
  } acc_mmu_resp_t;

  // ----------------------
  // Accelerator instruction/memory
  // ----------------------

  typedef struct packed {
    logic                                 req_valid;
    logic                                 resp_ready;
    riscv::instruction_t                  insn;
    riscv::xlen_t                         rs1;
    riscv::xlen_t                         rs2;
    fpnew_pkg::roundmode_e                frm;
    logic [ariane_pkg::TRANS_ID_BITS-1:0] trans_id;
    logic                                 store_pending;
    // Invalidation interface
    logic                                 acc_cons_en;
    logic                                 inval_ready;
  } accelerator_req_t;

  typedef struct packed {
    logic                                 req_ready;
    logic                                 resp_valid;
    riscv::xlen_t                         result;
    logic [ariane_pkg::TRANS_ID_BITS-1:0] trans_id;
    ariane_pkg::exception_t               exception;

    // Metadata
    logic                                 store_pending;
    logic                                 store_complete;
    logic                                 load_complete;
    logic [4:0]                           fflags;
    logic                                 fflags_valid;
    // Invalidation interface
    logic                                 inval_valid;
    logic [63:0]                          inval_addr;
  } accelerator_resp_t;

  // ----------------------
  // Accelerator Interface
  // ----------------------

  typedef struct packed {
    // Insn/mem
    accelerator_req_t                     acc_req;
    // MMU
    logic                                 acc_mmu_en;
    acc_mmu_resp_t                        acc_mmu_resp;
  } cva6_to_acc_t;

  typedef struct packed {
    // Insn/mem
    accelerator_resp_t                    acc_resp;
    // MMU
    acc_mmu_req_t                         acc_mmu_req;
  } acc_to_cva6_t;

endpackage
