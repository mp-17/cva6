// Copyright 2018 ETH Zurich and University of Bologna.
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
// Date: 19.04.2017
// Description: Load Store Unit, handles address calculation and memory interface signals


module load_store_unit import ariane_pkg::*; #(
    parameter int unsigned ASID_WIDTH = 1,
    parameter ariane_pkg::ariane_cfg_t ArianeCfg = ariane_pkg::ArianeDefaultConfig
)(
    input  logic                     clk_i,
    input  logic                     rst_ni,
    input  logic                     flush_i,
    output logic                     no_st_pending_o,
    input  logic                     amo_valid_commit_i,

    input  fu_data_t                 fu_data_i,
    output logic                     lsu_ready_o,              // FU is ready e.g. not busy
    input  logic                     lsu_valid_i,              // Input is valid

    output logic [TRANS_ID_BITS-1:0] load_trans_id_o,          // ID of scoreboard entry at which to write back
    output riscv::xlen_t             load_result_o,
    output logic                     load_valid_o,
    output exception_t               load_exception_o,         // to WB, signal exception status LD exception

    output logic [TRANS_ID_BITS-1:0] store_trans_id_o,         // ID of scoreboard entry at which to write back
    output riscv::xlen_t             store_result_o,
    output logic                     store_valid_o,
    output exception_t               store_exception_o,        // to WB, signal exception status ST exception

    input  logic                     commit_i,                 // commit the pending store
    output logic                     commit_ready_o,           // commit queue is ready to accept another commit request
    input  logic [TRANS_ID_BITS-1:0] commit_tran_id_i,

    input  logic                     enable_translation_i,     // enable virtual memory translation
    input  logic                     en_ld_st_translation_i,   // enable virtual memory translation for load/stores

    // MMU interface with accelerator
    input  exception_t               acc_mmu_misaligned_ex_i,
    input  logic                     acc_mmu_req_i,        // request address translation
    input  logic [riscv::VLEN-1:0]   acc_mmu_vaddr_i,      // virtual address in
    input  logic                     acc_mmu_is_store_i,   // the translation is requested by a store
    // if we need to walk the page table we can't grant in the same cycle
    // Cycle 0
    output logic                     acc_mmu_dtlb_hit_o,   // sent in the same cycle as the request if translation hits in the DTLB
    output logic [riscv::PPNW-1:0]   acc_mmu_dtlb_ppn_o,   // ppn (send same cycle as hit)
    // Cycle 1
    output logic                     acc_mmu_valid_o,      // translation is valid
    output logic [riscv::PLEN-1:0]   acc_mmu_paddr_o,      // translated address
    output exception_t               acc_mmu_exception_o,  // address translation threw an exception

    // icache translation requests
    input  icache_areq_o_t           icache_areq_i,
    output icache_areq_i_t           icache_areq_o,

    input  riscv::priv_lvl_t         priv_lvl_i,               // From CSR register file
    input  riscv::priv_lvl_t         ld_st_priv_lvl_i,         // From CSR register file
    input  logic                     sum_i,                    // From CSR register file
    input  logic                     mxr_i,                    // From CSR register file
    input  logic [riscv::PPNW-1:0]   satp_ppn_i,               // From CSR register file
    input  logic [ASID_WIDTH-1:0]    asid_i,                   // From CSR register file
    input  logic [ASID_WIDTH-1:0]    asid_to_be_flushed_i,
    input  logic [riscv::VLEN-1:0]   vaddr_to_be_flushed_i,
    input  logic                     flush_tlb_i,
    // Performance counters
    output logic                     itlb_miss_o,
    output logic                     dtlb_miss_o,

    // interface to dcache
    input  dcache_req_o_t [2:0]      dcache_req_ports_i,
    output dcache_req_i_t [2:0]      dcache_req_ports_o,
    input  logic                     dcache_wbuffer_empty_i,
    input  logic                     dcache_wbuffer_not_ni_i,
    // AMO interface
    output amo_req_t                 amo_req_o,
    input  amo_resp_t                amo_resp_i,
    // PMP
    input  riscv::pmpcfg_t [15:0]    pmpcfg_i,
    input  logic [15:0][riscv::PLEN-3:0] pmpaddr_i,

    //RVFI
    output [riscv::VLEN-1:0]         lsu_addr_o,
    output [riscv::PLEN-1:0]         mem_paddr_o,
    output [(riscv::XLEN/8)-1:0]     lsu_rmask_o,
    output [(riscv::XLEN/8)-1:0]     lsu_wmask_o,
    output [ariane_pkg::TRANS_ID_BITS-1:0] lsu_addr_trans_id_o
);

    // data is misaligned
    logic data_misaligned;
    // --------------------------------------
    // 1st register stage - (stall registers)
    // --------------------------------------
    // those are the signals which are always correct
    // e.g.: they keep the value in the stall case
    lsu_ctrl_t lsu_ctrl;

    logic      pop_st;
    logic      pop_ld;

    // ------------------------------
    // Address Generation Unit (AGU)
    // ------------------------------
    // virtual address as calculated by the AGU in the first cycle
    logic [riscv::VLEN-1:0]           vaddr_i;
    riscv::xlen_t                     vaddr_xlen;
    logic                             overflow;
    logic [(riscv::XLEN/8)-1:0]       be_i;

    assign vaddr_xlen = $unsigned($signed(fu_data_i.imm) + $signed(fu_data_i.operand_a));
    assign vaddr_i = vaddr_xlen[riscv::VLEN-1:0];
    // we work with SV39 or SV32, so if VM is enabled, check that all bits [XLEN-1:38] or [XLEN-1:31] are equal
    assign overflow = !((&vaddr_xlen[riscv::XLEN-1:riscv::SV-1]) == 1'b1 || (|vaddr_xlen[riscv::XLEN-1:riscv::SV-1]) == 1'b0);

    logic                     st_valid_i;
    logic                     ld_valid_i;
    logic                     ld_translation_req;
    logic                     cva6_st_translation_req, acc_st_translation_req, st_translation_req;
    logic [riscv::VLEN-1:0]   ld_vaddr;
    logic [riscv::VLEN-1:0]   st_vaddr;
    logic                     cva6_translation_req, acc_translation_req, translation_req;
    logic                     cva6_translation_valid, acc_translataion_valid, translation_valid;
    logic [riscv::VLEN-1:0]   cva6_mmu_vaddr, acc_mmu_vaddr, mmu_vaddr;
    logic [riscv::PLEN-1:0]   cva6_mmu_paddr, acc_mmu_paddr, mmu_paddr, mmu_vaddr_plen, fetch_vaddr_plen;
    exception_t               cva6_mmu_exception, acc_mmu_exception, mmu_exception;
    logic                     cva6_dtlb_hit, acc_dtlb_hit, dtlb_hit;
    logic [riscv::PPNW-1:0]   cva6_dtlb_ppn, acc_dtlb_ppn, dtlb_ppn;

    logic                     ld_valid;
    logic [TRANS_ID_BITS-1:0] ld_trans_id;
    riscv::xlen_t             ld_result;
    logic                     st_valid;
    logic [TRANS_ID_BITS-1:0] st_trans_id;
    riscv::xlen_t             st_result;

    logic [11:0]              page_offset;
    logic                     page_offset_matches;

    exception_t               cva6_misaligned_exception, acc_misaligned_exception, misaligned_exception;
    exception_t               ld_ex;
    exception_t               st_ex;

    // -------------------
    // MMU e.g.: TLBs/PTW
    // -------------------
    if (MMU_PRESENT && (riscv::XLEN == 64)) begin : gen_mmu_sv39
        mmu #(
            .INSTR_TLB_ENTRIES      ( ariane_pkg::INSTR_TLB_ENTRIES ),
            .DATA_TLB_ENTRIES       ( ariane_pkg::DATA_TLB_ENTRIES ),
            .ASID_WIDTH             ( ASID_WIDTH             ),
            .ArianeCfg              ( ArianeCfg              )
        ) i_cva6_mmu (
            // misaligned bypass
            .misaligned_ex_i        ( misaligned_exception   ),
            .lsu_is_store_i         ( st_translation_req     ),
            .lsu_req_i              ( translation_req        ),
            .lsu_vaddr_i            ( mmu_vaddr              ),
            .lsu_valid_o            ( translation_valid      ),
            .lsu_paddr_o            ( mmu_paddr              ),
            .lsu_exception_o        ( mmu_exception          ),
            .lsu_dtlb_hit_o         ( dtlb_hit               ), // send in the same cycle as the request
            .lsu_dtlb_ppn_o         ( dtlb_ppn               ), // send in the same cycle as the request
            // connecting PTW to D$ IF
            .req_port_i             ( dcache_req_ports_i [0] ),
            .req_port_o             ( dcache_req_ports_o [0] ),
            // icache address translation requests
            .icache_areq_i          ( icache_areq_i          ),
            .asid_to_be_flushed_i,
            .vaddr_to_be_flushed_i,
            .icache_areq_o          ( icache_areq_o          ),
            .pmpcfg_i,
            .pmpaddr_i,
            .*
        );
    end else if (MMU_PRESENT && (riscv::XLEN == 32)) begin : gen_mmu_sv32
        cva6_mmu_sv32 #(
            .INSTR_TLB_ENTRIES      ( ariane_pkg::INSTR_TLB_ENTRIES ),
            .DATA_TLB_ENTRIES       ( ariane_pkg::DATA_TLB_ENTRIES ),
            .ASID_WIDTH             ( ASID_WIDTH             ),
            .ArianeCfg              ( ArianeCfg              )
        ) i_cva6_mmu (
            // misaligned bypass
            .misaligned_ex_i        ( misaligned_exception   ),
            .lsu_is_store_i         ( st_translation_req     ),
            .lsu_req_i              ( translation_req        ),
            .lsu_vaddr_i            ( mmu_vaddr              ),
            .lsu_valid_o            ( translation_valid      ),
            .lsu_paddr_o            ( mmu_paddr              ),
            .lsu_exception_o        ( mmu_exception          ),
            .lsu_dtlb_hit_o         ( dtlb_hit               ), // send in the same cycle as the request
            .lsu_dtlb_ppn_o         ( dtlb_ppn               ), // send in the same cycle as the request
            // connecting PTW to D$ IF
            .req_port_i             ( dcache_req_ports_i [0] ),
            .req_port_o             ( dcache_req_ports_o [0] ),
            // icache address translation requests
            .icache_areq_i          ( icache_areq_i          ),
            .asid_to_be_flushed_i,
            .vaddr_to_be_flushed_i,
            .icache_areq_o          ( icache_areq_o          ),
            .pmpcfg_i,
            .pmpaddr_i,
            .*
        );
    end else begin : gen_no_mmu

        if (riscv::VLEN > riscv::PLEN) begin
          assign mmu_vaddr_plen = cva6_mmu_vaddr[riscv::PLEN-1:0];
          assign fetch_vaddr_plen = icache_areq_i.fetch_vaddr[riscv::PLEN-1:0];
        end else begin
          assign mmu_vaddr_plen = {{{riscv::PLEN-riscv::VLEN}{1'b0}}, cva6_mmu_vaddr};
          assign fetch_vaddr_plen = {{{riscv::PLEN-riscv::VLEN}{1'b0}}, icache_areq_i.fetch_vaddr};
        end

        assign  icache_areq_o.fetch_valid  = icache_areq_i.fetch_req;
        assign  icache_areq_o.fetch_paddr  = fetch_vaddr_plen;
        assign  icache_areq_o.fetch_exception      = '0;

        assign dcache_req_ports_o[0].address_index = '0;
        assign dcache_req_ports_o[0].address_tag   = '0;
        assign dcache_req_ports_o[0].data_wdata    = '0;
        assign dcache_req_ports_o[0].data_req      = 1'b0;
        assign dcache_req_ports_o[0].data_be       = '1;
        assign dcache_req_ports_o[0].data_size     = 2'b11;
        assign dcache_req_ports_o[0].data_we       = 1'b0;
        assign dcache_req_ports_o[0].kill_req      = '0;
        assign dcache_req_ports_o[0].tag_valid     = 1'b0;

        assign itlb_miss_o = 1'b0;
        assign dtlb_miss_o = 1'b0;
        assign cva6_dtlb_ppn    = mmu_vaddr_plen[riscv::PLEN-1:12];
        assign cva6_dtlb_hit    = 1'b1;

        assign cva6_mmu_exception = '0;

        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (~rst_ni) begin
                cva6_mmu_paddr         <= '0;
                cva6_translation_valid <= '0;
            end else begin
                cva6_mmu_paddr         <=  mmu_vaddr_plen;
                cva6_translation_valid <= cva6_translation_req;
            end
        end
    end

    // The MMU can be connected to CVA6 or the ACCELERATOR
    enum logic {CVA6, ACC} mmu_state_d, mmu_state_q;

    // Straightforward and slow-reactive MMU arbitration logic
    // This logic can be optimized to reduce answer latency and contention
    always_comb begin
      // Serve CVA6 and gate the accelerator by default
      // MMU input
      misaligned_exception   = cva6_misaligned_exception;
      st_translation_req     = cva6_st_translation_req;
      translation_req        = cva6_translation_req;
      mmu_vaddr              = cva6_mmu_vaddr;
      // MMU output
      cva6_translation_valid = translation_valid;
      cva6_mmu_paddr         = mmu_paddr;
      cva6_mmu_exception     = mmu_exception;
      cva6_dtlb_hit          = dtlb_hit;
      cva6_dtlb_ppn          = dtlb_ppn;
      acc_mmu_valid_o        = '0;
      acc_mmu_paddr_o        = '0;
      acc_mmu_exception_o    = '0;
      acc_mmu_dtlb_hit_o     = '0;
      acc_mmu_dtlb_ppn_o     = '0;

      unique case (mmu_state_q)
        CVA6: begin
          // Only the accelerator is requesting, and the MMU is not answering in this cycle
          if (acc_mmu_req_i && !cva6_translation_req) begin
            // Lock the MMU to the accelerator
            mmu_state_d = ACC;
          end
        end
        ACC: begin
          // MMU input
          misaligned_exception   = acc_mmu_misaligned_ex_i;
          st_translation_req     = acc_mmu_is_store_i;
          translation_req        = acc_mmu_req_i;
          mmu_vaddr              = acc_mmu_vaddr_i;
          // MMU output
          acc_mmu_valid_o        = translation_valid;
          acc_mmu_paddr_o        = mmu_paddr;
          acc_mmu_exception_o    = mmu_exception;
          acc_mmu_dtlb_hit_o     = dtlb_hit;
          acc_mmu_dtlb_ppn_o     = dtlb_ppn;
          cva6_translation_valid = '0;
          cva6_mmu_paddr         = '0;
          cva6_mmu_exception     = '0;
          cva6_dtlb_hit          = '0;
          cva6_dtlb_ppn          = '0;
          // Get back to CVA6 after the translation
          if (translation_valid) mmu_state_d = CVA6;
        end
        default: mmu_state_d = CVA6;
      endcase
    end

    logic store_buffer_empty;
    // ------------------
    // Store Unit
    // ------------------
    store_unit i_store_unit (
        .clk_i,
        .rst_ni,
        .flush_i,
        .no_st_pending_o,
        .store_buffer_empty_o  ( store_buffer_empty   ),

        .valid_i               ( st_valid_i           ),
        .lsu_ctrl_i            ( lsu_ctrl             ),
        .pop_st_o              ( pop_st               ),
        .commit_i,
        .commit_ready_o,
        .amo_valid_commit_i,

        .valid_o               ( st_valid             ),
        .trans_id_o            ( st_trans_id          ),
        .result_o              ( st_result            ),
        .ex_o                  ( st_ex                ),
        // MMU port
        .translation_req_o     ( cva6_st_translation_req   ),
        .vaddr_o               ( st_vaddr             ),
        .mem_paddr_o           ( mem_paddr_o          ),
        .paddr_i               ( cva6_mmu_paddr            ),
        .ex_i                  ( cva6_mmu_exception        ),
        .dtlb_hit_i            ( cva6_dtlb_hit             ),
        // Load Unit
        .page_offset_i         ( page_offset          ),
        .page_offset_matches_o ( page_offset_matches  ),
        // AMOs
        .amo_req_o,
        .amo_resp_i,
        // to memory arbiter
        .req_port_i             ( dcache_req_ports_i [2] ),
        .req_port_o             ( dcache_req_ports_o [2] )
    );

    // ------------------
    // Load Unit
    // ------------------
    load_unit #(
        .ArianeCfg ( ArianeCfg )
    ) i_load_unit (
        .valid_i               ( ld_valid_i           ),
        .lsu_ctrl_i            ( lsu_ctrl             ),
        .pop_ld_o              ( pop_ld               ),

        .valid_o               ( ld_valid             ),
        .trans_id_o            ( ld_trans_id          ),
        .result_o              ( ld_result            ),
        .ex_o                  ( ld_ex                ),
        // MMU port
        .translation_req_o     ( ld_translation_req   ),
        .vaddr_o               ( ld_vaddr             ),
        .paddr_i               ( cva6_mmu_paddr            ),
        .ex_i                  ( cva6_mmu_exception        ),
        .dtlb_hit_i            ( cva6_dtlb_hit             ),
        .dtlb_ppn_i            ( cva6_dtlb_ppn             ),
        // to store unit
        .page_offset_o         ( page_offset          ),
        .page_offset_matches_i ( page_offset_matches  ),
        .store_buffer_empty_i  ( store_buffer_empty   ),
        // to memory arbiter
        .req_port_i            ( dcache_req_ports_i [1] ),
        .req_port_o            ( dcache_req_ports_o [1] ),
        .dcache_wbuffer_not_ni_i,
        .commit_tran_id_i,
        .*
    );

    // ----------------------------
    // Output Pipeline Register
    // ----------------------------

    // amount of pipeline registers inserted for load/store return path
    // can be tuned to trade-off IPC vs. cycle time

    shift_reg #(
        .dtype ( logic[$bits(ld_valid) + $bits(ld_trans_id) + $bits(ld_result) + $bits(ld_ex) - 1: 0]),
        .Depth ( cva6_config_pkg::CVA6ConfigNrLoadPipeRegs )
    ) i_pipe_reg_load (
        .clk_i,
        .rst_ni,
        .d_i ( {ld_valid, ld_trans_id, ld_result, ld_ex} ),
        .d_o ( {load_valid_o, load_trans_id_o, load_result_o, load_exception_o} )
    );

    shift_reg #(
        .dtype ( logic[$bits(st_valid) + $bits(st_trans_id) + $bits(st_result) + $bits(st_ex) - 1: 0]),
        .Depth ( cva6_config_pkg::CVA6ConfigNrStorePipeRegs )
    ) i_pipe_reg_store (
        .clk_i,
        .rst_ni,
        .d_i ( {st_valid, st_trans_id, st_result, st_ex} ),
        .d_o ( {store_valid_o, store_trans_id_o, store_result_o, store_exception_o} )
    );

    // determine whether this is a load or store
    always_comb begin : which_op

        ld_valid_i = 1'b0;
        st_valid_i = 1'b0;

        cva6_translation_req      = 1'b0;
        cva6_mmu_vaddr            = {riscv::VLEN{1'b0}};

        // check the operation to activate the right functional unit accordingly
        unique case (lsu_ctrl.fu)
            // all loads go here
            LOAD:  begin
                ld_valid_i           = lsu_ctrl.valid;
                cva6_translation_req      = ld_translation_req;
                cva6_mmu_vaddr            = ld_vaddr;
            end
            // all stores go here
            STORE: begin
                st_valid_i           = lsu_ctrl.valid;
                cva6_translation_req      = cva6_st_translation_req;
                cva6_mmu_vaddr            = st_vaddr;
            end
            // not relevant for the LSU
            default: ;
        endcase
    end


    // ---------------
    // Byte Enable
    // ---------------
    // we can generate the byte enable from the virtual address since the last
    // 12 bit are the same anyway
    // and we can always generate the byte enable from the address at hand
    assign be_i = riscv::IS_XLEN64 ? be_gen(vaddr_i[2:0], extract_transfer_size(fu_data_i.operation)):
                                     be_gen_32(vaddr_i[1:0], extract_transfer_size(fu_data_i.operation));

    // ------------------------
    // Misaligned Exception
    // ------------------------
    // we can detect a misaligned exception immediately
    // the misaligned exception is passed to the functional unit via the MMU, which in case
    // can augment the exception if other memory related exceptions like a page fault or access errors
    always_comb begin : data_misaligned_detection

        cva6_misaligned_exception = {
            {riscv::XLEN{1'b0}},
            {riscv::XLEN{1'b0}},
            1'b0
        };

        data_misaligned = 1'b0;

        if (lsu_ctrl.valid) begin
            case (lsu_ctrl.operation)
                // double word
                LD, SD, FLD, FSD,
                AMO_LRD, AMO_SCD,
                AMO_SWAPD, AMO_ADDD, AMO_ANDD, AMO_ORD,
                AMO_XORD, AMO_MAXD, AMO_MAXDU, AMO_MIND,
                AMO_MINDU: begin
                    if (lsu_ctrl.vaddr[2:0] != 3'b000) begin
                        data_misaligned = 1'b1;
                    end
                end
                // word
                LW, LWU, SW, FLW, FSW,
                AMO_LRW, AMO_SCW,
                AMO_SWAPW, AMO_ADDW, AMO_ANDW, AMO_ORW,
                AMO_XORW, AMO_MAXW, AMO_MAXWU, AMO_MINW,
                AMO_MINWU: begin
                    if (lsu_ctrl.vaddr[1:0] != 2'b00) begin
                        data_misaligned = 1'b1;
                    end
                end
                // half word
                LH, LHU, SH, FLH, FSH: begin
                    if (lsu_ctrl.vaddr[0] != 1'b0) begin
                        data_misaligned = 1'b1;
                    end
                end
                // byte -> is always aligned
                default:;
            endcase
        end

        if (data_misaligned) begin

            if (lsu_ctrl.fu == LOAD) begin
                cva6_misaligned_exception = {
                    riscv::LD_ADDR_MISALIGNED,
                    {{riscv::XLEN-riscv::VLEN{1'b0}},lsu_ctrl.vaddr},
                    1'b1
                };

            end else if (lsu_ctrl.fu == STORE) begin
                cva6_misaligned_exception = {
                    riscv::ST_ADDR_MISALIGNED,
                    {{riscv::XLEN-riscv::VLEN{1'b0}},lsu_ctrl.vaddr},
                    1'b1
                };
            end
        end

        if (en_ld_st_translation_i && lsu_ctrl.overflow) begin

            if (lsu_ctrl.fu == LOAD) begin
                cva6_misaligned_exception = {
                    riscv::LD_ACCESS_FAULT,
                    {{riscv::XLEN-riscv::VLEN{1'b0}},lsu_ctrl.vaddr},
                    1'b1
                };

            end else if (lsu_ctrl.fu == STORE) begin
                cva6_misaligned_exception = {
                    riscv::ST_ACCESS_FAULT,
                    {{riscv::XLEN-riscv::VLEN{1'b0}},lsu_ctrl.vaddr},
                    1'b1
                };
            end
        end
    end

    // ------------------
    // LSU Control
    // ------------------
    // new data arrives here
    lsu_ctrl_t lsu_req_i;

    assign lsu_req_i = {lsu_valid_i, vaddr_i, overflow, fu_data_i.operand_b, be_i, fu_data_i.fu, fu_data_i.operation, fu_data_i.trans_id};

    lsu_bypass lsu_bypass_i (
        .lsu_req_i          ( lsu_req_i   ),
        .lsu_req_valid_i    ( lsu_valid_i ),
        .pop_ld_i           ( pop_ld      ),
        .pop_st_i           ( pop_st      ),

        .lsu_ctrl_o         ( lsu_ctrl    ),
        .ready_o            ( lsu_ready_o ),
        .*
    );

    assign lsu_addr_o = lsu_ctrl.vaddr;
    assign lsu_rmask_o = lsu_ctrl.fu == LOAD ? lsu_ctrl.be : '0;
    assign lsu_wmask_o = lsu_ctrl.fu == STORE ? lsu_ctrl.be : '0;
    assign lsu_addr_trans_id_o = lsu_ctrl.trans_id;

endmodule
