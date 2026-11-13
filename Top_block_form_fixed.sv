module Top #(
    parameter integer DECIMATION     = 250,
    parameter integer RECORD_SAMPLES = 200_000,
    parameter integer AUDIO_W        = 7
)(
    input  logic       clk,
    input  logic       pdm_ce,
    input  logic       data_in,
    input  logic       record_btn,
    input  logic       pause_on,
    input  logic [3:0] speed_select,
    output logic       data_out = 1'b0
);
    localparam integer ADDR_W  = $clog2(RECORD_SAMPLES);
    // The normal build uses native compact PCM end-to-end.  Keep the wider
    // intermediate only for the optional legacy 4-bit companded build.
    localparam integer PCM_W = (AUDIO_W == 4) ? 16 : AUDIO_W;
    localparam integer DECIM_W = $clog2(DECIMATION);
    localparam integer CIC_W = 1 + 2*$clog2(DECIMATION);
    localparam integer BLOCK_SUM_W = $clog2(DECIMATION + 1) + 1;
    // The two triangular block halves peak at R*(R+1)/2, while only their
    // final sum needs the full R^2 CIC output width.  Keeping bounded state at
    // its mathematical width is the block-form equivalent of CIC word pruning.
    localparam integer BLOCK_STATE_W =
        1 + $clog2((DECIMATION*(DECIMATION+1))/2 + 1);
    localparam integer CIC_SHIFT_RAW = 2*$clog2(DECIMATION) - (PCM_W-1);
    localparam integer RECORD_GAIN_BITS = 4;
    localparam integer CIC_SHIFT_GAINED = CIC_SHIFT_RAW - RECORD_GAIN_BITS;
    localparam integer CIC_SHIFT = (CIC_SHIFT_GAINED > 0) ? CIC_SHIFT_GAINED : 0;
    localparam integer BANK_ADDR_W = (ADDR_W < 15) ? ADDR_W : 15;
    localparam integer BANK_DEPTH  = 1 << BANK_ADDR_W;
    localparam integer BANKS       = (RECORD_SAMPLES + BANK_DEPTH - 1) / BANK_DEPTH;
    localparam integer BANK_W      = (BANKS <= 1) ? 1 : $clog2(BANKS);
    localparam logic [ADDR_W-1:0] LAST_ADDR = RECORD_SAMPLES-1;
    localparam logic [ADDR_W:0] RECORD_SIZE_EXT = RECORD_SAMPLES;
    localparam logic [DECIM_W-1:0] DECIM_LAST = DECIMATION-1;

    typedef enum logic [1:0] {
        S_IDLE   = 2'd0,
        S_RECORD = 2'd1,
        S_PRIME  = 2'd2,
        S_PLAY   = 2'd3
    } state_t;

    state_t state = S_IDLE;
    logic [3:0] speed_mode;
    // SW[5:2] directly selects the existing 15 playback modes.  The unused
    // 4'hf code has a defined safe behavior: normal 1x playback.
    assign speed_mode = (&speed_select) ? 4'd0 : speed_select;

    // Split the 200000 x AUDIO_W-bit store into 32K-sample banks. Explicit banks
    // avoid a fragile deep RAMB36 cascade for a single 200K-deep memory.
    logic                     mem_we = 1'b0, mem_re = 1'b0;
    logic [ADDR_W-1:0]        mem_addr = '0;
    logic signed [AUDIO_W-1:0] mem_wdata = '0, mem_rdata;
    logic [BANKS-1:0]          bank_enable;
    logic [BANK_W-1:0]         bank_select;
    logic signed [AUDIO_W-1:0] bank_dout [0:BANKS-1];

    generate
        if (BANKS == 1) begin : gen_one_bank_select
            assign bank_select = '0;
        end else begin : gen_many_bank_select
            assign bank_select = mem_addr[ADDR_W-1:BANK_ADDR_W];
        end
    endgenerate
    always_comb begin
        bank_enable = '0;
        // mem_addr is constrained to 0..LAST_ADDR by the record/playback FSM,
        // so bank_select can never name one of the unused binary codes.
        if (mem_we || mem_re)
            bank_enable[bank_select] = 1'b1;
    end

    // Pack the normal seven-bank read mux into dedicated slice mux resources.
    // Each half is a four-input data mux that fits one LUT6 per audio bit;
    // MUXF7 then selects the low/high bank group with bank_select[2].  This
    // keeps the existing latency and arithmetic while avoiding a scattered
    // second LUT level near the individual BRAM columns.
    generate
        if (BANKS == 7) begin : gen_muxf7_read_select
            logic [AUDIO_W-1:0] bank_mux_low, bank_mux_high;
            always_comb begin
                case (bank_select[1:0])
                    2'd0: bank_mux_low = bank_dout[0];
                    2'd1: bank_mux_low = bank_dout[1];
                    2'd2: bank_mux_low = bank_dout[2];
                    default: bank_mux_low = bank_dout[3];
                endcase
                case (bank_select[1:0])
                    2'd0: bank_mux_high = bank_dout[4];
                    2'd1: bank_mux_high = bank_dout[5];
                    2'd2: bank_mux_high = bank_dout[6];
                    default: bank_mux_high = '0;
                endcase
            end
            for (genvar mux_bit = 0; mux_bit < AUDIO_W; mux_bit = mux_bit + 1) begin : gen_bit
                MUXF7 read_mux (
                    .I0(bank_mux_low[mux_bit]),
                    .I1(bank_mux_high[mux_bit]),
                    .S(bank_select[2]),
                    .O(mem_rdata[mux_bit])
                );
            end
        end else begin : gen_generic_read_select
            always_comb mem_rdata = bank_dout[bank_select];
        end
    endgenerate

    genvar bank_i;
    generate
        for (bank_i = 0; bank_i < BANKS; bank_i = bank_i + 1) begin : gen_audio_bank
            (* keep_hierarchy = "yes" *) PcmBramBank #(
                .ADDR_W(BANK_ADDR_W),
                .DATA_W(AUDIO_W)
            ) bank (
                .clk(clk),
                .enable(bank_enable[bank_i]),
                .write_enable(mem_we),
                .addr(mem_addr[BANK_ADDR_W-1:0]),
                .data_in(mem_wdata),
                .data_out(bank_dout[bank_i])
            );
        end
    endgenerate

    // Exact block form of a second-order CIC.  For each R-sample PDM block,
    // forward_sum accumulates the current block with weights R..1 and
    // reverse_sum accumulates it with weights 0..R-1.  The CIC output is the
    // current forward sum plus the preceding block's reverse sum.  This is
    // algebraically identical to two integrators followed by two combs, but
    // all state is bounded and needs fewer registers.
    logic [DECIM_W-1:0] decim_count = '0;
    logic signed [BLOCK_SUM_W-1:0] cic_block_sum = '0;
    logic signed [BLOCK_STATE_W-1:0] cic_forward_sum = '0;
    logic signed [BLOCK_STATE_W-1:0] cic_reverse_sum = '0;
    logic signed [BLOCK_STATE_W-1:0] cic_previous_tail = '0;
    logic signed [BLOCK_SUM_W-1:0] cic_block_sum_next;
    logic signed [CIC_W-1:0] cic_alu_a, cic_alu_b, cic_alu_result;
    logic [1:0] cic_phase = '0;
    logic cic_emit = 1'b0, cic_subtract;

    function automatic logic signed [AUDIO_W-1:0] encode_sample(
        input logic signed [PCM_W-1:0] sample
    );
        logic [PCM_W-1:0] magnitude;
        logic [2:0] magnitude_code;
        begin
            if (AUDIO_W == 4) begin
                magnitude = sample[PCM_W-1] ? $unsigned(-sample) : $unsigned(sample);
                if (magnitude[6] || magnitude[5])
                    magnitude_code = 3'd7;
                else if (magnitude[4])
                    magnitude_code = 3'd6;
                else if (magnitude[3])
                    magnitude_code = 3'd5;
                else if (magnitude[2])
                    magnitude_code = 3'd4;
                else
                    magnitude_code = magnitude[2:0];
                encode_sample = $signed({sample[PCM_W-1], magnitude_code});
            end else begin
                encode_sample = sample[AUDIO_W-1:0];
            end
        end
    endfunction

    // Normalize the CIC gain and apply 16x record gain.  Saturate before
    // narrowing so microphone peaks cannot wrap across zero and create a
    // full-scale discontinuity (heard as a pop/crackle).
    function automatic logic signed [PCM_W-1:0] cic_to_pcm(
        input logic signed [CIC_W-1:0] sample
    );
        logic signed [CIC_W-1:0] scaled;
        logic signed [CIC_W-1:0] pcm_max;
        logic signed [CIC_W-1:0] pcm_min;
        begin
            scaled  = sample >>> CIC_SHIFT;
            pcm_max = (1 <<< (PCM_W-1)) - 1;
            pcm_min = -(1 <<< (PCM_W-1));
            if (scaled > pcm_max)
                cic_to_pcm = {1'b0, {(PCM_W-1){1'b1}}};
            else if (scaled < pcm_min)
                cic_to_pcm = {1'b1, {(PCM_W-1){1'b0}}};
            else
                cic_to_pcm = scaled;
        end
    endfunction

    function automatic logic signed [PCM_W-1:0] decode_sample(
        input logic signed [AUDIO_W-1:0] code
    );
        logic [PCM_W-1:0] magnitude;
        begin
            if (AUDIO_W == 4) begin
                case (code[2:0])
                    3'd0: magnitude = 7'd0;
                    3'd1: magnitude = 7'd1;
                    3'd2: magnitude = 7'd2;
                    3'd3: magnitude = 7'd3;
                    3'd4: magnitude = 7'd6;
                    3'd5: magnitude = 7'd12;
                    3'd6: magnitude = 7'd24;
                    default: magnitude = 7'd48;
                endcase
                decode_sample = code[3] ? -$signed(magnitude) : $signed(magnitude);
            end else begin
                decode_sample = {{(PCM_W-AUDIO_W){code[AUDIO_W-1]}}, code};
            end
        end
    endfunction

    always_comb begin
        // The narrow block accumulator is cheap enough to update beside the
        // wide shared ALU.  Both paths consume data_in only on the pdm_ce edge,
        // so later microphone transitions cannot affect this sample.
        cic_block_sum_next = cic_block_sum
                           + (data_in ? $signed({{(BLOCK_SUM_W-1){1'b0}}, 1'b1})
                                      : $signed({BLOCK_SUM_W{1'b1}}));
        cic_alu_a = '0;
        cic_alu_b = '0;
        cic_subtract = 1'b0;
        case (cic_phase)
            // reverse_sum += sample_index * PDM (+index/-index)
            2'd0: begin
                cic_alu_a = {{(CIC_W-BLOCK_STATE_W){cic_reverse_sum[BLOCK_STATE_W-1]}},
                             cic_reverse_sum};
                cic_alu_b = {{(CIC_W-DECIM_W){1'b0}}, decim_count};
                cic_subtract = ~data_in;
            end
            // forward_sum += the block sum captured in the preceding cycle
            2'd1: begin
                cic_alu_a = {{(CIC_W-BLOCK_STATE_W){cic_forward_sum[BLOCK_STATE_W-1]}},
                             cic_forward_sum};
                cic_alu_b = {{(CIC_W-BLOCK_SUM_W){cic_block_sum[BLOCK_SUM_W-1]}}, cic_block_sum};
            end
            // PCM = current forward sum + previous reverse sum
            default: begin
                cic_alu_a = {{(CIC_W-BLOCK_STATE_W){cic_forward_sum[BLOCK_STATE_W-1]}},
                             cic_forward_sum};
                cic_alu_b = {{(CIC_W-BLOCK_STATE_W){cic_previous_tail[BLOCK_STATE_W-1]}},
                             cic_previous_tail};
            end
        endcase
        // A single add/sub datapath is time-shared across the four CIC stages.
        cic_alu_result = cic_alu_a + (cic_subtract ? ~cic_alu_b : cic_alu_b)
                                   + cic_subtract;
    end

    // Recording and playback are mutually exclusive, so they share the
    // sample address.  Fast modes skip 1..8 addresses per output frame;
    // slow modes use a 3-bit exact modulo-2..8 divider and repeat samples.
    // This avoids a wide fractional phase accumulator and its step ROM.
    // mem_addr is also the logical record/playback cursor.  Keeping a second
    // 18-bit sample_addr register duplicated both the counter and its next-
    // state decode.  Retain this alias for simulation/debug visibility only.
    wire [ADDR_W-1:0] sample_addr = mem_addr;
    logic [2:0] slow_count = '0;
    logic [2:0] slow_terminal;
    logic [3:0] advance;
    logic [ADDR_W:0] address_sum;

    always_comb begin
        // Record and playback are mutually exclusive, so the same wide adder
        // handles record +1 and playback +1..8 instead of synthesizing two
        // separate address carry chains.
        advance = (state == S_RECORD) ? 4'd1 :
                  ((speed_mode < 4'd8) ? (speed_mode + 1'b1) : 4'd1);
        slow_terminal = speed_mode[2:0] + 1'b1;
        address_sum = {1'b0, mem_addr} + {{(ADDR_W-3){1'b0}}, advance};
    end

    // The inferred BRAM has one registered read cycle. The extra pending
    // stage ensures pcm_current sees the newly-read value, not the old one.
    logic [1:0] read_pending = '0;
    logic signed [PCM_W-1:0] pcm_current = '0;

    // The board controls enter Top directly.  Convert the record switch's
    // rising edge into a one-clock event.  Playback speed is selected directly
    // by speed_select, so no edge detector or mode counter is required.
    logic record_previous = 1'b0;
    logic record_press;
    assign record_press  = record_btn & ~record_previous;

    // First-order PCM_W-bit sigma-delta DAC. Run it at the full 100 MHz fabric
    // clock, independently of the 3.125 MHz microphone sample enable. Signed
    // PCM is changed to offset binary, so zero is a quiet 50-percent density.
    logic [PCM_W-1:0] sigma_acc = '0;
    logic [PCM_W:0] sigma_sum;
    logic [PCM_W-1:0] dac_sample;

    always_comb begin
        dac_sample = (state == S_PLAY) ?
                     ($unsigned(pcm_current) ^ {1'b1, {(PCM_W-1){1'b0}}}) :
                     {1'b1, {(PCM_W-1){1'b0}}};
        sigma_sum  = {1'b0, sigma_acc} + {1'b0, dac_sample};
    end

    always_ff @(posedge clk) begin
        record_previous <= record_btn;
        mem_we <= 1'b0;
        mem_re <= 1'b0;
        read_pending <= {read_pending[0], mem_re};

            if (read_pending[1]) begin
                pcm_current <= decode_sample(mem_rdata);
                if (state == S_PRIME) begin
                    state       <= S_PLAY;
                    decim_count <= '0;
                end
            end

            if (record_press) begin
                mem_we         <= 1'b0;
                mem_re         <= 1'b0;
                state         <= S_RECORD;
                mem_addr      <= '0;
                decim_count   <= '0;
                cic_block_sum    <= '0;
                cic_forward_sum  <= '0;
                cic_reverse_sum  <= '0;
                cic_previous_tail <= '0;
                cic_phase     <= '0;
                cic_emit      <= 1'b0;
                read_pending  <= '0;
            end else begin
                case (state)
                    S_RECORD: begin
                        // The bank consumes the old address on the cycle after
                        // mem_we is asserted.  Advance the shared cursor only
                        // after that write has taken place.
                        if (mem_we)
                            mem_addr <= address_sum[ADDR_W-1:0];
                        case (cic_phase)
                            2'd0: if (pdm_ce) begin
                                cic_reverse_sum <= cic_alu_result;
                                cic_block_sum <= cic_block_sum_next;
                                cic_emit <= (decim_count == DECIM_LAST);
                                if (decim_count == DECIM_LAST)
                                    decim_count <= '0;
                                else
                                    decim_count <= decim_count + 1'b1;
                                cic_phase <= 2'd1;
                            end
                            2'd1: begin
                                cic_forward_sum <= cic_alu_result;
                                cic_phase <= cic_emit ? 2'd2 : 2'd0;
                            end
                            default: begin
                                cic_previous_tail <= cic_reverse_sum;
                                cic_block_sum   <= '0;
                                cic_forward_sum <= '0;
                                cic_reverse_sum <= '0;
                                mem_wdata   <= encode_sample(
                                    cic_to_pcm(cic_alu_result)
                                );
                                mem_we      <= 1'b1;
                                cic_phase   <= 2'd0;
                                if (mem_addr == LAST_ADDR) begin
                                    state      <= S_PRIME;
                                    slow_count <= '0;
                                end
                            end
                        endcase
                    end

                    S_PRIME: begin
                        if (!mem_we && (read_pending == 0) && !mem_re) begin
                            mem_addr <= '0;
                            mem_re   <= 1'b1;
                        end
                    end

                    S_PLAY: begin
                        if (pdm_ce) begin
                            if (!pause_on) begin
                                if (decim_count == DECIM_LAST) begin
                                    decim_count <= '0;
                                    if ((speed_mode < 4'd8) ||
                                                 (slow_count == slow_terminal)) begin
                                        slow_count <= '0;
                                        if (address_sum >= RECORD_SIZE_EXT) begin
                                            // RECORD_SAMPLES is a multiple of eight and
                                            // advance is at most eight.  A crossing can
                                            // therefore only wrap to address 0..7; its
                                            // low three sum bits are the exact result.
                                            mem_addr  <= {{(ADDR_W-3){1'b0}}, address_sum[2:0]};
                                        end else begin
                                            mem_addr  <= address_sum[ADDR_W-1:0];
                                        end
                                        mem_re <= 1'b1;
                                    end else begin
                                        slow_count <= slow_count + 1'b1;
                                    end
                                end else begin
                                    decim_count <= decim_count + 1'b1;
                                end
                            end
                        end
                    end

                    default: ;
                endcase
            end

        // The DAC carrier/noise-shaper must not be throttled by pdm_ce.
        sigma_acc <= sigma_sum[PCM_W-1:0];
        data_out  <= sigma_sum[PCM_W];
    end
endmodule

module PcmBramBank #(
    parameter integer ADDR_W = 15,
    parameter integer DATA_W = 7
)(
    input  logic                     clk,
    input  logic                     enable,
    input  logic                     write_enable,
    input  logic [ADDR_W-1:0]        addr,
    input  logic signed [DATA_W-1:0] data_in,
    output logic signed [DATA_W-1:0] data_out
);
    (* ram_style = "block" *) logic signed [DATA_W-1:0] memory [0:(1<<ADDR_W)-1];

    always_ff @(posedge clk) begin
        if (enable) begin
            if (write_enable)
                memory[addr] <= data_in;
            else
                data_out <= memory[addr];
        end
    end
endmodule


