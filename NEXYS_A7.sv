module NEXYS_A7 #(
    parameter integer AUDIO_W = 7
)(
    input  logic CLK100MHZ,
    input  logic [2:0] SW,
    output logic M_CLK,
    input  logic M_DATA,
    output logic M_LRSEL,
    output wire  AUD_PWM,
    output logic AUD_SD
);
    logic pdm_half_cycle;
    logic pdm_period;
    logic pdm_ce;
    logic m_clk_next;
    logic audio_bit;

    // One rotating bit replaces the binary counter and decode. Q (tap 15)
    // and Q31 are 16 clocks apart, so they produce the two M_CLK edges.
    (* RLOC = "X0Y0", BEL = "A6LUT" *)
    SRLC32E #(.INIT(32'h00000001)) pdm_ring (
        .Q(pdm_half_cycle), .Q31(pdm_period), .A(5'd15),
        .CE(1'b1), .CLK(CLK100MHZ), .D(pdm_period)
    );

    // Pack the toggle function and M_CLK register beside the SRL.  LUT INIT
    // 0x56 implements I0 XOR (I1 OR I2).
    (* RLOC = "X0Y0", BEL = "B6LUT" *)
    LUT3 #(.INIT(8'h56)) pdm_clock_toggle (
        .O(m_clk_next), .I0(M_CLK), .I1(pdm_half_cycle), .I2(pdm_period)
    );
    (* RLOC = "X0Y0", BEL = "BFF" *)
    FDRE #(.INIT(1'b0)) pdm_clock_ff (
        .Q(M_CLK), .C(CLK100MHZ), .CE(1'b1), .D(m_clk_next), .R(1'b0)
    );

    // Q31 is one pulse per 32 clocks.  Top samples it on the same fabric edge
    // that lowers M_CLK, after the microphone data has settled for 160 ns.
    assign pdm_ce = pdm_period;

    assign M_LRSEL = 1'b0;
    assign AUD_SD  = 1'b1;
    // The board's analog rail supplies the high level.  Drive only zero and
    // release the pin for one, as required by the Nexys A7 audio interface.
    assign AUD_PWM = audio_bit ? 1'bz : 1'b0;

    Top #(
        .AUDIO_W(AUDIO_W)
    ) recorder (
        .clk(CLK100MHZ),
        .pdm_ce(pdm_ce),
        .data_in(M_DATA),
        .record_btn(SW[0]),
        .pause_on(SW[1]),
        .speed_btn(SW[2]),
        .data_out(audio_bit)
    );
endmodule
