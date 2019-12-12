`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// MIT 6.111 Final Project
// Rhian Chavez & Miles Johnson
//////////////////////////////////////////////////////////////////////////////////

module fft_mag(     input clk_100mhz,
                    input [11:0] sampled_adc_data,
                    input valid_ADC,
                    output logic [23:0] freq_mag,
                    output logic [6:0] freq_index = 7'd0,
                    output logic valid_out
    );

    // NOTE: samples to FFT will be coming much at about 24kHz and clock is 100Mhz,
    // take care of that in simulation
    
    // intialize fft logic
    // s_axis stuff
    // if I don't supply config data, it defaults to okay stuff!
//    logic s_axis_config_tvalid;         // input, assert master able to provide config data
//    logic s_axis_config_tready;         // output, assert slave ready to accept config data
//    logic s_axis_config_tdata;          // input, carries config info
//    logic s_axis_data_tvalid;           // input, assert master able to provide signal data
//    logic s_axis_data_tdata;            // input, carries unprocessed sample data XN_RE, XN_IM
//    logic s_axis_data_tlast;            // input, asserted on last sample of frame
    
    logic [6:0] window_count = 7'd0;
    
    always @(posedge clk_100mhz) begin
        if (valid_ADC) begin
            window_count <= window_count + 1'b1;
        end
        if (valid_out) begin
            freq_index <= freq_index + 1'b1;
        end 
    end
    
    assign frame_last = window_count[6]&&window_count[5]&&window_count[4]&&
                        window_count[3]&&window_count[2]&&window_count[1]&&window_count[0];
    
    // m_axis stuff
    logic fft_valid;                    // output, assert able to provide freq data
//    logic m_axis_data_tready;           // input, assert ready for freq data
    logic [31:0] fft;                   // output, carries processed freq data XK_RE & XK_IM
    logic tlast;                        // output, assert last freq sample of frame
    // need to count up from valid outputs so that the correct bin is known!
    
    // rest (don't use rn, hopefully don't need them)
    logic event_frame_started;          // output, assert processing new frame
    logic event_tlast_unexpected;       // output, obvious assertion
    logic event_tlast_missing;          // output, obvious assertion
    logic event_fft_overflow;           // output, obvious assertion
    logic event_data_in_channel_halt;   // output, obvious assertion
    logic event_data_out_channel_halt;  // output, obvious assertion
    logic event_status_channel_halt;    // output, obvious assertion
    
    logic [7:0] s_axis_config_tdata = 8'd0;
    
    // initialize fft
    xfft_0 my_fft ( .aclk(clk_100mhz), .s_axis_data_tdata({20'd0, sampled_adc_data}), .s_axis_data_tlast(frame_last),
                .s_axis_data_tvalid(valid_ADC), .m_axis_data_tvalid(fft_valid), .m_axis_data_tready(1'b1), 
                .m_axis_data_tdata(fft), .m_axis_data_tlast(tlast), .s_axis_config_tvalid(1'b0),
                .s_axis_config_tdata(s_axis_config_tdata) );
    
    // intialize squarer logic
    logic [32:0] power;
    logic squarer_done;
    
    // pass frequency data to squarer (3 clk cycle latency)
    squarer squarer_1(  .valid_in(fft_valid), .clk_100mhz(clk_100mhz), .real_part(fft[15:0]), .imag_part(fft[31:16]), 
                        .sum(power), .out_valid(squarer_done));
    
    
    // take square root using vivado logic cordic (single cycle throughput)
    cordic_0 sq_root(   .aclk(clk_100mhz), .s_axis_cartesian_tdata({7'd0, power}), .s_axis_cartesian_tvalid(squarer_done),
                        .m_axis_dout_tdata(freq_mag), .m_axis_dout_tvalid(valid_out)); 
    
    // since cordic valid output is only asserted for new output, just count that to determine which freq bin
    
endmodule
