`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// MIT 6.111 Final Project
// Rhian Chavez & Miles Johnson
//////////////////////////////////////////////////////////////////////////////////

module top_level(   input clk_100mhz,
                    input [15:0] sw,
                    input btnc, btnu, btnd, btnr, btnl,
                    input vauxp3,
                    input vauxn3,
                    input vn_in,
                    input vp_in,
                    output logic [15:0] led,
                    output logic [7:0] ja,
                    output logic aud_pwm,
                    output logic aud_sd,
                    input sd_cd, 
                    output sd_reset, 
                    output sd_sck, 
                    output sd_cmd, 
                    inout [3:0] sd_dat
    );
    
    // ADD ALL SD CARD LOGIC HERE
    
    logic wr;
    logic rd;
    logic[7:0] din;
    logic[7:0] dout;
    
    logic ready_for_next_byte;
    logic last_next_byte;
    logic next_byte;
    assign next_byte= (ready_for_next_byte&&!last_next_byte);
     
    logic ready_for_next_block;
    
    logic starting_next_block;
    logic last_next_starting;
    logic next_starting;
    assign next_starting= (starting_next_block&&!last_next_starting);
    
    logic BTNUU;
    
    debounce debouncer(.reset_in(btnd), .clock_in(clk_100mhz), .noisy_in(btnu),.clean_out(BTNUU));
    
    SD_interface sd_int(.CLK100MHZ(clk_100mhz), .SD_CD(sd_cd),.SD_RESET(sd_reset),.SD_SCK(sd_sck), .SD_CMD(sd_cmd), 
        .SD_DAT(sd_dat), .BTNC(btnc), .wr(wr),.din(din),.rd(rd),.data_out(dout),
        .ready_for_next_byte(ready_for_next_byte),.ready_for_next_block(ready_for_next_block),.starting_next_block(starting_next_block),.BTNU(BTNUU));
    
    logic signed [7:0] filter_input;
    logic signed [17:0] filter_output;
    logic[10:0] counter;
    logic filter_ready;
    fir31 filter (.clk_in(clk_100mhz),.rst_in(btnd),.ready_in(filter_ready),.x_in(filter_input),.y_out(filter_output));
        
    logic [11:0] first_sampled_adc_data;
    
    
    logic signed [7:0] filter_input2;
    logic signed [17:0] filter_output2;
    logic[10:0] counter2;
    logic filter_ready2;
    logic signed [7:0] audio_out2;
    logic zero;
    fir32 filter2 (.clk_in(clk_100mhz),.rst_in(btnd),.ready_in(filter_ready2),.x_in(filter_input2),.y_out(filter_output2));                    
    
    logic [7:0] FIFO1[511:0];
    logic [7:0] FIFO2[511:0];
    logic[9:0] fifo_counter1;
    logic[9:0] fifo_counter2;
    
    logic signed [7:0] audio_out;
    logic signed [7:0] filtered_adc_data;
    
    logic [7:0] vol_out;
    logic pwm_val; //pwm signal (HI/LO)
    assign aud_sd = 1;
    assign led = sw; //just to look pretty 
    

    // TAKE AUDIO INPUT FROM EITHER MIC/AUX/SD-CARD
    
    // PASS AUDIO TO OUTPUT AUX CONNECTED TO EXTERNALLY AMPLIFIED SPEAKER
    
    parameter SAMPLE_COUNT = 2082;      // downsample audio to approx 48 kHz sample rate. 
    parameter SAMPLE_COUNT_2 = 6;       // downsample filtered downsampled data to 8khz for FFT
    
    // filter out everything above 5khz or so. 
    
    // initialize ADC logic
    logic [15:0] sample_counter;
    logic [15:0] adc_data;
    logic signed [15:0] sampled_adc_data;
    logic sample_trigger;
    logic adc_ready;
    
    // code to interface with ADC from microphone
    xadc_wiz_0 my_adc ( .dclk_in(clk_100mhz), .daddr_in(8'h13), //read from 0x13 for channel vaux3
                    .vauxn3(vauxn3),.vauxp3(vauxp3),
                    .vp_in(1),.vn_in(1),
                    .di_in(16'b0),
                    .do_out(adc_data),.drdy_out(adc_ready),
                    .den_in(1), .dwe_in(0), .reset_in(1'b0) );                
    
    //MILES - SD and audio combinational logic BEGIN
    logic [15:0] sample_counter3rd;
    logic signed [7:0] sampled_adc_data3rd;
    logic sample_trigger3rd;
    assign sample_trigger3rd = (sample_counter3rd == 4095);
    
    always_ff @(posedge clk_100mhz)begin
        last_next_starting<=starting_next_block;
        last_next_byte<=ready_for_next_byte;
        
        if (sample_counter3rd == 4095)begin
            sample_counter3rd <= 16'b0;
        end else begin
            sample_counter3rd <= sample_counter3rd + 16'b1;
        end      
        if (next_starting) begin
            FIFO2<=FIFO1;
            fifo_counter1<=0;
            fifo_counter2<=0;
            ready_for_next_block<=0;
        end
        else begin
            if (next_byte) begin
                din<=FIFO2[fifo_counter2];
                fifo_counter2<=fifo_counter2+1;
            end
            if (sample_trigger3rd) begin
                //sampled_adc_data <= {~adc_data[11],adc_data[10:0]}; //convert to signed. incoming data is offset binary
                sampled_adc_data3rd<=filtered_adc_data;
                //https://en.wikipedia.org/wiki/Offset_binary
                
                
                if (fifo_counter1<10'd512) begin
                    FIFO1[fifo_counter1]<=sampled_adc_data3rd;//[11:4];
                    fifo_counter1<=fifo_counter1+1;
                end
                else begin
                    ready_for_next_block<=1;
                end
            end
        end
        if (counter==11'b111_1111_1111) begin
            counter<=0;
            filter_input<=first_sampled_adc_data[11:4];
            first_sampled_adc_data <= {~adc_data[11],adc_data[10:0]}; //convert to signed. incoming data is offset binary
            filtered_adc_data<=filter_output[17:10];
            filter_ready<=1;
        end
        else begin
            counter<=counter+1;
            filter_ready<=0;
        end
        
        if (counter2==11'd2047) begin
            counter2<=0;
            if (zero) begin
                filter_input2<=8'sb0;
                zero<=0;
            end
            else begin
                filter_input2<=audio_out;
                zero<=1;
            end
            
            audio_out2<=filter_output2[17:10];
            filter_ready2<=1;
        end
        else begin
            counter2<=counter2+1;
            filter_ready2<=0;
        end
        
        //makes sure both are not pressed at once
        if(btnr) begin
            wr<=1;
            rd<=0;
        end
        else if(btnl) begin
            wr<=0;
            rd<=1;
        end
        else begin
            wr<=0;
            rd<=0;
        end
        
        //If reading from SD, play from SD. If writing, play what writing (both filters), else only use second filter
        if (rd) begin
            audio_out<=dout;
        end
        else begin
            audio_out<=sampled_adc_data3rd;//[11:4];
        end
    end
    volume_control vc (.vol_in(sw[15:13]),
                       .signal_in(audio_out2), .signal_out(vol_out));
    pwm (.clk_in(clk_100mhz), .rst_in(btnd), .level_in({~vol_out[7],vol_out[6:0]}), .pwm_out(pwm_val));
    assign aud_pwm = pwm_val?1'bZ:1'b0; 
    
    //MILES - SD and audio combinational logic END
    
    
    // pass output of ADC into filter that removes everything above 4khz or so (down_sample to ~8khz before presenting to ADC)
    
//    logic [15:0] scaler;
    
    // use switches to scale ADC data
//    always_comb begin
//        case (sw[3:0])
//            4'b0000: scaler = adc_data;
//            4'b0001: scaler = adc_data >> 1;
//            4'b0011: scaler = adc_data >> 2;
//            4'b0100: scaler = adc_data >> 3;
//            4'b0101: scaler = adc_data >> 4;
//            4'b0110: scaler = adc_data >> 5;
//            4'b0111: scaler = adc_data >> 6;
//            4'b1000: scaler = adc_data >> 7;
//            4'b1001: scaler = adc_data >> 8;
//            4'b1010: scaler = adc_data >> 9;
//            4'b1011: scaler = adc_data >> 10;
//            4'b1100: scaler = adc_data >> 11;
//            4'b1101: scaler = adc_data >> 12;
//            4'b1110: scaler = adc_data >> 13;
//            4'b1111: scaler = adc_data >> 14;
//        endcase
//    end
    
//    logic [7:0] tone_750;
//    logic [7:0] vol_out;
//    logic [31:0] PHASE_INCR=32'd39370534;
//    logic pwm_val;
//    //generate a 750 Hz tone
//    sine_generator  tone750hz (   .clk_in(clk_100mhz), .rst_in(btnd), 
//                                 .step_in((sample_counter==4164)||(sample_counter==2082)), .amp_out(tone_750),.PHASE_INCR(PHASE_INCR));
    
//    volume_control vc (.vol_in(sw[15:13]),
//                       .signal_in(tone_750), .signal_out(vol_out));
//    pwm (.clk_in(clk_100mhz), .rst_in(btnd), .level_in({~vol_out[7],vol_out[6:0]}), .pwm_out(pwm_val));
//    assign aud_pwm = pwm_val?1'bZ:1'b0; 
//    assign aud_sd = 1;
    
    // Down-sample to ~ 48kHz
    assign sample_trigger = (sample_counter == SAMPLE_COUNT);

    always_ff @(posedge clk_100mhz) begin
        if (sample_trigger) begin
            sample_counter <= 32'b0;
            sampled_adc_data <= {~adc_data[11], adc_data[10:0]}; //convert from offset binary to 2's comp.
        end else begin
            sample_counter <= sample_counter + 32'b1;
        end
    end
    
    // apply filter to remove everything above 4khz
    // 0.165
    
//    logic [17:0] filter_out;
    
//    fir33 my_filter( .clk_in(clk_100mhz), .rst_in(1'b0), .ready_in(sample_trigger), .x_in(audio_out),//sampled_adc_data[11:4]),
//     .y_out(filter_out));
    
    // down sample again to 8khz
    logic [2:0] sample_counter_2 = 3'd0;
    
    assign sample_trigger_2 = (sample_counter_2 == 3'd5);
    logic [11:0] filtered_slow_data;
    
    always_ff @(posedge clk_100mhz) begin
        if (sample_trigger) begin   // counting number of previous samples to 6
            if (sample_trigger_2) begin
                sample_counter_2 <= 3'd0;
                filtered_slow_data <= {audio_out,4'b0};//filter_out[17:6];
            end
            else sample_counter_2 <= sample_counter_2 + 1'b1;
        end
    end
    
    // present data to FFT
    
    // fft logic
    logic [23:0] freq_mag;
    logic [6:0] freq_index;
    logic valid_freq_out;
    
    // initialize fft
    fft_mag my_magnater (   .clk_100mhz(clk_100mhz), .sampled_adc_data(filtered_slow_data), .valid_ADC((sample_trigger_2)&&(sample_trigger)), 
                            .freq_mag(freq_mag), .freq_index(freq_index), .valid_out(valid_freq_out));
    
    
    // mapping logic
    logic mapping_valid;
    logic [24:0] led_mag_out;
    
    // scale fft output
    
    logic [23:0] scalar;
    
    always_comb begin
        case (5'd0)      // was switch 3-0
            5'd0:   scalar = freq_mag;
            5'd1:   scalar = freq_mag>>1;
            5'd2:   scalar = freq_mag>>2;
            5'd3:   scalar = freq_mag>>3;
            5'd4:   scalar = freq_mag>>4;
            5'd5:   scalar = freq_mag>>5;
            5'd6:   scalar = freq_mag>>6;
            5'd7:   scalar = freq_mag>>7;
            5'd8:   scalar = freq_mag>>8;
            5'd9:   scalar = freq_mag>>9;
            5'd10:  scalar = freq_mag>>10;
            5'd11:  scalar = freq_mag>>11;
            5'd12:  scalar = freq_mag>>12;
            5'd13:  scalar = freq_mag>>13;
            5'd14:  scalar = freq_mag>>14;
            5'd15:  scalar = freq_mag>>15;
        default: scalar = freq_mag;
        endcase
    end
    
    // add beat detection 
    
    logic beat;
    
    beat_detect beat_boy(   .clk_100mhz(clk_100mhz), .valid_in(valid_freq_out), .fft_mag(scalar), .fft_index(freq_index), 
                            .beat_threshold(4'd0), .beat_flag(beat));
    
    // take fft magnitudes to bin -> led mapping
    freq_bin_to_led_num my_mapper ( .clk_100mhz(clk_100mhz), .fft_mag(scalar), .freq_index(freq_index), .valid_in(valid_freq_out), 
                                    .valid_out(mapping_valid), .led_mag_out(led_mag_out));
    
    // sweeper logic
    logic [7:0] threshold = 8'd50;
    logic [8:0] hue;
    logic [8:0] hue2;
    logic [7:0] val_out;
    logic [7:0] val_out2;
    logic sweep_out_ready;
    logic sweep_out_ready2;
    
    // take led mapped freq data to hue_sweep_listener
    hue_sweep_listen my_sweeper (   .clk_100mhz(clk_100mhz), .new_flag(mapping_valid), .freq_mag(led_mag_out), .threshold(sw[7:0]), 
                                    .hue(hue), .val_out(val_out), .out_ready(sweep_out_ready), .beat_in(1'b0));
                                    
    hue_sweep_listen my_sweeper2 (   .clk_100mhz(clk_100mhz), .new_flag(mapping_valid), .freq_mag(led_mag_out), .threshold((sw[7:0]-8'd3)), 
                                    .hue(hue2), .val_out(val_out2), .out_ready(sweep_out_ready2), .beat_in(1'b0));
 
    logic [23:0] rgb2;
    logic rgb_done2;
    logic [23:0] rgb;
    logic rgb_done;
    
    // take hue_sweep_listener to rgb converter
    hsv_to_rgb rgb_boi (.clk_100mhz(clk_100mhz), .hue(hue), .val(val_out), .valid_in(sweep_out_ready), .valid_out(rgb_done), .rgb(rgb));
    hsv_to_rgb rgb_boi2 (.clk_100mhz(clk_100mhz), .hue(hue2), .val(val_out2), .valid_in(sweep_out_ready2), .valid_out(rgb_done2), .rgb(rgb2));
    
    
    
    logic data_stream;
    logic led_clk;
    logic data_stream2;
    logic led_clk2;
    

     
    // take rgb data to leds 
    rgb_to_lights talker_boi ( .clk_100mhz(clk_100mhz), .rgb(rgb), .new_rgb(rgb_done),
                               .data_stream(data_stream), .clk_stream(led_clk) );
                               
    // take rgb data to leds 
    rgb_to_lights talker_boi2 ( .clk_100mhz(clk_100mhz), .rgb(rgb2), .new_rgb(rgb_done2),
                               .data_stream(data_stream2), .clk_stream(led_clk2) );
                               
    assign ja[0] = led_clk;
    assign ja[1] = led_clk;
    assign ja[2] = led_clk2;
    assign ja[3] = led_clk2;
    assign ja[4] = data_stream;
    assign ja[5] = data_stream;
    assign ja[6] = data_stream2;
    assign ja[7] = data_stream2;
    
endmodule


////Sine Wave Generator
//module sine_generator ( input clk_in, input rst_in, //clock and reset
//                        input step_in, //trigger a phase step (rate at which you run sine generator)
//                        input [31:0] PHASE_INCR,
//                        output logic [7:0] amp_out); //output phase   
//    //parameter PHASE_INCR = 32'b1000_0000_0000_0000_0000_0000_0000_0000>>5; //1/64th of 48 khz is 750 Hz
//    logic [7:0] divider;
//    logic [31:0] phase;
//    logic [7:0] amp;
//    assign amp_out = {~amp[7],amp[6:0]};
//    sine_lut lut_1(.clk_in(clk_in), .phase_in(phase[31:26]), .amp_out(amp));
    
//    always_ff @(posedge clk_in)begin
//        if (rst_in)begin
//            divider <= 8'b0;
//            phase <= 32'b0;
//        end else if (step_in)begin
//            phase <= phase+PHASE_INCR;
//        end
//    end
//endmodule

////6bit sine lookup, 8bit depth
//module sine_lut(input[5:0] phase_in, input clk_in, output logic[7:0] amp_out);
//  always_ff @(posedge clk_in)begin
//    case(phase_in)
//      6'd0: amp_out<=8'd128;
//      6'd1: amp_out<=8'd140;
//      6'd2: amp_out<=8'd152;
//      6'd3: amp_out<=8'd165;
//      6'd4: amp_out<=8'd176;
//      6'd5: amp_out<=8'd188;
//      6'd6: amp_out<=8'd198;
//      6'd7: amp_out<=8'd208;
//      6'd8: amp_out<=8'd218;
//      6'd9: amp_out<=8'd226;
//      6'd10: amp_out<=8'd234;
//      6'd11: amp_out<=8'd240;
//      6'd12: amp_out<=8'd245;
//      6'd13: amp_out<=8'd250;
//      6'd14: amp_out<=8'd253;
//      6'd15: amp_out<=8'd254;
//      6'd16: amp_out<=8'd255;
//      6'd17: amp_out<=8'd254;
//      6'd18: amp_out<=8'd253;
//      6'd19: amp_out<=8'd250;
//      6'd20: amp_out<=8'd245;
//      6'd21: amp_out<=8'd240;
//      6'd22: amp_out<=8'd234;
//      6'd23: amp_out<=8'd226;
//      6'd24: amp_out<=8'd218;
//      6'd25: amp_out<=8'd208;
//      6'd26: amp_out<=8'd198;
//      6'd27: amp_out<=8'd188;
//      6'd28: amp_out<=8'd176;
//      6'd29: amp_out<=8'd165;
//      6'd30: amp_out<=8'd152;
//      6'd31: amp_out<=8'd140;
//      6'd32: amp_out<=8'd128;
//      6'd33: amp_out<=8'd115;
//      6'd34: amp_out<=8'd103;
//      6'd35: amp_out<=8'd90;
//      6'd36: amp_out<=8'd79;
//      6'd37: amp_out<=8'd67;
//      6'd38: amp_out<=8'd57;
//      6'd39: amp_out<=8'd47;
//      6'd40: amp_out<=8'd37;
//      6'd41: amp_out<=8'd29;
//      6'd42: amp_out<=8'd21;
//      6'd43: amp_out<=8'd15;
//      6'd44: amp_out<=8'd10;
//      6'd45: amp_out<=8'd5;
//      6'd46: amp_out<=8'd2;
//      6'd47: amp_out<=8'd1;
//      6'd48: amp_out<=8'd0;
//      6'd49: amp_out<=8'd1;
//      6'd50: amp_out<=8'd2;
//      6'd51: amp_out<=8'd5;
//      6'd52: amp_out<=8'd10;
//      6'd53: amp_out<=8'd15;
//      6'd54: amp_out<=8'd21;
//      6'd55: amp_out<=8'd29;
//      6'd56: amp_out<=8'd37;
//      6'd57: amp_out<=8'd47;
//      6'd58: amp_out<=8'd57;
//      6'd59: amp_out<=8'd67;
//      6'd60: amp_out<=8'd79;
//      6'd61: amp_out<=8'd90;
//      6'd62: amp_out<=8'd103;
//      6'd63: amp_out<=8'd115;
//    endcase
//  end
//endmodule


//Volume Control
module volume_control (input [2:0] vol_in, input signed [7:0] signal_in, output logic signed[7:0] signal_out);
    logic [2:0] shift;
    assign shift = 3'd7 - vol_in;
    assign signal_out = signal_in>>>shift;
endmodule

//PWM generator for audio generation!
module pwm (input clk_in, input rst_in, input [7:0] level_in, output logic pwm_out);
    logic [7:0] count;
    assign pwm_out = count<level_in;
    always_ff @(posedge clk_in)begin
        if (rst_in)begin
            count <= 8'b0;
        end else begin
            count <= count+8'b1;
        end
    end
endmodule


///////////////////////////////////////////////////////////////////////////////
//
// 31-tap FIR filter, 8-bit signed data, 10-bit signed coefficients.
// ready is asserted whenever there is a new sample on the X input,
// the Y output should also be sampled at the same time.  Assumes at
// least 32 clocks between ready assertions.  Note that since the
// coefficients have been scaled by 2**10, so has the output (it's
// expanded from 8 bits to 18 bits).  To get an 8-bit result from the
// filter just divide by 2**10, ie, use Y[17:10].
//
///////////////////////////////////////////////////////////////////////////////

//module fir33(
//    input  clk_in,rst_in,ready_in,
//    input signed [7:0] x_in,
//    output logic signed [17:0] y_out = 18'd0
//);
//    logic signed [9:0] coeff;
//    logic [4:0] index = 5'd31;
//    // initialize coeffs module
//    coeffs33 my_coeffs(.index_in(index), .coeff_out(coeff));
    
//    logic signed [7:0] sample [31:0];
//    logic [4:0] offset = 5'd0;
//    // do convolution every clock and save in accumulator
//    logic signed [17:0] sum = 18'd0;
//    always_ff @(posedge clk_in) begin
//        // perform single step of convolution until done with  coeffs
//        if (ready_in) begin
//            // store incoming data at sample[offset]
//            sample[offset] <= x_in;
//            // increment offset
//            offset <= offset + 1'b1;
//            sum = 18'd0;
//            index = 5'd0;
//        end
//        else begin
//            if (index < 5'd31) begin
//                sum <= sum + coeff*sample[offset - index];      // this might need to be pipelined
//                index <= index + 1'b1;
//            end
//            else y_out <= sum;
//        end
//    end
//endmodule

/////////////////////////////////////////////////////////////////////////////////
////
//// Coefficients for a 31-tap low-pass FIR filter with Wn=.125 (eg, 3kHz for a
//// 48kHz sample rate).  Since we're doing integer arithmetic, we've scaled
//// the coefficients by 2**10
//// Matlab command: round(fir1(30,.125)*1024)
////
/////////////////////////////////////////////////////////////////////////////////

//module coeffs33(
//  input  [4:0] index_in,
//  output logic signed [9:0] coeff_out
//);
//  logic signed [9:0] coeff;
//  assign coeff_out = coeff;
//  // tools will turn this into a 31x10 ROM
//  always_comb begin
//    case (index_in)
//      5'd0:  coeff = 10'sd2;
//      5'd1:  coeff = 10'sd1;
//      5'd2:  coeff = 10'sd1;
//      5'd3:  coeff = -10'sd1;
//      5'd4:  coeff = -10'sd5;
//      5'd5:  coeff = -10'sd10;
//      5'd6:  coeff = -10'sd14;
//      5'd7:  coeff = -10'sd15;
//      5'd8:  coeff = -10'sd10;
//      5'd9:  coeff = 10'sd5;
//      5'd10: coeff = 10'sd29;
//      5'd11: coeff = 10'sd62;
//      5'd12: coeff = 10'sd99;
//      5'd13: coeff = 10'sd132;
//      5'd14: coeff = 10'sd155;
//      5'd15: coeff = 10'sd163;
//      5'd16: coeff = 10'sd155;
//      5'd17: coeff = 10'sd132;
//      5'd18: coeff = 10'sd99;
//      5'd19: coeff = 10'sd62;
//      5'd20: coeff = 10'sd29;
//      5'd21: coeff = 10'sd5;
//      5'd22: coeff = -10'sd10;
//      5'd23: coeff = -10'sd15;
//      5'd24: coeff = -10'sd14;
//      5'd25: coeff = -10'sd10;
//      5'd26: coeff = -10'sd5;
//      5'd27: coeff = -10'sd1;
//      5'd28: coeff = 10'sd1;
//      5'd29: coeff = 10'sd1;
//      5'd30: coeff = 10'sd2;
//      default: coeff = 10'hXXX;
//    endcase
//  end
//endmodule

module fir31(
  input  clk_in,rst_in,ready_in,
  input signed [7:0] x_in,
  output logic signed [17:0] y_out
);
    integer i;
    logic signed [7:0] sample [31:0];  // 32 element array each 8 bits wide
    logic [4:0] offset; //pointer for the array! (5 bits because 32 elements in above array! Do not make larger)
    logic signed [9:0] coeff;
    logic [4:0] index;
    coeffs31 conv(.index_in(index),.coeff_out(coeff));
    logic signed [17:0] accumulator;
    
    always_ff @(posedge clk_in) begin
        if (rst_in) begin
            for (i=0; i<32; i=i+1) begin
                sample[i]<=0;
            end
            offset<=0;
            index<=5'd31;
        end
        else begin
            if (ready_in) begin
                sample[offset+1]<=x_in;
                offset<=offset+1;
                index<=0;
                accumulator<=0;
            end
            else if (index<5'd31) begin
                if (index<5'd30) begin
                    accumulator<=accumulator + (coeff * sample[offset-index]);
                end
                else begin
                    y_out<=accumulator + (coeff * sample[offset-index]);
                end
                index<=index+1;
            end
        end
        
    end
endmodule





///////////////////////////////////////////////////////////////////////////////
//
// Coefficients for a 31-tap low-pass FIR filter with Wn=.125 (eg, 3kHz for a
// 48kHz sample rate).  Since we're doing integer arithmetic, we've scaled
// the coefficients by 2**10
// Matlab command: round(fir1(30,.125)*1024)
//
///////////////////////////////////////////////////////////////////////////////

module coeffs31(
  input  [4:0] index_in,
  output logic signed [9:0] coeff_out
);
  logic signed [9:0] coeff;
  assign coeff_out = coeff;
  // tools will turn this into a 31x10 ROM
  always_comb begin
    case (index_in)
      5'd0:  coeff = -10'sd1;
      5'd1:  coeff = -10'sd2;
      5'd2:  coeff = -10'sd2;
      5'd3:  coeff = 10'sd0;
      5'd4:  coeff = 10'sd5;
      5'd5:  coeff = 10'sd10;
      5'd6:  coeff = 10'sd10;
      5'd7:  coeff = 10'sd0;
      5'd8:  coeff = -10'sd19;
      5'd9:  coeff = -10'sd37;
      5'd10: coeff = -10'sd36;
      5'd11: coeff = 10'sd0;
      5'd12: coeff = 10'sd70;
      5'd13: coeff = 10'sd157;
      5'd14: coeff = 10'sd229;
      5'd15: coeff = 10'sd257;
      5'd16: coeff = 10'sd229;
      5'd17: coeff = 10'sd157;
      5'd18: coeff = 10'sd70;
      5'd19: coeff = 10'sd0;
      5'd20: coeff = -10'sd36;
      5'd21: coeff = -10'sd37;
      5'd22: coeff = -10'sd19;
      5'd23: coeff = 10'sd0;
      5'd24: coeff = 10'sd10;
      5'd25: coeff = 10'sd10;
      5'd26: coeff = 10'sd5;
      5'd27: coeff = 10'sd0;
      5'd28: coeff = -10'sd2;
      5'd29: coeff = -10'sd2;
      5'd30: coeff = -10'sd1;
      default: coeff = 10'hXXX;
    endcase
  end
endmodule


module fir32(
  input  clk_in,rst_in,ready_in,
  input signed [7:0] x_in,
  output logic signed [17:0] y_out
);
    integer i;
    logic signed [7:0] sample [31:0];  // 32 element array each 8 bits wide
    logic [4:0] offset; //pointer for the array! (5 bits because 32 elements in above array! Do not make larger)
    logic signed [9:0] coeff;
    logic [4:0] index;
    coeffs32 conv(.index_in(index),.coeff_out(coeff));
    logic signed [17:0] accumulator;
    
    always_ff @(posedge clk_in) begin
        if (rst_in) begin
            for (i=0; i<32; i=i+1) begin
                sample[i]<=0;
            end
            offset<=0;
            index<=5'd31;
        end
        else begin
            if (ready_in) begin
                sample[offset+1]<=x_in;
                offset<=offset+1;
                index<=0;
                accumulator<=0;
            end
            else if (index<5'd31) begin
                if (index<5'd30) begin
                    accumulator<=accumulator + (coeff * sample[offset-index]);
                end
                else begin
                    y_out<=accumulator + (coeff * sample[offset-index]);
                end
                index<=index+1;
            end
        end
        
    end
endmodule





///////////////////////////////////////////////////////////////////////////////
//
// Coefficients for a 31-tap low-pass FIR filter with Wn=.125 (eg, 3kHz for a
// 48kHz sample rate).  Since we're doing integer arithmetic, we've scaled
// the coefficients by 2**10
// Matlab command: round(fir1(30,.125)*1024)
//
///////////////////////////////////////////////////////////////////////////////

module coeffs32(
  input  [4:0] index_in,
  output logic signed [9:0] coeff_out
);
  logic signed [9:0] coeff;
  assign coeff_out = coeff;
  // tools will turn this into a 31x10 ROM
  always_comb begin
    case (index_in)
      5'd0:  coeff = -10'sd1;
      5'd1:  coeff = -10'sd2;
      5'd2:  coeff = -10'sd2;
      5'd3:  coeff = 10'sd0;
      5'd4:  coeff = 10'sd5;
      5'd5:  coeff = 10'sd10;
      5'd6:  coeff = 10'sd10;
      5'd7:  coeff = 10'sd0;
      5'd8:  coeff = -10'sd19;
      5'd9:  coeff = -10'sd37;
      5'd10: coeff = -10'sd36;
      5'd11: coeff = 10'sd0;
      5'd12: coeff = 10'sd70;
      5'd13: coeff = 10'sd157;
      5'd14: coeff = 10'sd229;
      5'd15: coeff = 10'sd257;
      5'd16: coeff = 10'sd229;
      5'd17: coeff = 10'sd157;
      5'd18: coeff = 10'sd70;
      5'd19: coeff = 10'sd0;
      5'd20: coeff = -10'sd36;
      5'd21: coeff = -10'sd37;
      5'd22: coeff = -10'sd19;
      5'd23: coeff = 10'sd0;
      5'd24: coeff = 10'sd10;
      5'd25: coeff = 10'sd10;
      5'd26: coeff = 10'sd5;
      5'd27: coeff = 10'sd0;
      5'd28: coeff = -10'sd2;
      5'd29: coeff = -10'sd2;
      5'd30: coeff = -10'sd1;
     // 5'd15: coeff= 12'sd1024;
      default: coeff = 12'h0;
    endcase
  end
endmodule
