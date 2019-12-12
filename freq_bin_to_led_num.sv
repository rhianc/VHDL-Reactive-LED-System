`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: MIT 6.111 Final Project
// Engineer: Rhian Chavez
//////////////////////////////////////////////////////////////////////////////////


module freq_bin_to_led_num( input clk_100mhz,
                            input [23:0] fft_mag,
                            input [6:0] freq_index = 7'd0,
                            input valid_in,
                            output logic valid_out,
                            output logic [24:0] led_mag_out
    );
    // worst case latency 2 clk cycles, best case 1 clk cycle. 
    
    
    // 128 bin FFT input
    // 72 bin LED output
    
    // move DC to center
    // start counter so that first stuff transmitted is highest frequency (bin 65)
    logic started = 1'b0;
    logic added = 1'b0;
    logic [23:0] holder = 24'd0;
    
    // wait until highest frequency content bin arrives, send that out first
    
    // if using high frequency content, sum two together and send out
    // if using low frequency content, only use 1 and send out
    always @(posedge clk_100mhz) begin
        if (valid_in) begin
            if (started == 1'b1) begin
                // typical operation
                if ((freq_index > 7'd8)&&(freq_index < 7'd121)) begin
                    // double count
                    if (~added) begin
                        holder <= (fft_mag>>1);
                        added <= added + 1'b1;
                        valid_out <= 1'b0;
                    end
                    else begin
                        led_mag_out <= holder + (fft_mag>>1);
                        added <= added + 1'b1;
                        valid_out <= 1'b1;
                    end
                end
                else begin
                    valid_out <= 1'b1;
                    // single count
                    if (freq_index == 7'd0) begin
                        led_mag_out <= 25'd0;
                    end
                    else begin
                        led_mag_out <= fft_mag; 
                    end
                end 
            end
            if (freq_index == 7'd63) begin
                // only start transmitting stuff after moved to DC since we want center to have DC component
                started <= 1'b1;
            end
        end
        else begin
            valid_out <= 1'b0;
        end
        
    end
    
endmodule
