`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////


module beat_detect( input clk_100mhz,
                    input valid_in,
                    input [23:0] fft_mag,
                    input [6:0] fft_index,
                    input [3:0] beat_threshold,
                    output logic beat_flag = 1'b0
    );
    
    logic [26:0] current_bass_sum = 27'd0;
    logic [26:0] prev_bass_sum = 27'd0;
    logic [26:0] threshold;
    logic [26:0] next_bass_sum = 27'd0;
    
    logic [26:0] starter = 27'd1;
    
    always_comb begin
        case (beat_threshold)
            4'd0:   threshold = starter << 5;
            4'd1:   threshold = starter << 6;
            4'd2:   threshold = starter << 7;
            4'd3:   threshold = starter << 8;
            4'd4:   threshold = starter << 9;
            4'd5:   threshold = starter >> 10;
            4'd6:   threshold = starter >> 11;
            4'd7:   threshold = starter >> 13;
            4'd8:   threshold = starter >> 14;
            4'd9:   threshold = starter >> 15;
            4'd10:  threshold = starter >> 16;
            4'd11:  threshold = starter >> 17;
            4'd12:  threshold = starter >> 18;
            4'd13:  threshold = starter >> 19;
            4'd14:  threshold = starter >> 20;
            5'd15:  threshold = starter >> 21;
            default: threshold = 3'bXXX;
        endcase
        
        next_bass_sum = current_bass_sum + fft_mag;
    end
    
    // sum bin 1 to 3
    
    always @(posedge clk_100mhz) begin
        if (valid_in) begin
            if ((fft_index > 7'd0) && (fft_index < 7'd4)) begin
                if (fft_index == 7'd3) begin
                    // end sum, compare to previous
                        if ((next_bass_sum - prev_bass_sum) > threshold) begin
                            beat_flag <= 1'b1;
                        end
                        else beat_flag <= 1'b0;
                    // make current previous
                    prev_bass_sum <= next_bass_sum;
                    current_bass_sum <= 27'd0;
                end
                else begin
                    current_bass_sum <= current_bass_sum + fft_mag;
                    beat_flag <= 1'b0;
                end
            end
            else beat_flag <= 1'b0;
        end
        else beat_flag <= 1'b0;
    end
    
endmodule
