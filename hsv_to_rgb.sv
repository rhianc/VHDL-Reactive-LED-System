`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: MIT 6.111 Final Project
// Engineer: Rhian Chavez
//////////////////////////////////////////////////////////////////////////////////


module hsv_to_rgb(  input clk_100mhz,
                    input [8:0] hue,
                    input [7:0] val,
                    input valid_in,
                    output logic [23:0] rgb,
                    output logic valid_out
    );
    
    // always assume saturation is maximum
    
    logic [2:0] h_prime;
    logic [20:0] x;
    logic signed [9:0] mod;
    logic signed [9:0] half = 10'd60;
    logic signed [9:0] difference;
    logic signed [9:0] signed_hue;
    logic [9:0] mag_diff;
    logic [9:0] scalar;
    
    assign signed_hue = {1'b0,hue};
    
    always_comb begin
        // determine which section of color wheel
        if (hue < 9'd60) begin
            h_prime = 3'd1;
            mod = signed_hue;
        end
        else if ((9'd59 < hue)&&(hue < 9'd120)) begin
            h_prime = 3'd2;
            mod = signed_hue;
        end
        else if ((9'd119 < hue)&&(hue < 9'd180)) begin
            h_prime = 3'd3;
            mod = signed_hue - 9'd120;
        end
        else if ((9'd179 < hue)&&(hue < 9'd240)) begin
            h_prime = 3'd4;
            mod = signed_hue - 9'd120;
        end
        else if ((9'd239 < hue)&&(hue< 9'd300)) begin
            h_prime = 3'd5;
            mod = signed_hue - 9'd240;
        end
        else begin
            // won't be any greater than 359
            h_prime = 3'd6;
            mod = signed_hue - 9'd240;
        end
        
    end
    
    logic [14:0] stages = 15'd0;
    logic [39:0] val_pass = 40'd0;
    logic div_valid;
    logic [31:0] div_out;
    logic [20:0] rel_out;
    assign rel_out = div_out[28:8];     // don't care about start padding or remainder shit
    logic signed [9:0] mid_mod;
    
    div_gen_0 my_div (  .s_axis_divisor_tdata(8'd60), .s_axis_divisor_tvalid(1'b1), .s_axis_dividend_tdata({3'd0, x}), .s_axis_dividend_tvalid(1'b1),
                                .aclk(clk_100mhz), .m_axis_dout_tdata(div_out), .m_axis_dout_tvalid(div_valid));
    
    // do things combinationally instead to ensure no timing problems
    always @(posedge clk_100mhz) begin
        if (valid_in ||  (stages > 15'd0)) begin
            // move forward stages
            if (valid_in) begin
                stages <= {h_prime[2:0], stages[14:3]};
                val_pass <= {val[7:0], val_pass[39:8]};
            end
            else begin
                stages <= {3'd0, stages[14:3]};
                val_pass <= {8'd0, val_pass[39:8]};
            end
            mid_mod <= mod;
            // first stage
            difference <= mid_mod - half;
            // second stage
            if (difference[9] == 1'b1) begin
                // negative number
                scalar <= 9'd59 - (~difference);
            end
            else begin
                // positive number
                scalar <= 9'd60 - difference;
            end
            // third stage
            x <= val_pass[23:16]*scalar;    // divide this by 60 before next stage
            // fourth stage (divide by 60)
                // do nothing but wait single cycle latency for divider output
            // fifth stage (pass to output)
            case (stages[2:0])
                3'd1:   rgb <= {val_pass[7:0], rel_out[7:0], 8'd0};
                3'd2:   rgb <= {rel_out[7:0], val_pass[7:0], 8'd0};
                3'd3:   rgb <= {8'd0, val_pass[7:0], rel_out[7:0]};
                3'd4:   rgb <= {8'd0, rel_out[7:0], val_pass[7:0]};
                3'd5:   rgb <= {rel_out[7:0], 8'd0, val_pass[7:0]};
                3'd6:   rgb <= {val_pass[7:0], 8'd0, rel_out[7:0]};
                default: rgb <= 24'd0;
            endcase
            
        end
        if (stages[2:0] != 3'd0) valid_out <= 1'b1;
        else valid_out <= 1'b0;
    end
  
endmodule
