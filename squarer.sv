`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: MIT 6.111 Final Project
// Engineer: Rhian Chavez
//////////////////////////////////////////////////////////////////////////////////


module squarer(     input clk_100mhz,
                    input logic valid_in,
                    input [15:0] real_part,
                    input [15:0] imag_part,
                    output logic out_valid = 1'b0,
                    output logic [32:0] sum
    );
    
    // 3 clock cycle latency & 1 clock cycle throughput
    
    // instead use a shifter for position validity, 1 is valid, 0 is invalid
    logic [2:0] validity = 3'b000;
    
    // intialize intermediate logic
    logic [15:0] real_choice;
    logic [15:0] imag_choice;
    
    logic [31:0] sq_real;
    logic [31:0] sq_imag;
    
    
    always @(posedge clk_100mhz) begin
        // transition validity vector
        validity <= {valid_in, validity[2:1]};
        
        // if validity[1] is non-zero, then a new output will be ready
        out_valid <= validity[1];
        
        // transition stuff if there is valid data to be carried, otherwise sit still 
        if (validity[2:1]>2'd0 || valid_in) begin
            // get magnitude of real numbers
            if (real_part[15] == 1'b1) begin
                real_choice <= ({1'b0,~real_part[14:0]})+16'd1;
            end
            else begin
                real_choice <= {1'b0, real_part[14:0]};
            end
            
            if (imag_part[15] == 1'b1) begin
                imag_choice <= ({1'b0,~imag_part[14:0]})+16'd1;
            end
            else begin
                imag_choice <= {1'b0, imag_part[14:0]};
            end
            
            // square them
            sq_real <= real_choice**2;
            sq_imag <= imag_choice**2;
            
            // sum of squares to output
            sum <= sq_real + sq_imag;
        end        
        
    end
    
endmodule
