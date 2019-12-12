`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: MIT 6.111 Final Project
// Engineer: Rhian Chavez
//////////////////////////////////////////////////////////////////////////////////


module rgb_to_lights(   input clk_100mhz,          
                        input [23:0] rgb,
                        input new_rgb,              // pulse
                        output logic data_stream,
                        output logic clk_stream
    );
    
    // should take ~ 4ms to update the entire LED strip
    
    // use BRAM to continuously store new rgb data in correct bin, read and update data stream on 
    
    //logic write_mem;                      // enable write to lights, maybe just make new_rgb
    logic [23:0] mem_data;                  // what is actually sent to lights
    logic [6:0] mem_address;                // switch between write_led_count and read_led_count when necessary
    
    // read and write address
    logic [6:0] write_led_count = 7'd0;         
    logic [6:0] read_led_count = 7'd0;      // first and last are special, rest are data
    
    // 2 CYCLE LATENCY
    // port A is write
    // port B is read
    blk_mem_gen_0 memory ( .clka(clk_100mhz), .clkb(clk_100mhz), .addra(write_led_count), .addrb(read_led_count-1'b1), .dina(rgb), .doutb(mem_data), .wea(new_rgb) );
    
    parameter read_led_count_max = 7'd73;   // first and last are special, need to subtract 1 when comparing to write logic
    parameter write_led_count_max = 7'd71;
    parameter clk_period = 8'd200;          // down convert clk to 500 kHz
    parameter half_clk_period = 8'd100;
    
    logic [31:0] current_rgb;               // hold read data, shift as going down
    // for now, start each stream with red_green_blue_0xff 
    logic [4:0] shift_count = 5'd0;         // count how far into current rgb data               
    
    // down convert clock to reasonable streaming frequency
    logic [7:0] clk_count = 8'd0;
    
    always_comb begin
        // update data stream
        data_stream = current_rgb[0];
    end
    
    logic [7:0] flip_red;
    logic [7:0] flip_green;
    logic [7:0] flip_blue;
    assign flip_red = {<<{mem_data[23:16]}};
    assign flip_green ={<<{mem_data[15:8]}};
    assign flip_blue = {<<{mem_data[7:0]}};
    
    // need to give clock 50% duty cycle and transition output data on falling edge
    
    always @(posedge clk_100mhz) begin
        // write logic
        if (new_rgb) begin
            // increment write address
            if (write_led_count == write_led_count_max) begin
                write_led_count <= 7'd0;
            end
            else begin
                write_led_count <= write_led_count + 1'b1;
            end
        end
        
        // read logic
        if (clk_count <= half_clk_period) begin
            // always incrment clk_count
            clk_count <= clk_count + 1'b1;
            if (clk_count == half_clk_period) begin
                // rising edge of clk, don't touch data
                clk_stream <= 1'b1;
            end
        end
        else begin
            if (clk_count == clk_period) begin
                // falling edge of clk, reset clk_counter
                clk_stream <= 1'b0;
                clk_count <= 8'd0;
                // if shift count is over, get new data and present 1'b1 to data_out
                if (shift_count == 5'd0) begin
                    // read from RAM 
                    if (read_led_count == 7'd0) begin
                        // don't need to read, rgb should be all zeros
                        current_rgb <= 32'd0;
                        // increment read_led_count
                        read_led_count <= read_led_count + 1'b1;
                    end
                    else if (read_led_count == 7'd73 ) begin
                        // dont' need to read, rgb should be all ones
                        current_rgb <= 32'hff_ff_ff_ff;
                        // reset read_led_count
                        read_led_count <= 7'd0;
                    end
                    else begin
                        // read and increment led_count
                        current_rgb <= {flip_red, flip_green, flip_blue, 8'b1111_1111};      // this ordering was messed up before
                        read_led_count <= read_led_count + 1'b1;
                    end
                    shift_count <= shift_count + 1'b1;
                end
                else if (shift_count == 5'd31) begin
                    // reset shift count 
                    shift_count <= 5'd0;
                    // slide in last data
                    current_rgb <= {1'b0, current_rgb[31:1]};
                end
                else begin
                    // shift data
                    current_rgb <= {1'b0, current_rgb[31:1]};
                    // increment shift count
                    shift_count <= shift_count + 1'b1;
                end
            end
            else begin
                // increment clk_count
                clk_count <= clk_count + 1'b1;
            end
        end
    end
    
    
    
endmodule
