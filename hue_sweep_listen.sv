`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: MIT 6.111 Final Project
// Engineer: Rhian Chavez
//////////////////////////////////////////////////////////////////////////////////


module hue_sweep_listen(    input logic clk_100mhz,
                            input logic new_flag,               // pulse
                            input logic [24:0] freq_mag,        // assume only bottom 8 bits matter
                            input logic [7:0] threshold,        // only modify value for LED if incoming freq_mag is greater
                            input beat_in,
                            output logic [8:0] hue = 9'd0,      // max is 0d359 (cyclic)
                            output logic [7:0] val_out,         // max is 0xff
                            output logic out_ready = 1'b0       // pulse       
    );
    
    // outputs both hue and value to HSV -> RGB stage (saturation always assumed to be maximum)
    
    // we will always assume maximum saturation
    // hue and value (brightness) will differ
    logic [7:0] relevant_freq_mag;
    
    assign relevant_freq_mag = freq_mag[7:0];           // only bottom 8 because SD data is only 8 bit
    // only change data in RAM if relevant_freq_mag > threshold (make threshold changeable with switches)
    
    // assume first data received is first LED (needs to be taken care of in FFT to LED mapper)
    logic [6:0] write_mem_address;
    logic [7:0] val_in;
    logic [6:0] led_count = 7'd0;                   // also serves as read mem_address
    logic [26:0] hue_clk_count = 27'd0;
    logic [11:0] decay_clk_count = 12'd0;
    logic [6:0] input_counter = 7'd0;
    
    // initiate value RAM in order to have memory (decrease brightness over time)
    blk_mem_gen_1 value_mem ( .clka(clk_100mhz), .clkb(clk_100mhz), .addra(write_mem_address), .addrb(led_count), .dina(val_in), .doutb(val_out), .wea(write_mem) );
    
    // wait for first data received
    parameter max_hue_clk_count = 27'd10_000_000;       // make freq ~10Hz, should take 36 seconds to go around wheel
    parameter hue_max = 9'd359;                        // 360 degree color wheel
    // step through LED count every time decay_count clk is reached
    parameter led_count_max = 7'd71;                   // 72 LEDs per strip
    // upon reaching this clk_count check memory and decay value if non-zero, also spit out new value to next step
    parameter decay_count_max = 12'd2500;              // every 250us decay a single LED, should take ~0.5 seconds to decay ALL to zero
    
    assign write_from_input = new_flag && (relevant_freq_mag > threshold);
    assign write_from_decay = (decay_clk_count == decay_count_max);
    assign write_mem = (write_from_input) || (write_from_decay);
    
    always_comb begin
        // assign proper val_in -- WRITING LOGIC
        if (write_from_input) begin
            val_in = relevant_freq_mag;
            // assign write address from input counter
            write_mem_address = input_counter;
        end
        else if (write_from_decay) begin
            // change write address to led_count
            write_mem_address = led_count;
            // only decrease if non-zero
            if (val_out > 8'd0) begin
                val_in = val_out - 1'b1;
            end
            else val_in = 8'd0;
        end
        
        if (write_from_decay) out_ready = 1'b1;
        else out_ready = 1'b0;
    end
    
    always @(posedge clk_100mhz) begin
        if (beat_in) begin
            // beat detected, jump 60 in hue
            if (hue < 9'd300) begin
                hue <= hue + 9'd60;
            end
            else begin
                hue <= 9'd60 - (9'd360 - hue);
            end
        end
        // constantly hue sweep
        else if (hue_clk_count == max_hue_clk_count ) begin
            // increment hue
            if (hue == hue_max ) begin
                // reset to zero
                hue <= 9'd0;
            end
            else begin
                // increment up
                hue <= hue + 1'b1;
            end
            // reset clk_count
            hue_clk_count <= 27'd0;
        end
        else begin
            // increment clk_count
            hue_clk_count <= hue_clk_count + 1'b1;
        end
        
        // decay_count reached
        if (decay_clk_count == decay_count_max) begin
            // set decay count back to zero
            decay_clk_count <= 12'd0;

            // going to need to increment to next LED regardless
            if (led_count == led_count_max) begin
                led_count <= 7'd0;
            end
            else begin
                led_count <= led_count + 1'b1;
            end 
        end
        else begin
            // increment decay count
            decay_clk_count <= decay_clk_count + 1'b1;
        end
        
        // count up from data received
        if (new_flag) begin
            if (input_counter == led_count_max ) begin
                input_counter <= 7'd0;
            end
            else begin
                input_counter <= input_counter + 1'b1;
            end
        end
    end
    
endmodule
