//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/12/2019 02:46:57 PM
// Design Name: 
// Module Name: SD_interface
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module SD_interface(input CLK100MHZ, input SD_CD, output SD_RESET, output SD_SCK, output SD_CMD, 
	inout [3:0] SD_DAT, input BTNC, input wr, input[7:0] din, input rd, output logic[7:0] data_out
	,output logic ready_for_next_byte, input ready_for_next_block, output logic starting_next_block, input BTNU);
    
    logic last_BTNU;
    logic last_last_BTNU;
    logic pos_BTNU;
    assign pos_BTNU=last_BTNU&&!last_last_BTNU;
    // Clock the SD card at 25 MHz.
	wire clk_100mhz = CLK100MHZ;
    wire clk_50mhz;
    wire clk_25mhz;
    clock_divider div1(clk_100mhz, clk_50mhz);
    clock_divider div2(clk_50mhz, clk_25mhz);

    wire rst = BTNC;
    wire spiClk;
    wire spiMiso;
    wire spiMosi;
    wire spiCS;

    // MicroSD SPI/SD Mode/Nexys 4
    // 1: Unused / DAT2 / SD_DAT[2]
    // 2: CS / DAT3 / SD_DAT[3]
    // 3: MOSI / CMD / SD_CMD
    // 4: VDD / VDD / ~SD_RESET
    // 5: SCLK / SCLK / SD_SCK
    // 6: GND / GND / - 
    // 7: MISO / DAT0 / SD_DAT[0]
    // 8: UNUSED / DAT1 / SD_DAT[1]
    assign SD_DAT[2] = 1;
    assign SD_DAT[3] = spiCS;
    assign SD_CMD = spiMosi;
    assign SD_RESET = 0;
    assign SD_SCK = spiClk;
    assign spiMiso = SD_DAT[0];
    assign SD_DAT[1] = 1;
    
    //SD ready to read/write signal
    wire ready;
    //SD address to read/write from/to
    reg [31:0] adr;
    //status for debugging
    wire [4:0] status;
    
    //read command signal to SD card.
    //I make this different from rd, which is wired to BTNL, so that I can control the timing better
    logic RD;
    logic WR;
    
    //direct output from SD card
    logic[7:0] dout;
    //SD signal for reading byte available
    logic byte_available;
	
    sd_controller sdcont(.cs(spiCS), .mosi(spiMosi), .miso(spiMiso),
            .sclk(spiClk), .rd(RD), .wr(WR), .reset(rst),
            .din(din), .dout(dout), .byte_available(byte_available),
            .ready(ready), .address(adr), 
            .ready_for_next_byte(ready_for_next_byte), .clk(clk_25mhz), 
            .status(status));
     
     //Beginning of state machine logic       
     parameter writing=2'b01, reading=2'b10, idle=2'b00;
     logic[1:0] state;
     
     //counter is used while reading and writing to monitor time
     //each 512 byte block is taking
     logic[23:0] counter;
     //cycle_delay is used while reading to support the writing
     //of the timing of each 512 byte block to the BRAM.
     //cycle_delay is used while writing to take the recorded
     //times from BRAM and stall the calling of the next 512 byte block
     //until each of those times has passed.
     logic[23:0] cycle_delay;
     //BRAM records values in bytes, but the timing is a 24 bit piece of data.
     //The following three variables are used to divide and save these 24 bit data
     //in three parts.
     logic part2;
     logic part3;
     logic[1:0] updater;
     
     //The reading state operates by reading a 512 byte block, and spacing out
     //each of those bytes across the corresponding time read from BRAM. It sends
     //bytes as it is reading the 512 block, so it relies on an observation that
     //reading from the SD card is faster than writing (i.e. we would have
     //a problem if FIFO_out>FIFO_in). The following four variables are used to 
     //implement the FIFO.
     logic[7:0] FIFO1[511:0];
     logic[7:0] FIFO2[511:0];
     integer i;
     logic[9:0] FIFO_out;
     logic[9:0] FIFO_in;
     logic[23:0] delayer;
     
     //BRAM to record time for each 512 block.
     logic [7:0] data_to_bram;
     logic [7:0] data_from_bram;
     logic [15:0] addr;
     logic bram_write;
     blk_mem_gen_2(.addra(addr), .clka(clk_25mhz), .dina(data_to_bram), .douta(data_from_bram), 
                   .ena(1), .wea(bram_write));                                  
    
     
     always @(posedge clk_25mhz) begin
        if(rst) begin
            adr<= 32'h00_00_00_00;
            state<=idle;
        end
        else begin
            case(state)
                idle: begin
                    //If want to read and SD card is ready,
                    //call ready, go to reading state, and initialize variables
                    if (rd==1 && ready) begin
                        RD<=1;
                        adr<= 0;
                        //For some reason the first value written always appears on addr=2 when reading.
                        //Setting addr to 1 here lets me read that first value first
                        addr<=1;
                        
                        state<=reading;
                        updater<=2'b00;
                        part2<=0;
                        part3<=0;
                    end
                    //Since wr is directly wired to the SD, if we enter the next condition holds the SD has begun writing, hence the updated adr.
                    else if (wr==1 && ready) begin
                        state<=writing;
                        adr<= 10'd512;
                    end
                    //Otherwise keep variables initialized
                    else begin
                        state<=idle;
                        RD<=0;
                        counter<=0;
                        for(i=0; i<512; i=i+1) begin
                            FIFO1[i]<=0;
                            FIFO2[i]<=0;
                        end
                        FIFO_out<=0;
                        FIFO_in<=0;
                        
                        addr<=0;
                        bram_write<=0;
                        part2<=0;
                        part3<=0;
                        updater<=2'b00;
                        
                        //This is the highest value so that, when we start reading, the module starts by
                        //assuming it took a bunch of time to read the first block and does not move on before
                        //cycle_delay is updated.
                        cycle_delay<=24'b1111_1111_1111_1111_1111_1111;
                        delayer<=24'b1111_1111_1111_1111_1111_1111;
                        
                    end
                end
                reading: begin
                    if (rd==0) begin
                        state<=idle;
                        adr<= 32'h00_00_00_00;
                    end
                    //move to the next SD block if the SD is ready and
                    //the current block has taken exactly the same time to read as it did to write
                    else if (ready && (counter == delayer)) begin
                    //else if (ready && (counter == 23'd524288)) begin
                        adr<= adr+10'd512;
                        RD<=1;
                        counter<=0;
                        FIFO_in<=0;
                        FIFO_out<=0;
                        
                        FIFO2<=FIFO1;
                        delayer<=cycle_delay;//23'd524288;
                        
                        //triggers updating cycle_delay from bram
                        updater<=2'b00;
                    end
                    else begin
                        last_BTNU<=BTNU;
                        last_last_BTNU<=last_BTNU;
        
                        RD<=0;
                        counter<=counter+1;
                        
                        if (pos_BTNU && counter>23'd256) begin
                            addr<=addr-8'd150;
                            adr<=adr-15'd25600;
                        end
                        
                        //The updater logic does the following:
                        //  change input address
                        //  wait to let data_from_bram update
                        //  keep track of which part of the 24 bit time value we are on, and assing to corresponding part of cycle_delay
                        //  go to next part of 24 bit value, or do nothing if finished updating cycle_delay
                        case(updater)
                            2'b00: begin
                                addr<=addr+1;
                                updater<=2'b01;
                            end
                            2'b01: updater<=2'b11;
                            2'b11: begin
                                if (part3) begin
                                    cycle_delay[7:0]<=data_from_bram;
                                    
                                    updater<=2'b10;
                                    part2<=0;
                                    part3<=0;
                                end
                                else if (part2) begin
                                    cycle_delay[15:8]<=data_from_bram;
                                    part3<=1;
                                    updater<=2'b10;
                                end
                                else begin
                                    cycle_delay[23:16]<=data_from_bram;
                                    part2<=1;
                                    updater<=2'b10;
                                end
                            end
                            2'b10: begin
                                if(part2) begin
                                    updater<=2'b00;
                                end
                                else begin
                                    updater<=updater;
                                end
                            end
                            default: begin
                                cycle_delay<=24'b1111_1111_1111_1111_1111_1111;
                                part2<=0;
                                part3<=0;
                                updater<=2'b10;
                                adr<=0;
                                addr<=0;
                            end
                        endcase
                    end
                    //Each byte in 512 byte sequence should fill up FIFO queue
                    if(byte_available) begin
                        FIFO1[FIFO_in]<=dout;
                        FIFO_in<=FIFO_in+1;
                    end
                    //Divide cycle_delay into 512 time steps, one for each byte
                    if(counter==((delayer>>9)*(FIFO_out+1)) && FIFO_out<10'd512) begin
                        data_out<=FIFO2[FIFO_out];
                        FIFO_out<=FIFO_out+1;
                    
                    end
                    
                end
                writing: begin
                    if (wr==0) begin
                        state<=idle;
                        adr<= 32'h00_00_00_00;
                        bram_write<=0;
                    end
                    //If starting next 512 byte block, set up for next block, and record time to bram
                    else if (ready && ready_for_next_block) begin
                        starting_next_block<=1;
                        WR<=1;
                        
                        //when ready, starts writing to current value of adr, so updating for next time ready
                        adr<= adr+10'd512;
                        
                        //BRAM stuff
                        data_to_bram<=counter[23:16];
                        addr<=addr+1;
                        part2<=1;
                        updater<=2'b01;
                        //BRAM stuff end
                        
                        cycle_delay<=counter;
                        counter<=0;
                    end
                    else begin
                        starting_next_block<=0;
                        WR<=0;
                        counter<=counter+1;
                        
                        case(updater)
                            2'b00: begin
                                if (part3) begin
                                    data_to_bram<=cycle_delay[7:0];
                                    addr<=addr+1;
                                    part2<=0;
                                    part3<=0;
                                    updater<=2'b01;
                                end
                                else if (part2) begin
                                    data_to_bram<=cycle_delay[15:8];
                                    addr<=addr+1;
                                    part3<=1;
                                    updater<=2'b01;
                                end
                                else begin
                                    updater<=updater;
                                end
                            end
                            2'b01:  updater<=2'b11;
                            2'b11: begin
                                bram_write<=1;
                                updater<=2'b10;
                            end
                            2'b10: begin
                                bram_write<=0;
                                updater<=2'b00;
                            end
                            default: begin
                                bram_write<=0;
                                addr<=0;
                                adr<=0;
                                updater<=2'b00;
                            end
                        endcase
                    end
                end
            endcase
        end
    end
endmodule

module clock_divider(input clk_in, output reg clk_out = 0);
	always @(posedge clk_in) begin
		clk_out <= ~clk_out;
	end
endmodule
