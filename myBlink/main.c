#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "myblink.pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define NUM_SAMPLES 1024
// Here's where we'll have the DMA channel put ADC samples
uint16_t sample_array[NUM_SAMPLES] ;
uint16_t * sample_address_pointer = &sample_array[0] ;

// Helper function (for use in C program) to initialize this PIO program
void myblink_program_init(PIO pio, uint sm, uint offset, uint pin, float div, uint inPinBase) {

    // Sets up state machine and wrap target. This function is automatically
    // generated in blink.pio.h.
    pio_sm_config c = myblink_program_get_default_config(offset);

    // Allow PIO to control GPIO pin (as output)
    pio_gpio_init(pio, pin);

    // Connect pin to SET pin (control with 'set' instruction)
    sm_config_set_set_pins(&c, pin, 1);

    // Set the pin direction to output (in PIO)
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    sm_config_set_in_pins(&c, inPinBase);



    pio_sm_set_consecutive_pindirs(pio, sm, inPinBase, 10, false);

    sm_config_set_in_shift(&c, false, true, 10);
    sm_config_set_fifo_join(&c,PIO_FIFO_JOIN_RX);
    // Set the clock divider for the state machine
    sm_config_set_clkdiv(&c, div);

    // Load configuration and jump to start of the program
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

}

int main() {

    int myClk = 280000;
    set_sys_clock_khz(myClk, true);
    stdio_init_all();

    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    uint sm = pio_claim_unused_sm(pio, true);


    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset = pio_add_program(pio, &myblink_program);

    int clkPin = 15;
    int dataPin = 2;
    //sleep_ms(3000);

    //////////////////////////////////////DMA/////////////////////////////////////

    int dIN_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dIN_dma_chan_config = dma_channel_get_default_config(dIN_dma_chan); //get default config
    channel_config_set_transfer_data_size(&dIN_dma_chan_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dIN_dma_chan_config, false);
    channel_config_set_write_increment(&dIN_dma_chan_config, true);
    channel_config_set_dreq(&dIN_dma_chan_config, DREQ_PIO0_RX0);

    dma_channel_configure(
        dIN_dma_chan,
        &dIN_dma_chan_config,
        sample_address_pointer,
        &pio0_hw->rxf[0],
        1024,
        false
    );

    ///////////////////////////////DMA Control Chan//////////////////////////////

    int controlChan = dma_claim_unused_channel(true);
    dma_channel_config controlChan_config = dma_channel_get_default_config(controlChan);
    channel_config_set_transfer_data_size(&controlChan_config, DMA_SIZE_32);
    channel_config_set_read_increment(&controlChan_config, false);
    channel_config_set_write_increment(&controlChan_config, false);
    channel_config_set_chain_to(&controlChan_config, dIN_dma_chan);

    dma_channel_configure(
        controlChan,
        &controlChan_config,
        &dma_hw->ch[dIN_dma_chan].write_addr,
        &sample_address_pointer,
        1,
        false
    );



    /////////////////////////////////////////////////////////////////////////////



    sleep_ms(4000);

    printf("Starting DMA...\n");

    //dma_channel_start(dIN_dma_chan);
    dma_start_channel_mask((1u << dIN_dma_chan)) ;    

    printf("Starting PIO...\n");

    myblink_program_init(pio, sm, offset, clkPin, 1.75, dataPin);
    printf("Stoping PIO...\n");


    dma_channel_wait_for_finish_blocking(dIN_dma_chan);

    //pio_sm_set_enabled(pio, sm, false);
    // printf("myBlink init with PIO: %d, SM: %d\n", pio, sm);
    printf("DMA started on channel: %d\n", dIN_dma_chan);

    dma_channel_start(controlChan);



    // dma_channel_set_write_addr(dIN_dma_chan, sample_address_pointer, true);
    printf("Restarted DMA\n");

    // //dma_channel_start(dIN_dma_chan);

    //pio_restart_sm_mask(pio, 1);


    // printf("Restarted Sm \n");




    for (int i = 0; i < 1024; i++) {
        printf("Index:%d, Value: %d \n", i, sample_array[i]);       
    }




    while(1) {
        dma_channel_wait_for_finish_blocking(dIN_dma_chan);

        //pio_sm_set_enabled(pio, sm, false);
        // printf("myBlink init with PIO: %d, SM: %d\n", pio, sm);
//        printf("DMA started on channel: %d\n", dIN_dma_chan);

        dma_channel_start(controlChan);



        // dma_channel_set_write_addr(dIN_dma_chan, sample_address_pointer, true);
//        printf("Restarted DMA\n");

        // //dma_channel_start(dIN_dma_chan);

        //pio_restart_sm_mask(pio, 1);
        tight_loop_contents();
        // sleep_ms(1);
        
    }



}


/*
// static inline int32_t read_RxFIFO(PIO pio, uint sm) {
//     // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
//     io_rw_32 *rxfifo_shift = (io_rw_32*)&pio->rxf[sm];
//     while (pio_sm_is_rx_fifo_empty(pio, sm))
//         tight_loop_contents();
//     return (int32_t)*rxfifo_shift;
// }


// int main() {
//     // static const float pio_freqKHz = 80000;
//     // int myClkFreqKHz = 3*pio_freqKHz;
//     // set_sys_clock_khz(myClkFreqKHz, true); // 158us

//     int myClk = 280000;
//     set_sys_clock_khz(myClk, true);


//     static const uint led_pin = 15;
//     static const float pio_freq = 2000;


//     int sample_chan = 2 ;
//     int control_chan = 3 ;

//     // Channel configurations
//     dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
//     dma_channel_config c3 = dma_channel_get_default_config(control_chan);


//     // ADC SAMPLE CHANNEL
//     // Reading from constant address, writing to incrementing byte addresses
//     channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
//     channel_config_set_read_increment(&c2, false);
//     channel_config_set_write_increment(&c2, true);
//     // Pace transfers based on availability of ADC samples
//     channel_config_set_dreq(&c2, DREQ_PIO0_RX0);
//     // Configure the channel

//     dma_channel_configure(sample_chan,
//         &c2,            // channel config
//         sample_array,   // dst
//         pio_sm_get,  // src
//         NUM_SAMPLES,    // transfer count
//         false            // start immediately
//     );

//     // CONTROL CHANNEL
//     channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);              // 32-bit txfers
//     channel_config_set_read_increment(&c3, false);                        // no read incrementing
//     channel_config_set_write_increment(&c3, false);                       // no write incrementing
//     channel_config_set_chain_to(&c3, sample_chan);

//     dma_channel_configure(
//         control_chan,                         // Channel to be configured
//         &c3,                                // The configuration we just created
//         &dma_hw->ch[sample_chan].write_addr,  // Write address (channel 0 read address)
//         &sample_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
//         1,                                  // Number of transfers, in this case each is 4 byte
//         false                               // Don't start immediately.
//     );


//     // Choose PIO instance (0 or 1)
//     PIO pio = pio0;

//     // Get first free state machine in PIO 0
//     uint sm = pio_claim_unused_sm(pio, true);
//     //uint sm2 = pio_claim_unused_sm(pio,  true);

//     // Add PIO program to PIO instruction memory. SDK will find location and
//     // return with the memory offset of the program.
//     uint offset = pio_add_program(pio, &myblink_program);
//     //uint offset2 = pio_add_program(pio, &myblink_program);

//     stdio_init_all();


//     sleep_ms(3000);


//     dma_start_channel_mask((1u << sample_chan)) ;



//     // Calculate the PIO clock divider
//     // float div = (float)clock_get_hz(clk_sys) / pio_freq;
//     // float div = (float) myClkFreqKHz / pio_freqKHz;

//     // Initialize the program using the helper function in our .pio file
//     myblink_program_init(pio, sm, offset, led_pin, 1.75, 2);
//     //myblink_program_init(pio, sm2, offset2, 16, 10, 2);
//     // Start running our PIO program in the state machine
//     // pio_sm_set_enabled(pio, sm, true);
//     // pio_sm_set_enabled(pio, sm2, true);

//     pio_enable_sm_mask_in_sync(pio, 3);










//     // int getVal = read_RxFIFO(pio, sm);
//     // Do nothing
//     int32_t prevVal = 0;
//     while (true) {


//         // Wait for NUM_SAMPLES samples to be gathered
//         // Measure wait time with timer
//         dma_channel_wait_for_finish_blocking(sample_chan);

//         printf("%d \n", sample_array[0]);

//         // Restart the sample channel, now that we have our copy of the samples
//         dma_channel_start(control_chan) ;
//         // int getVal = 7;
//         // int32_t getVal = read_RxFIFO(pio, sm);
//         // for (int i = 0; i < 1080; i++){
//         //     getVal = read_RxFIFO(pio, sm);            
//         // }

//         // if (prevVal != getVal) {
//         //     prevVal = getVal;
//         //     printf(" Value: %d\n",  getVal);
//         // }
        
//     }
// }
*/