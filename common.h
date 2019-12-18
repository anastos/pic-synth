#ifndef COMMON_H
#define	COMMON_H

#include "config_1_3_2.h"
#include "math.h"

#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits

#define PRESSED(b) ((btn_st.i & b) && !(btn_st_prev.i & b))

#define DAC_CONFIG_CHAN_A 0b0011000000000000
#define DISABLE_ISR INTEnable(INT_T2, 0)
#define ENABLE_ISR INTEnable(INT_T2, 1)

#define MAX_RECORDED_NOTES 256
#define NUM_NOTES 18
#define SINES_PER_NOTE 4
#define SINE_TABLE_SIZE 512
#define ENV_TABLE_SIZE 512
#define REL_TABLE_SIZE 32

// useful for sending ints over UART
typedef union { int i; char c[4]; } char_int;

struct Note {
    // 0: button not pressed
    // 1: button pressed
    // 2: button just released
    volatile char state;

    // inc ~ frequency, idx: current location in sine table
    _Accum inc[SINES_PER_NOTE], idx[SINES_PER_NOTE];

    // curent indices in envelope tables
    int env_idx, rel_idx;
};

// frequencies of every note relative to the lowest
// base_freqs[i] = 2^(i/12)
const _Accum base_freqs[] = {1, 1.0595, 1.1225, 1.1892, 1.2599, 1.3348,
        1.4142, 1.4983, 1.5874, 1.6818, 1.7818, 1.8877, 2,
        2.1189, 2.2449, 2.3784, 2.5198, 2.6697};

// harmonic frequencies for each sound type
const _Accum freq_ratios[3][SINES_PER_NOTE] = {
    {1, 2, 3, 4}, // piano
    {1, 1.5, 3, 6}, // organ
    {1, 2, 3, 4} // guitar
};

// relative amplitudes of each harmonic
const _Accum ampl_ratios[3][SINES_PER_NOTE] = {
    {1, 3.433, 1.836, 0.7996}, // piano
    {1, 0.6608, 0.7184, 1.103}, //organ
    {1, 0.4563, 0.1282, 0.08147} // guitar
};

void configureUART() {
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, sys_clock, 57600);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    PPSInput(2, U2RX, RPB5);
    PPSOutput(4, RPA3, U2TX);
}

void configureDAC() {
    // DAC chip select
    mPORTBSetPinsDigitalOut(BIT_1);
    mPORTBSetBits(BIT_1);

    // audio sample timer
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 8);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    INTEnableSystemMultiVectoredInt();
}

void configureLED() {
    mPORTASetPinsDigitalOut(BIT_0);
    mPORTAClearBits(BIT_0);
}

void configureSPI() {
    PPSInput(2, SDI1, RPB11);
    PPSOutput(2, RPA1, SDO1);
    SpiChnOpen(SPI_CHANNEL1, SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV, 4);
}

#endif

