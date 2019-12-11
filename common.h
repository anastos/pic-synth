#ifndef COMMON_H
#define	COMMON_H

#include "config_1_3_2.h"
#include "math.h"

#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits

#define DAC_CONFIG_CHAN_A 0b0011000000000000
#define DISABLE_ISR INTEnable(INT_T2, 0)
#define ENABLE_ISR INTEnable(INT_T2, 1)

#define MAX_RECORDED_NOTES 256
#define NUM_NOTES 18
#define SINES_PER_NOTE 4
#define SINE_TABLE_SIZE 1024
#define ENV_TABLE_SIZE 512
#define REL_TABLE_SIZE 32

typedef union { int i; char c[4]; } char_int;

struct Note {
    volatile char state;
    _Accum inc[SINES_PER_NOTE], idx[SINES_PER_NOTE];
    int env_idx, rel_idx;
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

