#include "common.h"

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) {
    mT2ClearIntFlag();
}

int main() {
    configureLED();
    configureSPI();
    configureUART();
    configureDAC();
    
    union { int i; char c[4]; } data;
    char recorded_notes[MAX_RECORDED_NOTES];
    int recorded_times[MAX_RECORDED_NOTES];
    
    for (;;) {
        while(!UARTReceivedDataIsAvailable(UART2));
        char recorded_count = UARTGetDataByte(UART2);
        
        int i, j;
        for (i = 0; i < recorded_count; i++) {
            while(!UARTReceivedDataIsAvailable(UART2));
            recorded_notes[i] = UARTGetDataByte(UART2);
            union {int i; char c[4]; } time;
            for (j = 0; j < 4; j++) {
                while(!UARTReceivedDataIsAvailable(UART2));
                time.c[j] = UARTGetDataByte(UART2);
            }
            recorded_times[i] = time.i;
        }
        
        for (i = 0; i < recorded_count; i++) {
            mPORTAToggleBits(BIT_0);
            for (j = 0; j < 1000000; j++);
        }
    }
}