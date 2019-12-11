#include "common.h"
#include "port_expander_brl4.h"

#define PRESSED(b) ((btn_st.i & b) && !(btn_st_prev.i & b))

struct Note notes[NUM_NOTES];
_Accum sine_tables[SINES_PER_NOTE][SINE_TABLE_SIZE];
_Accum env_table[ENV_TABLE_SIZE];
_Accum rel_table[REL_TABLE_SIZE];
volatile int recording_time;
char recorded_notes[MAX_RECORDED_NOTES];
int recorded_times[MAX_RECORDED_NOTES];
unsigned char playback, playback_idx, recorded_count;

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) {
    mT2ClearIntFlag();

    _Accum out = 0;
    int i, j;
    for (i = 0; i < NUM_NOTES; i++) {
        struct Note * note = &notes[i];
        if (note->state) {
            _Accum out_i = 0;
            for (j = 0; j < SINES_PER_NOTE; j++) {
                _Accum idx = note->idx[j];
                out_i += sine_tables[j][(unsigned int) idx % SINE_TABLE_SIZE];
                note->idx[j] = idx + note->inc[j];
            }
            if (note->state == 2) {
                out_i *= rel_table[min(note->rel_idx++ >> 8, REL_TABLE_SIZE - 1)];
                if (note->rel_idx >> 8 == REL_TABLE_SIZE)
                    note->state = 0;
            }
            out += out_i * env_table[min(note->env_idx++ >> 8, ENV_TABLE_SIZE - 1)];
        }
    }

    recording_time++;

    while (TxBufFullSPI1());
    mPORTBClearBits(BIT_1);
    SPI_Mode16;
    WriteSPI1(DAC_CONFIG_CHAN_A | (((int) out + 0x800) & 0x0fff));
    while (SPI1STATbits.SPIBUSY);
    ReadSPI1();
    mPORTBSetBits(BIT_1);
}

int main() {    
    _Accum base_freqs[] = {1, 1.0595, 1.1225, 1.1892, 1.2599, 1.3348,
            1.4142, 1.4983, 1.5874, 1.6818, 1.7818, 1.8877, 2,
            2.1189, 2.2449, 2.3784, 2.5198, 2.6697};
    _Accum freq_ratios[] = {1, 2, 3, 4, 5};
    _Accum ampl_ratios[] = {1, 3.433, 1.836, 0.7996, 0.9882};
                        // {1, 0.5839, 0.2303, 0.1026, 0.04308};

    int i, j;
    for (i = 0; i < SINES_PER_NOTE; i++)
        for (j = 0; j < SINE_TABLE_SIZE; j++)
            sine_tables[i][j] = 25 * ampl_ratios[i] * sin(j * 6.28319 / SINE_TABLE_SIZE);

    for (i = 0; i < NUM_NOTES; i++)
        for (j = 0; j < SINES_PER_NOTE; j++) {
            notes[i].inc[j] = 15.3897 * base_freqs[i] * freq_ratios[j];
            notes[i + NUM_NOTES].inc[j] = notes[i].inc[j];
        }

    for (i = 0; i < ENV_TABLE_SIZE; i++)
        env_table[i] = 0.1 + 2 * exp(-((_Accum) i) / 128);

    for (i = 0; i < REL_TABLE_SIZE; i++)
        rel_table[i] = exp(-((_Accum) i) / 8);

    configureLED();
    configureSPI();
    configureDAC();
    configurePE();
    configureUART();

    mPORTBSetPinsDigitalOut(BIT_13 | BIT_15);
    mPORTBClearBits(BIT_13 | BIT_15);

    char_int btn_st, btn_samp;
    char recording, pending_recording;

    while (1) {
        int i, j;
        for (i = 100000; i--;);

        char_int btn_samp_prev = btn_samp;
        for (i = 0; i < 4; i++) {
            DISABLE_ISR;
            writePE(GPIOZ, 1 << i);
            btn_samp.c[i] = readPE(GPIOY);
            ENABLE_ISR;
        }

        char_int btn_st_prev = btn_st;
        btn_st.i = (btn_samp.i ^ btn_samp_prev.i) & btn_st.i | btn_samp.i & btn_samp_prev.i;

        for (i = 0; i < NUM_NOTES; i++) {
            int btn = btn_st.i & (1 << i);
            int btn_prev = btn_st_prev.i & (1 << i);
            if (btn != btn_prev) {
                if (btn) {
                    notes[i].state = 1;
                    notes[i].env_idx = 0;
                    notes[i].rel_idx = 0;
                } else {
                    notes[i].state = 2;
                }
                if (recording) {
                    recorded_notes[recorded_count] = i;
                    recorded_times[recorded_count++] = recording_time;
                }
            }
        }
        if (PRESSED(BIT_21)) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            for (i = 0; i < NUM_NOTES; i++)
                for (j = 0; j < SINES_PER_NOTE; j++)
                    notes[i].inc[j] <<= 1;
        }
        if (PRESSED(BIT_22)) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            for (i = 0; i < NUM_NOTES; i++)
                for (j = 0; j < SINES_PER_NOTE; j++)
                    notes[i].inc[j] >>= 1;
        }
        if (PRESSED(BIT_23)) {
            DISABLE_ISR;
            recording = !recording;
            if (recording) {
                recording_time = 0;
                recorded_count = 0;
                mPORTBSetBits(BIT_15);
            } else {
                mPORTBClearBits(BIT_15);
                pending_recording = 1;
                mPORTBSetBits(BIT_13);
            }
            ENABLE_ISR;
        }
        if (PRESSED(BIT_24)) {
            while(!UARTTransmitterIsReady(UART2));
            UARTSendDataByte(UART2, recorded_count);
            for (i = 0; i < recorded_count; i++) {
                while(!UARTTransmitterIsReady(UART2));
                UARTSendDataByte(UART2, recorded_notes[i]);
                char_int time;
                time.i = recorded_times[i];
                for (j = 0; j < 4; j++) {
                    while(!UARTTransmitterIsReady(UART2));
                    UARTSendDataByte(UART2, time.c[j]);
                }
            }
            mPORTBClearBits(BIT_13);
        }
    }
}
