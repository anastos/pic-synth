#include "common.h"

struct Note notes[NUM_NOTES];
_Accum sine_tables[SINES_PER_NOTE][SINE_TABLE_SIZE];
_Accum env_table[ENV_TABLE_SIZE];
_Accum rel_table[REL_TABLE_SIZE];
volatile int playback_time;
char note_on[NUM_NOTES];
char recorded_notes[MAX_RECORDED_NOTES];
int recorded_times[MAX_RECORDED_NOTES];
unsigned char playback, playback_idx, recorded_count;

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) {
    mT2ClearIntFlag();

    if (!playback)
        return;

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

    playback_time++;
    while (playback_time == recorded_times[playback_idx]) {
        note_on[recorded_notes[playback_idx]] = !note_on[recorded_notes[playback_idx]];
        if (++playback_idx == recorded_count) {
            playback = 0;
            mPORTBClearBits(BIT_15);
            DISABLE_ISR;
            break;
        }
    }

    while (TxBufFullSPI1());
    mPORTBClearBits(BIT_1);
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
    configureUART();
    configureDAC();

    mPORTBSetPinsDigitalIn(BIT_0);
    EnablePullDownB(BIT_0);

    mPORTBSetPinsDigitalOut(BIT_15);
    mPORTBClearBits(BIT_15);

    for (;;) {
        char prev_button;
        while (!UARTReceivedDataIsAvailable(UART2)) {
            char button = mPORTBReadBits(BIT_0);
            if (button && !prev_button) {
                playback = !playback;
                if (playback) {
                    playback_time = 0;
                    playback_idx = 0;
                    ENABLE_ISR;
                    mPORTBSetBits(BIT_15);
                } else {
                    for (i = 0; i < NUM_NOTES; i++) {
                        notes[i + NUM_NOTES].state = 0;
                        note_on[i] = 0;
                    }
                    mPORTBClearBits(BIT_15);
                    DISABLE_ISR;
                }
            }
            prev_button = button;
            for (i = 0; i < NUM_NOTES; i++) {
                if (notes[i].state == 1) {
                    if (note_on[i]) {
                        notes[i].state = 1;
                        notes[i].env_idx = 0;
                        notes[i].rel_idx = 0;
                    }
                } else if (!note_in[i]) {
                    notes[i].state = 2;
                }
            }
        }

        recorded_count = UARTGetDataByte(UART2);

        for (i = 0; i < recorded_count; i++) {
            while (!UARTReceivedDataIsAvailable(UART2));
            recorded_notes[i] = UARTGetDataByte(UART2);
            char_int time;
            for (j = 0; j < 4; j++) {
                while (!UARTReceivedDataIsAvailable(UART2));
                time.c[j] = UARTGetDataByte(UART2);
            }
            recorded_times[i] = time.i;
        }
    }
}
