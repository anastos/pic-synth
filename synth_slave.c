#include "common.h"
#include "pt_cornell_1_3_2.h"

struct Note notes[NUM_NOTES];
_Accum sine_tables[SINES_PER_NOTE][SINE_TABLE_SIZE];
_Accum env_table[ENV_TABLE_SIZE];
_Accum rel_table[REL_TABLE_SIZE];
volatile int playback_time;
char note_on[NUM_NOTES];
char recorded_notes[MAX_RECORDED_NOTES];
int recorded_times[MAX_RECORDED_NOTES];
int recorded_length;
unsigned char playback, loop, playback_idx, recorded_count, octave = 3, initial_octave;

struct pt pt_uart, pt_btn;

void initialize_playback() {
    playback = 1;
    playback_time = 0;
    playback_idx = 0;
    int i, j;
    for (i = 0; i < NUM_NOTES; i++) {
        notes[i].state = 0;
        note_on[i] = 0;
    }
    if (initial_octave < octave) {
        for (i = 0; i < NUM_NOTES; i++)
            for (j = 0; j < SINES_PER_NOTE; j++)
                notes[i].inc[j] >>= (octave - initial_octave);
    } else if (initial_octave > octave) {
        for (i = 0; i < NUM_NOTES; i++)
            for (j = 0; j < SINES_PER_NOTE; j++)
                notes[i].inc[j] <<= (initial_octave - octave);
    }
    octave = initial_octave;
    ENABLE_ISR;
    mPORTBSetBits(BIT_15);
}

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
        char n = recorded_notes[playback_idx++];
        if (n & BIT_5) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            if (n & BIT_0) {
                for (i = 0; i < NUM_NOTES; i++)
                    for (j = 0; j < SINES_PER_NOTE; j++)
                        notes[i].inc[j] <<= 1;
                octave++;
            } else {
                for (i = 0; i < NUM_NOTES; i++)
                    for (j = 0; j < SINES_PER_NOTE; j++)
                        notes[i].inc[j] >>= 1;
                octave--;
            }
        } else {
            note_on[n] = !note_on[n];
        }
    }
    if (playback_time == recorded_length) {
        if (loop) {
            initialize_playback();
        } else {
            playback = 0;
            mPORTBClearBits(BIT_15);
            DISABLE_ISR;
        }
    }

    while (TxBufFullSPI1());
    mPORTBClearBits(BIT_1);
    WriteSPI1(DAC_CONFIG_CHAN_A | (((int) out + 0x800) & 0x0fff));
    while (SPI1STATbits.SPIBUSY);
    ReadSPI1();
    mPORTBSetBits(BIT_1);
}

static PT_THREAD (btn_thread(struct pt *pt)) {
    PT_BEGIN(pt);
    static char_int btn_st, btn_samp;
    for (;;) {
        PT_YIELD_TIME_msec(30);
        int i, j;
        char_int btn_samp_prev = btn_samp;
        btn_samp.i = mPORTBReadBits(BIT_0 | BIT_2);
        char_int btn_st_prev = btn_st;
        btn_st.i = (btn_samp.i ^ btn_samp_prev.i) & btn_st.i | btn_samp.i & btn_samp_prev.i;
        if (PRESSED(BIT_0)) {
            if (playback) {
                playback = 0;
                loop = 0;
                mPORTBClearBits(BIT_15);
                DISABLE_ISR;
            } else {
                initialize_playback();
            }
        }
        if (PRESSED(BIT_2)) {
            loop = !loop;
            if (loop && !playback) {
                initialize_playback();
            }
        }
        for (i = 0; i < NUM_NOTES; i++) {
            if (notes[i].state == 1) {
                if (!note_on[i])
                    notes[i].state = 2;
            } else if (note_on[i]) {
                notes[i].state = 1;
                notes[i].env_idx = 0;
                notes[i].rel_idx = 0;
            }
        }
    }
    PT_END(pt);
}

static PT_THREAD (uart_thread(struct pt *pt)) {
    PT_BEGIN(pt);
    for (;;) {
        int i, j;
        char_int time;
        while (!UARTReceivedDataIsAvailable(UART2)) {
            PT_YIELD_TIME_msec(5);
        }
        mPORTASetBits(BIT_0);
        initial_octave = UARTGetDataByte(UART2);
        for (j = 0; j < 4; j++) {
            while (!UARTReceivedDataIsAvailable(UART2));
            time.c[j] = UARTGetDataByte(UART2);
        }
        recorded_length = time.i;
        while (!UARTReceivedDataIsAvailable(UART2));
        recorded_count = UARTGetDataByte(UART2);

        for (i = 0; i < recorded_count; i++) {
            while (!UARTReceivedDataIsAvailable(UART2));
            recorded_notes[i] = UARTGetDataByte(UART2);
            for (j = 0; j < 4; j++) {
                while (!UARTReceivedDataIsAvailable(UART2));
                time.c[j] = UARTGetDataByte(UART2);
            }
            recorded_times[i] = time.i;
        }
        recorded_times[recorded_count] = 0;
        mPORTAClearBits(BIT_0);
    }
    PT_END(pt);
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


    PT_setup();
    configureLED();
    configureSPI();
    configureUART();
    configureDAC();

    mPORTBSetPinsDigitalIn(BIT_0 | BIT_2);
    EnablePullDownB(BIT_0 | BIT_2);

    mPORTBSetPinsDigitalOut(BIT_15);
    mPORTBClearBits(BIT_15);

    PT_INIT(&pt_uart);
    PT_INIT(&pt_btn);

    for (;;) {
        PT_SCHEDULE(uart_thread(&pt_uart));
        PT_SCHEDULE(btn_thread(&pt_btn));
    }
}
