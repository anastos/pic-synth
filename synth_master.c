#include "common.h"
#include "port_expander_brl4.h"

#define SEND_BYTE(b) { \
    while(!UARTTransmitterIsReady(UART2)); \
    UARTSendDataByte(UART2, b); \
}

#define SEND_INT(d) { \
    char_int ci; \
    ci.i = d; \
    int idx; \
    for (idx = 0; idx < 4; idx++) \
        SEND_BYTE(ci.c[idx]) \
}

struct Note notes[NUM_NOTES];
_Accum sine_tables[SINES_PER_NOTE][SINE_TABLE_SIZE];
_Accum env_table[3][ENV_TABLE_SIZE];
_Accum rel_table[REL_TABLE_SIZE];
volatile int recording_time;
char sound;

unsigned char recording, octave = 3, initial_octave, initial_sound, recorded_count;
char recorded_notes[MAX_RECORDED_NOTES];
int recorded_times[MAX_RECORDED_NOTES];
int recorded_length;

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
            out += out_i * env_table[sound][min(note->env_idx++ >> 8, ENV_TABLE_SIZE - 1)];
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

void setSound(char new_sound) {
    mPORTASetBits(BIT_0);
    int i, j;
    for (i = 0; i < NUM_NOTES; i++)
        notes[i].state = 0;
    sound = new_sound;
    for (i = 0; i < SINES_PER_NOTE; i++)
        for (j = 0; j < SINE_TABLE_SIZE; j++)
            sine_tables[i][j] = 50 * ampl_ratios[sound][i] * sin(j * 6.28319 / SINE_TABLE_SIZE);
    for (i = 0; i < NUM_NOTES; i++)
        for (j = 0; j < SINES_PER_NOTE; j++)
            notes[i].inc[j] = (_Accum) (0.96186 * base_freqs[i] * freq_ratios[sound][j]) << octave;
    mPORTAClearBits(BIT_0);
}

void sendRecording(char slave) {
    mPORTASetBits(BIT_0);
    SEND_BYTE(slave);
    int i;
    for (i = 100000; i--;);
    SEND_BYTE(recorded_count);
    SEND_BYTE(initial_octave);
    SEND_BYTE(initial_sound);
    SEND_INT(recorded_length);
    for (i = 0; i < recorded_count; i++) {
        SEND_BYTE(recorded_notes[i]);
        SEND_INT(recorded_times[i]);
    }
    mPORTAClearBits(BIT_0);
    mPORTBClearBits(BIT_13);
}

int main() {
    char_int btn_st, btn_samp;

    int i;
    for (i = 0; i < ENV_TABLE_SIZE; i++) {
        _Accum a = i;
        env_table[0][i] = 0.05 + exp(-a >> 7); // piano
        env_table[1][i] = (1 - exp(-a >> 5)) * exp(-a >> 10); // organ
        env_table[2][i] = 4 * exp(-a >> 4); // guitar
    }

    for (i = 0; i < REL_TABLE_SIZE; i++)
        rel_table[i] = exp(-((_Accum) i) / 8);

    setSound(0);

    configureLED();
    configureSPI();
    configureDAC();
    configurePE();
    configureUART();

    mPORTBSetPinsDigitalOut(BIT_13 | BIT_15);
    mPORTBClearBits(BIT_13 | BIT_15);

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
        if (PRESSED(BIT_18)) {
            setSound(0);
            if (recording) {
                recorded_notes[recorded_count] = BIT_6 | sound;
                recorded_times[recorded_count++] = recording_time;
            }
        }
        if (PRESSED(BIT_19)) {
            setSound(1);
            if (recording) {
                recorded_notes[recorded_count] = BIT_6 | sound;
                recorded_times[recorded_count++] = recording_time;
            }
        }
        if (PRESSED(BIT_20)) {
            setSound(2);
            if (recording) {
                recorded_notes[recorded_count] = BIT_6 | sound;
                recorded_times[recorded_count++] = recording_time;
            }
        }
        if (PRESSED(BIT_21) && octave < 5) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            for (i = 0; i < NUM_NOTES; i++)
                for (j = 0; j < SINES_PER_NOTE; j++)
                    notes[i].inc[j] <<= 1;
            octave++;
            if (recording) {
                recorded_notes[recorded_count] = BIT_5 | BIT_0;
                recorded_times[recorded_count++] = recording_time;
            }
        }
        if (PRESSED(BIT_22) && 0 < octave) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            for (i = 0; i < NUM_NOTES; i++)
                for (j = 0; j < SINES_PER_NOTE; j++)
                    notes[i].inc[j] >>= 1;
            octave--;
            if (recording) {
                recorded_notes[recorded_count] = BIT_5;
                recorded_times[recorded_count++] = recording_time;
            }
        }
        if (PRESSED(BIT_23)) {
            DISABLE_ISR;
            recording = !recording;
            if (recording) {
                recording_time = 0;
                recorded_count = 0;
                initial_octave = octave;
                initial_sound = sound;
                mPORTBClearBits(BIT_13);
                mPORTBSetBits(BIT_15);
            } else {
                recorded_length = recording_time;
                mPORTBClearBits(BIT_15);
                mPORTBSetBits(BIT_13);
            }
            ENABLE_ISR;
        }
        if (PRESSED(BIT_24))
            sendRecording(0);
        if (PRESSED(BIT_25))
            sendRecording(1);
    }
}
