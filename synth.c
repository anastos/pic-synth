#include "config_1_3_2.h"
#include "math.h"
#include "port_expander_brl4.h"

#define DAC_CONFIG_CHAN_A 0b0011000000000000
#define DISABLE_ISR INTEnable(INT_T2, 0)
#define ENABLE_ISR INTEnable(INT_T2, 1)

#define NUM_NOTES 17
#define SINES_PER_NOTE 3
#define SINE_TABLE_SIZE 1024
#define ENV_TABLE_SIZE 512
#define REL_TABLE_SIZE 32
#define NUM_NOTES_RECORDED 200

struct Note {
    volatile char state;
    _Accum inc[SINES_PER_NOTE], idx[SINES_PER_NOTE];
    int env_idx, rel_idx;
};

struct Note notes[2 * NUM_NOTES];
_Accum sine_tables[SINES_PER_NOTE][SINE_TABLE_SIZE];
_Accum env_table[ENV_TABLE_SIZE];
_Accum rel_table[REL_TABLE_SIZE];
volatile int recording_time, playback_time;
volatile char note_on[NUM_NOTES];
int recorded_times[NUM_NOTES_RECORDED], recorded_notes[NUM_NOTES_RECORDED];
char playback, playback_idx, recorded_count;

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) {
    mT2ClearIntFlag();
    
    _Accum out = 0;
    int i;
    for (i = 0; i < 2 * NUM_NOTES; i++) {
        struct Note * note = &notes[i];
        if (note->state) {
            _Accum out_i = 0;
            int j;
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
    playback_time++;
    if (playback && playback_time == recorded_times[playback_idx]) {
        note_on[recorded_notes[playback_idx]] = !note_on[recorded_notes[playback_idx]];
        if (++playback_idx == recorded_count)
            playback = 0;
    }

    while (TxBufFullSPI1());
    mPORTBClearBits(BIT_1);
    SPI_Mode16;
    WriteSPI1(DAC_CONFIG_CHAN_A | (((int) out + 0x800) & 0x0fff));
    while (SPI1STATbits.SPIBUSY);
    ReadSPI1();
    mPORTBSetBits(BIT_1);
}

int main() {    
    _Accum base_freqs[] = {10, 10.595, 11.225, 11.892, 12.599, 13.348,
            14.142, 14.983, 15.874, 16.818, 17.818, 18.877, 20,
            21.189, 22.449, 23.784, 25.198};
    _Accum freq_ratios[] = {1, 2, 3, 4, 5};
    _Accum ampl_ratios[] = {1, 3.433, 1.836, 0.7996, 0.9882};
                         //{1, 0.5839, 0.2303, 0.1026, 0.04308};

    int i, j;
    for (i = 0; i < SINES_PER_NOTE; i++)
        for (j = 0; j < SINE_TABLE_SIZE; j++)
            sine_tables[i][j] = 25 * ampl_ratios[i] * sin(j * 6.28319 / SINE_TABLE_SIZE);
    
    for (i = 0; i < NUM_NOTES; i++)
        for (j = 0; j < SINES_PER_NOTE; j++) {
            notes[i].inc[j] = base_freqs[i] * freq_ratios[j];
            notes[i + NUM_NOTES].inc[j] = notes[i].inc[j];
        }
    
    for (i = 0; i < ENV_TABLE_SIZE; i++)
        env_table[i] = 0.1 + 2 * exp(-((_Accum) i) / 128);

    for (i = 0; i < REL_TABLE_SIZE; i++)
        rel_table[i] = exp(-((_Accum) i) / 8);

    mPORTASetPinsDigitalOut(BIT_0); // LED
    mPORTAClearBits(BIT_0);
    
    mPORTBSetPinsDigitalOut(BIT_0 | BIT_1); // Chip selects
    mPORTBSetBits(BIT_0 | BIT_1);
    
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 8);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    PPSOutput(2, RPA1, SDO1);
    PPSInput(2, SDI1, RPB11);
    SpiChnOpen(SPI_CHANNEL1, SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV, 4);
    initPE();
    
    mPortYSetPinsIn(0xff);
    mPortZSetPinsOut(0xff);

    clearBits(GPIOZ, BIT_7);
    
    INTEnableSystemMultiVectoredInt();

    char buttons[3], buttons_prev[3], recording;

    while (1) {
        int i, j;
        for (i = 0; i < 3; i++)
            buttons_prev[i] = buttons[i];
        for (i = 0; i < 3; i++) {
            DISABLE_ISR;
            setBits(GPIOZ, (1 << i));
            buttons[i] = readPE(GPIOY);
            clearBits(GPIOZ, (1 << i));
            ENABLE_ISR;
        }
        for (i = 0; i < 2 * NUM_NOTES; i++) {
            char button_pressed = i < NUM_NOTES ? buttons[i >> 3] & (1 << (i % 8)) : note_on[i - NUM_NOTES];
            switch (notes[i].state) {
                case 0:
                    if (button_pressed) {
                        notes[i].state = 1;
                        notes[i].env_idx = 0;
                        notes[i].rel_idx = 0;
                        if (recording) {
                            recorded_notes[recorded_count] = i;
                            recorded_times[recorded_count++] = recording_time;
                        }
                    }
                    break;
                case 1:
                    if (!button_pressed) {
                        notes[i].state = 2;
                        if (recording) {
                            recorded_notes[recorded_count] = i;
                            recorded_times[recorded_count++] = recording_time;
                        }
                    }
                    break;
                case 2:
                    if (button_pressed) {
                        notes[i].state = 1;
                        notes[i].env_idx = 0;
                        notes[i].rel_idx = 0;
                        if (recording) {
                            recorded_notes[recorded_count] = i;
                            recorded_times[recorded_count++] = recording_time;
                        }
                    }
                    break;
            }
        }
        if ((buttons[2] & 64) && !(buttons_prev[2] & 64)) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            for (i = 0; i < NUM_NOTES; i++)
                for (j = 0; j < SINES_PER_NOTE; j++)
                    notes[i].inc[j] <<= 1;
        }
        if ((buttons[2] & 128) && !(buttons_prev[2] & 128)) {
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            for (i = 0; i < NUM_NOTES; i++)
                for (j = 0; j < SINES_PER_NOTE; j++)
                    notes[i].inc[j] >>= 1;
        }
        if ((buttons[2] & 32) && !(buttons_prev[2] & 32)) {
            DISABLE_ISR;
            recording = !recording;
            if (recording) {
                recording_time = 0;
                recorded_count = 0;
                setBits(GPIOZ, BIT_7);
            } else {
                clearBits(GPIOZ, BIT_7);
            }
            ENABLE_ISR;
        }
        if ((buttons[2] & 16) && !(buttons_prev[2] & 16)) {
            playback = !playback;
            if (playback) {
                playback_time = 0;
                playback_idx = 0;
            } else {
                for (i = 0; i < NUM_NOTES; i++) {
                    notes[i + NUM_NOTES].state = 0;
                    note_on[i] = 0;
                }
            }
        }

        playback ? mPORTASetBits(BIT_0) : mPORTAClearBits(BIT_0);
    }
}
