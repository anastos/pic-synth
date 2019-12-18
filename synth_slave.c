#include "common.h"
#include "pt_cornell_1_3_2.h"

#define SLAVE_ID 1

#define RECV_BYTE(d) { \
    while (!UARTReceivedDataIsAvailable(UART2)); \
    d = UARTGetDataByte(UART2); \
}

#define RECV_INT(d) { \
    char_int ci; \
    int idx; \
    for (idx = 0; idx < 4; idx++) \
        RECV_BYTE(ci.c[idx]); \
    d = ci.i; \
}

struct Note notes[NUM_NOTES];
_Accum sine_tables[SINES_PER_NOTE][SINE_TABLE_SIZE];
_Accum env_table[3][ENV_TABLE_SIZE];
_Accum rel_table[REL_TABLE_SIZE];
volatile int playback_time;
unsigned char
    note_on[NUM_NOTES], // analogous to whether button is down or not
    recorded_notes[MAX_RECORDED_NOTES], // commands recieved
    playback, // whether the slave is playing a song
    loop, // whether the slave should repeat the song
    playback_idx, // next index of command to playback in recorded_notes
    recorded_count, // number of valid commands in recorded_notes
    octave = 3, // 0 <= octave <= 5, octave of notes begin played
    initial_octave, // octave to start playback in
    sound, // 0: piano, 1: organ, 2: plucked string
    initial_sound; // sound to start playback in

int recorded_times[MAX_RECORDED_NOTES], // times at which commands should run
    recorded_length; // time at which playback should end/start over

struct pt pt_uart, pt_btn; // threads for UART comm and checking buttons

// sets lookup tables, etc. to a specific sound
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

// start playback from the beginning
void initialize_playback() {
    playback = 1;
    playback_time = 0;
    playback_idx = 0;
    int i, j;
    for (i = 0; i < NUM_NOTES; i++) {
        notes[i].state = 0;
        note_on[i] = 0;
    }
    setSound(initial_sound);
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

// runs every 2048 cycles to compute DAC output
void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) {
    mT2ClearIntFlag();

    if (!playback)
        return;

    _Accum out = 0;
    int i, j;
    for (i = 0; i < NUM_NOTES; i++) {
        struct Note * note = &notes[i];
        if (note->state) { // pressed or recently released
            _Accum out_i = 0;
            for (j = 0; j < SINES_PER_NOTE; j++) { // add each freq component
                _Accum idx = note->idx[j];
                out_i += sine_tables[j][(unsigned int) idx % SINE_TABLE_SIZE];
                note->idx[j] = idx + note->inc[j];
            }
            if (note->state == 2) { // released
                out_i *= rel_table[min(note->rel_idx++ >> 8, REL_TABLE_SIZE - 1)];
                if (note->rel_idx >> 8 == REL_TABLE_SIZE)
                    note->state = 0;
            }
            //multiply by envelope function and add to total ouput
            out += out_i * env_table[sound][min(note->env_idx++ >> 8, ENV_TABLE_SIZE - 1)];
        }
    }

    playback_time++;
    // check for commands issued at this time
    while (playback_time == recorded_times[playback_idx]) {
        char n = recorded_notes[playback_idx++];
        if (n & BIT_5) { // change octave
            for (i = 0; i < NUM_NOTES; i++)
                notes[i].state = 0;
            if (n & BIT_0) { // up
                for (i = 0; i < NUM_NOTES; i++)
                    for (j = 0; j < SINES_PER_NOTE; j++)
                        notes[i].inc[j] <<= 1;
                octave++;
            } else { // down
                for (i = 0; i < NUM_NOTES; i++)
                    for (j = 0; j < SINES_PER_NOTE; j++)
                        notes[i].inc[j] >>= 1;
                octave--;
            }
        } else if (n & BIT_6) { // change sound
            setSound(n & (BIT_1 | BIT_0));
        } else { // toggle a note
            note_on[n] = !note_on[n];
        }
    }
    if (playback_time == recorded_length) { // end of recording
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
    // DC bias output to prevent clipping
    WriteSPI1(DAC_CONFIG_CHAN_A | (((int) out + 0x800) & 0x0fff));
    while (SPI1STATbits.SPIBUSY);
    ReadSPI1();
    mPORTBSetBits(BIT_1);
}

// responds to button presses
static PT_THREAD (btn_thread(struct pt *pt)) {
    PT_BEGIN(pt);
    static char_int btn_st, btn_samp;
    for (;;) {
        PT_YIELD_TIME_msec(30);
        int i;
        char_int btn_samp_prev = btn_samp;
        btn_samp.i = mPORTBReadBits(BIT_0 | BIT_2);
        char_int btn_st_prev = btn_st;
        btn_st.i = (btn_samp.i ^ btn_samp_prev.i) & btn_st.i | btn_samp.i & btn_samp_prev.i;
        if (PRESSED(BIT_0)) { // start/stop playback
            if (playback) {
                playback = 0;
                loop = 0;
                mPORTBClearBits(BIT_15);
                DISABLE_ISR;
            } else {
                initialize_playback();
            }
        }
        if (PRESSED(BIT_2)) { // start/stop looping playback
            loop = !loop;
            if (loop && !playback) {
                initialize_playback();
            }
        }
        for (i = 0; i < NUM_NOTES; i++) { // set note states
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

// recieve UART transmissions
static PT_THREAD (uart_thread(struct pt *pt)) {
    PT_BEGIN(pt);
    for (;;) {
        int i, j;
        char_int time;
        char slave, count;
        while (!UARTReceivedDataIsAvailable(UART2))
            PT_YIELD_TIME_msec(5);
        RECV_BYTE(slave);
        RECV_BYTE(count);
        if (slave == SLAVE_ID) {
            mPORTASetBits(BIT_0);
            recorded_count = count;
            RECV_BYTE(initial_octave);
            RECV_BYTE(initial_sound);
            RECV_INT(recorded_length);
            for (i = 0; i < recorded_count; i++) {
                RECV_BYTE(recorded_notes[i]);
                RECV_INT(recorded_times[i])
            }
            recorded_times[recorded_count] = 0;
            mPORTAClearBits(BIT_0);
        } else { // not for me, ignore
            for (i = 0; i < 5 * count + 6; i++)
                RECV_BYTE(j);
        }
    }
    PT_END(pt);
}

int main() {
    int i;
    for (i = 0; i < ENV_TABLE_SIZE; i++) {
        _Accum a = i;
        env_table[0][i] = 0.05 + exp(-a >> 7); // piano
        env_table[1][i] = (1 - exp(-a >> 5)) * exp(-a >> 10); // organ
        env_table[2][i] = 4 * exp(-a >> 4); // guitar
    }

    for (i = 0; i < REL_TABLE_SIZE; i++)
        rel_table[i] = exp(-((_Accum) i) / 8);

    PT_setup();
    configureLED();
    configureSPI();
    configureUART();
    configureDAC();

    // playback and loop buttons
    mPORTBSetPinsDigitalIn(BIT_0 | BIT_2);
    EnablePullDownB(BIT_0 | BIT_2);

    // is playing LED indicator
    mPORTBSetPinsDigitalOut(BIT_15);
    mPORTBClearBits(BIT_15);

    PT_INIT(&pt_uart);
    PT_INIT(&pt_btn);

    for (;;) {
        PT_SCHEDULE(uart_thread(&pt_uart));
        PT_SCHEDULE(btn_thread(&pt_btn));
    }
}
