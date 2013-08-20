// http://elm-chan.org/works/mxb/report.html

// C:\Program Files (x86)\Microchip\mplabc32\v2.01\pic32mx\\include\proc\p32mx220f032b.h
// C:\Program Files (x86)\Microchip\mplabc32\v2.01\pic32mx\lib\proc\32MX220F032B\configuration.data

// DEVCFG3:
#pragma config IOL1WAY  = OFF           // Peripheral Pin Select Configuration
// DEVCFG2:
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config UPLLEN   = OFF           // USB PLL Enabled
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
// DEVCFG1:
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = OFF           // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = FRCPLL        // Oscillator Selection
// DEVCFG0:
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = ON            // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // JTAG Enable
#pragma config DEBUG    = OFF           // Background Debugger Enable

#define SYSCLK 80000000L

#include <stdint.h>
#include <stdbool.h>

#include <p32xxxx.h>
#include <plib.h>

#include "wavetable.h"
#include "tune_still_alive.h"

void pwm_init()
{
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0 );
    OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 2047 ); // gives a sample rate of 80 MHz / 2048 = 39062 Hz
    OC1RS = 0;
    //mT2SetIntPriority( 7 );
    //mT2ClearIntFlag();
    //mT2IntEnable( 1 );
    // PPS:
    RPB15Rbits.RPB15R = 5; // OC1
}

//void __ISR( _TIMER_2_VECTOR, ipl7 ) T2Interrupt( void )
//{
//    // clear interrupt flag and exit
//    mT2ClearIntFlag();
//}

#define POT 8 // power of two; must match with scale_table values
#define ENVPOT 7
#define OSCILLATOR_COUNT 64 // 16 oscillators: 25% load with -O1 (64: 90%)
#define TICKS_LIMIT 203 // 39062 / ( 4 * 48 )
#define CLIP 1016 // 127 * 8
int main( void )
{
    SYSTEMConfigPerformance( SYSCLK );
    INTEnableSystemMultiVectoredInt();

    ANSELA = 0x0000; // all digital pins
    ANSELB = 0x0000;
    pwm_init();

    // load visualization pin
    LATBbits.LATB14 = 0;
    TRISBbits.TRISB14 = 0;

    uint16_t increments_pot[ OSCILLATOR_COUNT ];
    uint32_t phase_accu_pot[ OSCILLATOR_COUNT ];
    uint32_t envelope_positions_envpot[ OSCILLATOR_COUNT ];
    for ( uint8_t osc = 0; osc < OSCILLATOR_COUNT; ++osc ) {
        increments_pot[ osc ] = 0;
        phase_accu_pot[ osc ] = 0;
        envelope_positions_envpot[ osc ] = 0;
    }
    uint8_t next_osc = 0;
    uint16_t ticks = 0;
    uint16_t time = 0;
    uint16_t event_index = 0;
    const uint16_t event_count = sizeof( tune_still_alive ) / sizeof( tune_still_alive[ 0 ] );
    const uint32_t sizeof_wt_pot = ( (uint32_t)sizeof( wt ) << POT );
    const uint32_t sizeof_wt_sustain_pot = ( (uint32_t)sizeof( wt_sustain ) << POT );
    const uint32_t sizeof_wt_attack_pot = ( (uint32_t)sizeof( wt_attack ) << POT );
    const uint32_t sizeof_envelope_table_envpot = ( (uint32_t)sizeof( envelope_table ) << ENVPOT );
    while ( true ) {
        LATBbits.LATB14 = 1; // load visualization pin
        while ( time >= tune_still_alive[ event_index ].time ) {
            increments_pot[ next_osc ] = scale_table[ tune_still_alive[ event_index ].pitch ];
            phase_accu_pot[ next_osc ] = 0;
            envelope_positions_envpot[ next_osc ] = 0;
            ++next_osc;
            if ( next_osc >= OSCILLATOR_COUNT ) {
                next_osc = 0;
            }
            ++event_index;
            if ( event_index >= event_count ) {
                ticks = 0;
                time = 0;
                event_index = 0;
            }
        }
        ++ticks;
        if ( ticks >= TICKS_LIMIT ) {
            ticks = 0;
            time += 1;
        }

        int32_t value = 0;
        for ( uint8_t osc = 0; osc < OSCILLATOR_COUNT; ++osc ) {
            phase_accu_pot[ osc ] += increments_pot[ osc ];
            if ( phase_accu_pot[ osc ] >= sizeof_wt_pot ) {
                phase_accu_pot[ osc ] -= sizeof_wt_sustain_pot;
            }
            uint16_t phase_accu = ( phase_accu_pot[ osc ] >> POT );
            value += envelope_table[ envelope_positions_envpot[ osc ] >> ENVPOT ] * wt[ phase_accu ];
            if ( phase_accu_pot[ osc ] >= sizeof_wt_attack_pot &&
                 envelope_positions_envpot[ osc ] < sizeof_envelope_table_envpot - 1 )
            {
                ++envelope_positions_envpot[ osc ];
            }
        }
        value >>= 8; // envelope_table resolution
        if ( value > CLIP ) {
            value = CLIP;
        } else if ( value < -CLIP ) {
            value = -CLIP;
        }
        LATBbits.LATB14 = 0; // load visualization pin
        while ( TMR2 > 10 ) {
        }
        OC1RS = 1024 + value;
    }

    return 0;
}
