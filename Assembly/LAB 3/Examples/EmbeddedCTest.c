#include "msp.h"

int main(void)
{
    volatile uint32_t i;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;   // Stop watchdog timer

    					// The following code toggles P1.0 port
    P1->DIR |= BIT0;                             // Configure P1.0 as output

    while(1)
    {
        P1->OUT ^= BIT0;                         // Toggle P1.0
        for(i=10000; i>0; i--);                  // Delay
    }
}
