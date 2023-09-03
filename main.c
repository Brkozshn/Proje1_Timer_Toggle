
#include <msp430.h>

void Init_GPIO();


#define MCLK_FREQ_MHZ 8                     // MCLK = 8MHz

#define TOGGLE_1SEC_VALUE 200
#define TOGGLE_10SEC_VALUE TOGGLE_1SEC_VALUE * 10

unsigned int ADC_Result;

volatile unsigned int toggle_1s = 0;
volatile unsigned int toggle_10s = 0;
volatile unsigned int toggle_off = 0;


enum{
    m_PHASE0,
    m_PHASE1,
    m_PHASE2,
    m_PHASE3,


    m_NUM_OF_PHASES
};

int g_phase = m_PHASE1;

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                // Stop watchdog timer


    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    P1IFG &= ~BIT3;                         // P1.3 IFG cleared


  // Configure GPIO
  Init_GPIO();


   TA0CCTL0 |= CCIE;                             // TACCR0 interrupt enabled
   TA0CCR0 = 50000;
   TA0CTL |= TASSEL__SMCLK | MC__UP | ID_3;     // SMCLK, continuous mode  divided by 2^3      8Mhz / 8 = 1Mhz


  __bis_SR_register(SCG0);                 // disable FLL
  CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
  CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
  CSCTL2 = FLLD_0 + 243;                  // DCODIV = 8MHz
  __delay_cycles(3);
  __bic_SR_register(SCG0);                // enable FLL

  CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                           // default DCODIV as MCLK and SMCLK source

  // Configure UART pins
  P5SEL0 |= BIT1 | BIT2;                    // set 2-UART pin as second function
  SYSCFG3|=USCIA0RMP;                       //Set the remapping source
  // Configure UART
  UCA0CTLW0 |= UCSWRST;
  UCA0CTLW0 |= UCSSEL__SMCLK;

  // Baud Rate calculation
  // 8000000/(16*9600) = 52.083
  // Fractional portion = 0.083
  // User's Guide Table 17-4: UCBRSx = 0x49
  // UCBRFx = int ( (52.083-52)*16) = 1
  UCA0BR0 = 52;                             // 8000000/16/9600
  UCA0BR1 = 0x00;
  UCA0MCTLW = 0x4900 | UCOS16 | UCBRF_1;

  UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt


  __bis_SR_register(GIE);          //Global interrupt enable
  while(1)
  {

__no_operation();

/*
    if ((g_phase == m_PHASE1)) {

            P1OUT &= ~BIT0;     // Set P1.0 LED off
            UCA0TXBUF = 'x';
            TA0CCR0 = 1000000;
            //__bis_SR_register(LPM3_bits|GIE);

           // g_phase = m_PHASE0;
       }
*/
 }


 // __bis_SR_register(LPM3_bits|GIE);         // Enter LPM3, interrupts enabled
  //__no_operation();                         // For debugger

}


void Init_GPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00;

    // Configure GPIO
      P1DIR |= BIT0;                                           // Set P1.0/LED to output direction
      P1OUT &= ~BIT0;                                          // P1.0 LED off

      // Configure ADC A1 pin
      P1SEL0 |= BIT1;
      P1SEL1 |= BIT1;


      // Configure GPIO  for button
         P1OUT &= ~BIT0;                         // Clear P1.0 output latch for a defined power-on state
         P1DIR |= BIT0;                          // Set P1.0 to output direction

         P1OUT |= BIT3;                          // Configure P1.3 as pulled-up
         P1REN |= BIT3;                          // P1.3 pull-up register enable
         P1IES |= BIT3;                          // P1.3 Hi/Low edge
         P1IE |= BIT3;                           // P1.3 interrupt enabled

         // Configure GPIO for Timer
           P5DIR |= BIT0;                                // P5.0 output
           P5OUT |= BIT0 ;                               // P5.0  high

           P4DIR |= BIT7;                                // P4.7 output
           P4OUT |= BIT7 ;                               // P4.7  high


}



// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{

/*
    switch(g_phase)
               {
                   case m_PHASE0:
                       g_phase = m_PHASE2;
                       //while(!(UCA0IFG&UCTXIFG));
                       //UCA0TXBUF = 'C';
                       TA0CCR0 = 50000;
                       break;

                   case m_PHASE1:
                       g_phase = m_PHASE0;
                                                // TACCR0 interrupt enabled
                       TA0CCR0 = 10000;
                       break;

                   case m_PHASE2:
                       g_phase = m_PHASE1;
                       TA0CCR0 = 5000;
                       break;

               }

    __no_operation();                                    // For debug only
*/

    P1IFG &= ~BIT3;                         // Clear P1.3 IFG

}


// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{

/*
    switch(g_phase)
                  {
                      case m_PHASE0:
                          //g_phase = m_PHASE2;
                          while(!(UCA0IFG&UCTXIFG));
                          UCA0TXBUF = 'C';
                          break;

                      case m_PHASE1:
                          //g_phase = m_PHASE0;
                          while(!(UCA0IFG&UCTXIFG));
                          UCA0TXBUF = 'A';
                          break;

                      case m_PHASE2:
                          //g_phase = m_PHASE1;
                          while(!(UCA0IFG&UCTXIFG));
                          UCA0TXBUF = 'x';
                          break;

                  }

*/

            toggle_1s++;
            toggle_10s++;
            toggle_off++;

            // Toggle LED for 1 seconds
            if (toggle_1s == TOGGLE_1SEC_VALUE) {
                P5OUT ^= BIT0;    // Toggling led
                toggle_1s = 0;
            }

            // Toggle LED for 10 seconds
             if (toggle_10s == TOGGLE_10SEC_VALUE) {
                P4OUT ^= BIT7;      // Toggling led
                toggle_10s = 0;
            }

             if (toggle_off == 14) {
                //P5OUT ^= BIT0;      // Toggling led
                P5OUT &= ~BIT0;      // Led off
                __delay_cycles(10);
                toggle_off = 0;
            }

}


// UART interrupt service routine

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
        while(!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = UCA0RXBUF;
      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}
