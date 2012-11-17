//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
//*****************************************************************************
#include "hw_nvic.h"
#include "hw_types.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void Reset_Handler(void);
void NmiSR(void);
void FaultISR(void);
void IntDefaultHandler(void);

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

/* exception and interrupt vector table ------------------------------------*/
typedef void (*ExceptionHandler)(void);
typedef union {
	ExceptionHandler handler;
	void *pointer;
} VectorTableEntry;

extern unsigned _estack;

/* External Interrupts */
void __attribute__((weak)) MPUFault_Handler(void);
void __attribute__((weak)) BusFault_Handler(void);
void __attribute__((weak)) UsageFault_Handler(void);
void __attribute__((weak)) SVCall_Handler(void);
void __attribute__((weak)) Debug_Handler(void);
void __attribute__((weak)) PendSV_Handler(void);
void __attribute__((weak)) SysTick_Handler(void);
void __attribute__((weak)) GPIOPortA_Handler(void);
void __attribute__((weak)) GPIOPortB_Handler(void);
void __attribute__((weak)) GPIOPortC_Handler(void);
void __attribute__((weak)) GPIOPortD_Handler(void);
void __attribute__((weak)) GPIOPortE_Handler(void);
void __attribute__((weak)) UART0_Handler(void);
void __attribute__((weak)) UART1_Handler(void);
void __attribute__((weak)) SSI0_Handler(void);
void __attribute__((weak)) I2C0_Handler(void);
void __attribute__((weak)) PWMFault_Handler(void);
void __attribute__((weak)) PWMGenerator0_Handler(void);
void __attribute__((weak)) PWMGenerator1_Handler(void);
void __attribute__((weak)) PWMGenerator2_Handler(void);
void __attribute__((weak)) QuadratureEncoder0_Handler(void);
void __attribute__((weak)) ADCSequence0_Handler(void);
void __attribute__((weak)) ADCSequence1_Handler(void);
void __attribute__((weak)) ADCSequence2_Handler(void);
void __attribute__((weak)) ADCSequence3_Handler(void);
void __attribute__((weak)) Watchdogtimer_Handler(void);
void __attribute__((weak)) Timer0subtimerA_Handler(void);
void __attribute__((weak)) Timer0subtimerB_Handler(void);
void __attribute__((weak)) Timer1subtimerA_Handler(void);
void __attribute__((weak)) Timer1subtimerB_Handler(void);
void __attribute__((weak)) Timer2subtimerA_Handler(void);
void __attribute__((weak)) Timer2subtimerB_Handler(void);
void __attribute__((weak)) AnalogComparator0_Handler(void);
void __attribute__((weak)) AnalogComparator1_Handler(void);
void __attribute__((weak)) AnalogComparator2_Handler(void);
void __attribute__((weak)) SystemControl_Handler(void);
void __attribute__((weak)) FLASHControl_Handler(void);
void __attribute__((weak)) GPIOPortF_Handler(void);
void __attribute__((weak)) GPIOPortG_Handler(void);
void __attribute__((weak)) GPIOPortH_Handler(void);
void __attribute__((weak)) UART2_Handler(void);
void __attribute__((weak)) SSI1_Handler(void);
void __attribute__((weak)) Timer3subtimerA_Handler(void);
void __attribute__((weak)) Timer3subtimerB_Handler(void);
void __attribute__((weak)) I2C1_Handler(void);
void __attribute__((weak)) QuadratureEncoder1_Handler(void);
void __attribute__((weak)) CAN0_Handler(void);
void __attribute__((weak)) CAN1_Handler(void);
void __attribute__((weak)) CAN2_Handler(void);
void __attribute__((weak)) Ethernet_Handler(void);
void __attribute__((weak)) Hibernate_Handler(void);
void __attribute__((weak)) USB0_Handler(void);
void __attribute__((weak)) PWMGenerator3_Handler(void);
void __attribute__((weak)) uDMASoftwareTransfer_Handler(void);
void __attribute__((weak)) uDMAError_Handler(void);
void __attribute__((weak)) ADC1Sequence0_Handler(void);
void __attribute__((weak)) ADC1Sequence1_Handler(void);
void __attribute__((weak)) ADC1Sequence2_Handler(void);
void __attribute__((weak)) ADC1Sequence3_Handler(void);
void __attribute__((weak)) I2S0_Handler(void);
void __attribute__((weak)) ExternalBusInterface0_Handler(void);
void __attribute__((weak)) GPIOPortJ_Handler(void);
void __attribute__((weak)) GPIOPortK_Handler(void);
void __attribute__((weak)) GPIOPortL_Handler(void);
void __attribute__((weak)) SSI2_Handler(void);
void __attribute__((weak)) SSI3_Handler(void);
void __attribute__((weak)) UART3_Handler(void);
void __attribute__((weak)) UART4_Handler(void);
void __attribute__((weak)) UART5_Handler(void);
void __attribute__((weak)) UART6_Handler(void);
void __attribute__((weak)) UART7_Handler(void);
void __attribute__((weak)) I2C2_Handler(void);
void __attribute__((weak)) I2C3_Handler(void);
void __attribute__((weak)) Timer4subtimerA_Handler(void);
void __attribute__((weak)) Timer4subtimerB_Handler(void);
void __attribute__((weak)) Timer5subtimerA_Handler(void);
void __attribute__((weak)) Timer5subtimerB_Handler(void);
void __attribute__((weak)) WideTimer0subtimerA_Handler(void);
void __attribute__((weak)) WideTimer0subtimerB_Handler(void);
void __attribute__((weak)) WideTimer1subtimerA_Handler(void);
void __attribute__((weak)) WideTimer1subtimerB_Handler(void);
void __attribute__((weak)) WideTimer2subtimerA_Handler(void);
void __attribute__((weak)) WideTimer2subtimerB_Handler(void);
void __attribute__((weak)) WideTimer3subtimerA_Handler(void);
void __attribute__((weak)) WideTimer3subtimerB_Handler(void);
void __attribute__((weak)) WideTimer4subtimerA_Handler(void);
void __attribute__((weak)) WideTimer4subtimerB_Handler(void);
void __attribute__((weak)) WideTimer5subtimerA_Handler(void);
void __attribute__((weak)) WideTimer5subtimerB_Handler(void);
void __attribute__((weak)) FPU_Handler(void);
void __attribute__((weak)) PECI0_Handler(void);
void __attribute__((weak)) LPC0_Handler(void);
void __attribute__((weak)) I2C4_Handler(void);
void __attribute__((weak)) I2C5_Handler(void);
void __attribute__((weak)) GPIOPortM_Handler(void);
void __attribute__((weak)) GPIOPortN_Handler(void);
void __attribute__((weak)) QuadratureEncoder2_Handler(void);
void __attribute__((weak)) Fan0_Handler(void);
void __attribute__((weak)) GPIOPortP_Handler(void);
void __attribute__((weak)) GPIOPortP1_Handler(void);
void __attribute__((weak)) GPIOPortP2_Handler(void);
void __attribute__((weak)) GPIOPortP3_Handler(void);
void __attribute__((weak)) GPIOPortP4_Handler(void);
void __attribute__((weak)) GPIOPortP5_Handler(void);
void __attribute__((weak)) GPIOPortP6_Handler(void);
void __attribute__((weak)) GPIOPortP7_Handler(void);
void __attribute__((weak)) GPIOPortQ_Handler(void);
void __attribute__((weak)) GPIOPortQ1_Handler(void);
void __attribute__((weak)) GPIOPortQ2_Handler(void);
void __attribute__((weak)) GPIOPortQ3_Handler(void);
void __attribute__((weak)) GPIOPortQ4_Handler(void);
void __attribute__((weak)) GPIOPortQ5_Handler(void);
void __attribute__((weak)) GPIOPortQ6_Handler(void);
void __attribute__((weak)) GPIOPortQ7_Handler(void);
void __attribute__((weak)) GPIOPortR_Handler(void);
void __attribute__((weak)) GPIOPortS_Handler(void);
void __attribute__((weak)) PWM1Generator0_Handler(void);
void __attribute__((weak)) PWM1Generator1_Handler(void);
void __attribute__((weak)) PWM1Generator2_Handler(void);
void __attribute__((weak)) PWM1Generator3_Handler(void);
void __attribute__((weak)) PWM1Fault_Handler(void);

/*----------------------------------------------------------------------------
 * weak aliases for each Exception handler to the Spurious_Handler.
 * Any function with the same name will override these definitions.
 */
#pragma weak MPUFault_Handler=IntDefaultHandler
#pragma weak BusFault_Handler=IntDefaultHandler
#pragma weak UsageFault_Handler=IntDefaultHandler
#pragma weak SVCall_Handler=IntDefaultHandler
#pragma weak Debug_Handler=IntDefaultHandler
#pragma weak PendSV_Handler=IntDefaultHandler
#pragma weak SysTick_Handler=IntDefaultHandler
#pragma weak GPIOPortA_Handler=IntDefaultHandler
#pragma weak GPIOPortB_Handler=IntDefaultHandler
#pragma weak GPIOPortC_Handler=IntDefaultHandler
#pragma weak GPIOPortD_Handler=IntDefaultHandler
#pragma weak GPIOPortE_Handler=IntDefaultHandler
#pragma weak UART0_Handler=IntDefaultHandler
#pragma weak UART1_Handler=IntDefaultHandler
#pragma weak SSI0_Handler=IntDefaultHandler
#pragma weak I2C0_Handler=IntDefaultHandler
#pragma weak PWMFault_Handler=IntDefaultHandler
#pragma weak PWMGenerator0_Handler=IntDefaultHandler
#pragma weak PWMGenerator1_Handler=IntDefaultHandler
#pragma weak PWMGenerator2_Handler=IntDefaultHandler
#pragma weak QuadratureEncoder0_Handler=IntDefaultHandler
#pragma weak ADCSequence0_Handler=IntDefaultHandler
#pragma weak ADCSequence1_Handler=IntDefaultHandler
#pragma weak ADCSequence2_Handler=IntDefaultHandler
#pragma weak ADCSequence3_Handler=IntDefaultHandler
#pragma weak Watchdogtimer_Handler=IntDefaultHandler
#pragma weak Timer0subtimerA_Handler=IntDefaultHandler
#pragma weak Timer0subtimerB_Handler=IntDefaultHandler
#pragma weak Timer1subtimerA_Handler=IntDefaultHandler
#pragma weak Timer1subtimerB_Handler=IntDefaultHandler
#pragma weak Timer2subtimerA_Handler=IntDefaultHandler
#pragma weak Timer2subtimerB_Handler=IntDefaultHandler
#pragma weak AnalogComparator0_Handler=IntDefaultHandler
#pragma weak AnalogComparator1_Handler=IntDefaultHandler
#pragma weak AnalogComparator2_Handler=IntDefaultHandler
#pragma weak SystemControl_Handler=IntDefaultHandler
#pragma weak FLASHControl_Handler=IntDefaultHandler
#pragma weak GPIOPortF_Handler=IntDefaultHandler
#pragma weak GPIOPortG_Handler=IntDefaultHandler
#pragma weak GPIOPortH_Handler=IntDefaultHandler
#pragma weak UART2_Handler=IntDefaultHandler
#pragma weak SSI1_Handler=IntDefaultHandler
#pragma weak Timer3subtimerA_Handler=IntDefaultHandler
#pragma weak Timer3subtimerB_Handler=IntDefaultHandler
#pragma weak I2C1_Handler=IntDefaultHandler
#pragma weak QuadratureEncoder1_Handler=IntDefaultHandler
#pragma weak CAN0_Handler=IntDefaultHandler
#pragma weak CAN1_Handler=IntDefaultHandler
#pragma weak CAN2_Handler=IntDefaultHandler
#pragma weak Ethernet_Handler=IntDefaultHandler
#pragma weak Hibernate_Handler=IntDefaultHandler
#pragma weak USB0_Handler=IntDefaultHandler
#pragma weak PWMGenerator3_Handler=IntDefaultHandler
#pragma weak uDMASoftwareTransfer_Handler=IntDefaultHandler
#pragma weak uDMAError_Handler=IntDefaultHandler
#pragma weak ADC1Sequence0_Handler=IntDefaultHandler
#pragma weak ADC1Sequence1_Handler=IntDefaultHandler
#pragma weak ADC1Sequence2_Handler=IntDefaultHandler
#pragma weak ADC1Sequence3_Handler=IntDefaultHandler
#pragma weak I2S0_Handler=IntDefaultHandler
#pragma weak ExternalBusInterface0_Handler=IntDefaultHandler
#pragma weak GPIOPortJ_Handler=IntDefaultHandler
#pragma weak GPIOPortK_Handler=IntDefaultHandler
#pragma weak GPIOPortL_Handler=IntDefaultHandler
#pragma weak SSI2_Handler=IntDefaultHandler
#pragma weak SSI3_Handler=IntDefaultHandler
#pragma weak UART3_Handler=IntDefaultHandler
#pragma weak UART4_Handler=IntDefaultHandler
#pragma weak UART5_Handler=IntDefaultHandler
#pragma weak UART6_Handler=IntDefaultHandler
#pragma weak UART7_Handler=IntDefaultHandler
#pragma weak I2C2_Handler=IntDefaultHandler
#pragma weak I2C3_Handler=IntDefaultHandler
#pragma weak Timer4subtimerA_Handler=IntDefaultHandler
#pragma weak Timer4subtimerB_Handler=IntDefaultHandler
#pragma weak Timer5subtimerA_Handler=IntDefaultHandler
#pragma weak Timer5subtimerB_Handler=IntDefaultHandler
#pragma weak WideTimer0subtimerA_Handler=IntDefaultHandler
#pragma weak WideTimer0subtimerB_Handler=IntDefaultHandler
#pragma weak WideTimer1subtimerA_Handler=IntDefaultHandler
#pragma weak WideTimer1subtimerB_Handler=IntDefaultHandler
#pragma weak WideTimer2subtimerA_Handler=IntDefaultHandler
#pragma weak WideTimer2subtimerB_Handler=IntDefaultHandler
#pragma weak WideTimer3subtimerA_Handler=IntDefaultHandler
#pragma weak WideTimer3subtimerB_Handler=IntDefaultHandler
#pragma weak WideTimer4subtimerA_Handler=IntDefaultHandler
#pragma weak WideTimer4subtimerB_Handler=IntDefaultHandler
#pragma weak WideTimer5subtimerA_Handler=IntDefaultHandler
#pragma weak WideTimer5subtimerB_Handler=IntDefaultHandler
#pragma weak FPU_Handler=IntDefaultHandler
#pragma weak PECI0_Handler=IntDefaultHandler
#pragma weak LPC0_Handler=IntDefaultHandler
#pragma weak I2C4_Handler=IntDefaultHandler
#pragma weak I2C5_Handler=IntDefaultHandler
#pragma weak GPIOPortM_Handler=IntDefaultHandler
#pragma weak GPIOPortN_Handler=IntDefaultHandler
#pragma weak QuadratureEncoder2_Handler=IntDefaultHandler
#pragma weak Fan0_Handler=IntDefaultHandler
#pragma weak GPIOPortP_Handler=IntDefaultHandler
#pragma weak GPIOPortP1_Handler=IntDefaultHandler
#pragma weak GPIOPortP2_Handler=IntDefaultHandler
#pragma weak GPIOPortP3_Handler=IntDefaultHandler
#pragma weak GPIOPortP4_Handler=IntDefaultHandler
#pragma weak GPIOPortP5_Handler=IntDefaultHandler
#pragma weak GPIOPortP6_Handler=IntDefaultHandler
#pragma weak GPIOPortP7_Handler=IntDefaultHandler
#pragma weak GPIOPortQ_Handler=IntDefaultHandler
#pragma weak GPIOPortQ1_Handler=IntDefaultHandler
#pragma weak GPIOPortQ2_Handler=IntDefaultHandler
#pragma weak GPIOPortQ3_Handler=IntDefaultHandler
#pragma weak GPIOPortQ4_Handler=IntDefaultHandler
#pragma weak GPIOPortQ5_Handler=IntDefaultHandler
#pragma weak GPIOPortQ6_Handler=IntDefaultHandler
#pragma weak GPIOPortQ7_Handler=IntDefaultHandler
#pragma weak GPIOPortR_Handler=IntDefaultHandler
#pragma weak GPIOPortS_Handler=IntDefaultHandler
#pragma weak PWM1Generator0_Handler=IntDefaultHandler
#pragma weak PWM1Generator1_Handler=IntDefaultHandler
#pragma weak PWM1Generator2_Handler=IntDefaultHandler
#pragma weak PWM1Generator3_Handler=IntDefaultHandler
#pragma weak PWM1Fault_Handler=IntDefaultHandler


//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
VectorTableEntry const g_pfnVectors[] =
{
    { .pointer = &_estack },				// The initial stack pointer
    { .handler = &Reset_Handler},           // The reset handler
    { .handler = &NmiSR},                   // The NMI handler
    { .handler = &FaultISR},                // The hard fault handler
    { .handler = &MPUFault_Handler},                      // The MPU fault handler
    { .handler = &BusFault_Handler},                      // The bus fault handler
    { .handler = &UsageFault_Handler},                      // The usage fault handler
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = &SVCall_Handler},                      // SVCall handler
    { .handler = &Debug_Handler},                      // Debug monitor handler
    { .handler = 0},                                      // Reserved
    { .handler = &PendSV_Handler},                      // The PendSV handler
    { .handler = &SysTick_Handler},                      // The SysTick handler
    { .handler = &GPIOPortA_Handler},                      // GPIO Port A
    { .handler = &GPIOPortB_Handler},                      // GPIO Port B
    { .handler = &GPIOPortC_Handler},                      // GPIO Port C
    { .handler = &GPIOPortD_Handler},                      // GPIO Port D
    { .handler = &GPIOPortE_Handler},                      // GPIO Port E
    { .handler = &UART0_Handler},                      // UART0 Rx and Tx
    { .handler = &UART1_Handler},                      // UART1 Rx and Tx
    { .handler = &SSI0_Handler},                      // SSI0 Rx and Tx
    { .handler = &I2C0_Handler},                      // I2C0 Master and Slave
    { .handler = &PWMFault_Handler},                      // PWM Fault
    { .handler = &PWMGenerator0_Handler},                      // PWM Generator 0
    { .handler = &PWMGenerator1_Handler},                      // PWM Generator 1
    { .handler = &PWMGenerator2_Handler},                      // PWM Generator 2
    { .handler = &QuadratureEncoder0_Handler},                      // Quadrature Encoder 0
    { .handler = &ADCSequence0_Handler},                      // ADC Sequence 0
    { .handler = &ADCSequence1_Handler},                      // ADC Sequence 1
    { .handler = &ADCSequence2_Handler},                      // ADC Sequence 2
    { .handler = &ADCSequence3_Handler},                      // ADC Sequence 3
    { .handler = &Watchdogtimer_Handler},                      // Watchdog timer
    { .handler = &Timer0subtimerA_Handler},                      // Timer 0 subtimer A
    { .handler = &Timer0subtimerB_Handler},                      // Timer 0 subtimer B
    { .handler = &Timer1subtimerA_Handler},                      // Timer 1 subtimer A
    { .handler = &Timer1subtimerB_Handler},                      // Timer 1 subtimer B
    { .handler = &Timer2subtimerA_Handler},                      // Timer 2 subtimer A
    { .handler = &Timer2subtimerB_Handler},                      // Timer 2 subtimer B
    { .handler = &AnalogComparator0_Handler},                      // Analog Comparator 0
    { .handler = &AnalogComparator1_Handler},                      // Analog Comparator 1
    { .handler = &AnalogComparator2_Handler},                      // Analog Comparator 2
    { .handler = &SystemControl_Handler},                      // System Control (PLL, OSC, BO)
    { .handler = &FLASHControl_Handler},                      // FLASH Control
    { .handler = &GPIOPortF_Handler},                      // GPIO Port F
    { .handler = &GPIOPortG_Handler},                      // GPIO Port G
    { .handler = &GPIOPortH_Handler},                      // GPIO Port H
    { .handler = &UART2_Handler},                      // UART2 Rx and Tx
    { .handler = &SSI1_Handler},                      // SSI1 Rx and Tx
    { .handler = &Timer3subtimerA_Handler},                      // Timer 3 subtimer A
    { .handler = &Timer3subtimerB_Handler},                      // Timer 3 subtimer B
    { .handler = &I2C1_Handler},                      // I2C1 Master and Slave
    { .handler = &QuadratureEncoder1_Handler},                      // Quadrature Encoder 1
    { .handler = &CAN0_Handler},                      // CAN0
    { .handler = &CAN1_Handler},                      // CAN1
    { .handler = &CAN2_Handler},                      // CAN2
    { .handler = &Ethernet_Handler},                      // Ethernet
    { .handler = &Hibernate_Handler},                      // Hibernate
    { .handler = &USB0_Handler},                      // USB0
    { .handler = &PWMGenerator3_Handler},                      // PWM Generator 3
    { .handler = &uDMASoftwareTransfer_Handler},                      // uDMA Software Transfer
    { .handler = &uDMAError_Handler},                      // uDMA Error
    { .handler = &ADC1Sequence0_Handler},                      // ADC1 Sequence 0
    { .handler = &ADC1Sequence1_Handler},                      // ADC1 Sequence 1
    { .handler = &ADC1Sequence2_Handler},                      // ADC1 Sequence 2
    { .handler = &ADC1Sequence3_Handler},                      // ADC1 Sequence 3
    { .handler = &I2S0_Handler},                      // I2S0
    { .handler = &ExternalBusInterface0_Handler},                      // External Bus Interface 0
    { .handler = &GPIOPortJ_Handler},                      // GPIO Port J
    { .handler = &GPIOPortK_Handler},                      // GPIO Port K
    { .handler = &GPIOPortL_Handler},                      // GPIO Port L
    { .handler = &SSI2_Handler},                      // SSI2 Rx and Tx
    { .handler = &SSI3_Handler},                      // SSI3 Rx and Tx
    { .handler = &UART3_Handler},                      // UART3 Rx and Tx
    { .handler = &UART4_Handler},                      // UART4 Rx and Tx
    { .handler = &UART5_Handler},                      // UART5 Rx and Tx
    { .handler = &UART6_Handler},                      // UART6 Rx and Tx
    { .handler = &UART7_Handler},                      // UART7 Rx and Tx
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = &I2C2_Handler},                      // I2C2 Master and Slave
    { .handler = &I2C3_Handler},                      // I2C3 Master and Slave
    { .handler = &Timer4subtimerA_Handler},                      // Timer 4 subtimer A
    { .handler = &Timer4subtimerB_Handler},                      // Timer 4 subtimer B
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = 0},                                      // Reserved
    { .handler = &Timer5subtimerA_Handler},                      // Timer 5 subtimer A
    { .handler = &Timer5subtimerB_Handler},                      // Timer 5 subtimer B
    { .handler = &WideTimer0subtimerA_Handler},                      // Wide Timer 0 subtimer A
    { .handler = &WideTimer0subtimerB_Handler},                      // Wide Timer 0 subtimer B
    { .handler = &WideTimer1subtimerA_Handler},                      // Wide Timer 1 subtimer A
    { .handler = &WideTimer1subtimerB_Handler},                      // Wide Timer 1 subtimer B
    { .handler = &WideTimer2subtimerA_Handler},                      // Wide Timer 2 subtimer A
    { .handler = &WideTimer2subtimerB_Handler},                      // Wide Timer 2 subtimer B
    { .handler = &WideTimer3subtimerA_Handler},                      // Wide Timer 3 subtimer A
    { .handler = &WideTimer3subtimerB_Handler},                      // Wide Timer 3 subtimer B
    { .handler = &WideTimer4subtimerA_Handler},                      // Wide Timer 4 subtimer A
    { .handler = &WideTimer4subtimerB_Handler},                      // Wide Timer 4 subtimer B
    { .handler = &WideTimer5subtimerA_Handler},                      // Wide Timer 5 subtimer A
    { .handler = &WideTimer5subtimerB_Handler},                      // Wide Timer 5 subtimer B
    { .handler = &FPU_Handler},                      // FPU
    { .handler = &PECI0_Handler},                      // PECI 0
    { .handler = &LPC0_Handler},                      // LPC 0
    { .handler = &I2C4_Handler},                      // I2C4 Master and Slave
    { .handler = &I2C5_Handler},                      // I2C5 Master and Slave
    { .handler = &GPIOPortM_Handler},                      // GPIO Port M
    { .handler = &GPIOPortN_Handler},                      // GPIO Port N
    { .handler = &QuadratureEncoder2_Handler},                      // Quadrature Encoder 2
    { .handler = &Fan0_Handler},                      // Fan 0
    { .handler = 0},                                      // Reserved
    { .handler = &GPIOPortP_Handler},                      // GPIO Port P (Summary or P0)
    { .handler = &GPIOPortP1_Handler},                      // GPIO Port P1
    { .handler = &GPIOPortP2_Handler},                      // GPIO Port P2
    { .handler = &GPIOPortP3_Handler},                      // GPIO Port P3
    { .handler = &GPIOPortP4_Handler},                      // GPIO Port P4
    { .handler = &GPIOPortP5_Handler},                      // GPIO Port P5
    { .handler = &GPIOPortP6_Handler},                      // GPIO Port P6
    { .handler = &GPIOPortP7_Handler},                      // GPIO Port P7
    { .handler = &GPIOPortQ_Handler},                      // GPIO Port Q (Summary or Q0)
    { .handler = &GPIOPortQ1_Handler},                      // GPIO Port Q1
    { .handler = &GPIOPortQ2_Handler},                      // GPIO Port Q2
    { .handler = &GPIOPortQ3_Handler},                      // GPIO Port Q3
    { .handler = &GPIOPortQ4_Handler},                      // GPIO Port Q4
    { .handler = &GPIOPortQ5_Handler},                      // GPIO Port Q5
    { .handler = &GPIOPortQ6_Handler},                      // GPIO Port Q6
    { .handler = &GPIOPortQ7_Handler},                      // GPIO Port Q7
    { .handler = &GPIOPortR_Handler},                      // GPIO Port R
    { .handler = &GPIOPortS_Handler},                      // GPIO Port S
    { .handler = &PWM1Generator0_Handler},                      // PWM 1 Generator 0
    { .handler = &PWM1Generator1_Handler},                      // PWM 1 Generator 1
    { .handler = &PWM1Generator2_Handler},                      // PWM 1 Generator 2
    { .handler = &PWM1Generator3_Handler},                      // PWM 1 Generator 3
    { .handler = &PWM1Fault_Handler}                       // PWM 1 Fault
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _sidata;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void Reset_Handler(void)
{
    unsigned long *pulSrc, *pulDest;
    unsigned long *dst;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_sidata;
    for(pulDest = &_sdata; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

	/* zero fill the .bss segment... */
	for (dst = &_sbss; dst < &_ebss; ++dst) {
		*dst = 0;
	}


	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	HWREG(NVIC_FPCC) |= NVIC_FPCC_ASPEN | NVIC_FPCC_LSPEN;

    //
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).  Any configuration of the floating-point unit using DriverLib
    // APIs must be done here prior to the floating-point unit being enabled.
    //
    // Note that this does not use DriverLib since it might not be included in
    // this project.
    //
    HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) &
                         ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                        NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

    //
    // Call the application's entry point.
    //
    main();
}

/**
 * Initialize static objects and run constructors
 */
void _init(void) {
	asm volatile (
	"		ldr		r3, =__init_array_start"   "\n\t" // load adress of __ctors_start__ into r0
	"		ldr		r4, =__init_array_end"		"\n\t"
	"		subs	r4, r4, r3		"	    "\n\t" // Length of ctors section
	"		beq		exit_init		"	    "\n\t" // Skip if no constructors
	"ctors_loop:					"		"\n\t"
	"		ldr		r0, [r3], #4	"		"\n\t" // Load next constructor address
	"		blx		r0				"		"\n\t" // Call constructor
	"		subs	r4, r4, #4		"		"\n\t" // Decrement counter
	"		bgt		ctors_loop		"		"\n\t" // Repeat until done
	"exit_init: "
	:  :  : "r0", "r3", "r4");
}


//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
void NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
void FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
void IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}
