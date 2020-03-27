
#include "LPC11Uxx.h"

// #define ENABLE_VOLTMETER

#ifdef ENABLE_VOLTMETER

#define ADC_DONE (0x80000000)
#define ADC_CHANNELS (8)

// max = 4.5 MHz
// If PCLK is 48 MHz, max = 48 / 11 (about 4.36 MHz)
#define ADC_CLK (4360000)

#define AMP_PORT 1
#define AMP_PIN 15

static void timer_init(void);
static void timer_fini(void);
static void timer_set_period(uint32_t us);
static void adc_init(uint8_t div);
static void adc_fini(void);
static void amp_init(void);
static void amp_set(uint8_t amp);

__WEAK void voltmeter_isr(uint16_t ch6, uint16_t ch7)
{
}

void voltmeter_start(uint32_t options)
{
    uint8_t amp = 0;
    uint8_t div = 12;

    if (options & 0x40000000) {
        amp = (options >> 8) & 0xFF;
        div = (options >> 16) & 0xFF;
    }
    amp_init();
    amp_set(amp);

    adc_init(div);
    // timer_init();
    // timer_set_period(voltmeter_period);
}

void voltmeter_stop()
{
    // timer_fini();
    adc_fini();
}

void voltmeter_set_amp(uint8_t amp)
{
    amp_set(amp);
}

void voltmeter_set_period(uint32_t us)
{
    timer_set_period(us);
}

void ADC_IRQHandler(void)
{
    voltmeter_isr(LPC_ADC->DR[6], LPC_ADC->DR[7]);
}

static void adc_init(uint8_t div)
{
    /* Disable Power down bit to the ADC block. */
    LPC_SYSCON->PDRUNCFG &= ~(0x1 << 4);

    /* Enable AHB clock to the ADC. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 13);

    /* P0.11 = ADC0 */
    // LPC_IOCON->TDI_PIO0_11   &= ~0x9F;
    // LPC_IOCON->TDI_PIO0_11   |= 0x02;

    /* P0.12 = ADC1 */
    // LPC_IOCON->TMS_PIO0_12   &= ~0x9F;
    // LPC_IOCON->TMS_PIO0_12   |= 0x02;

    /* P0.13 = ADC2 */
    // LPC_IOCON->TDO_PIO0_13   &= ~0x9F;
    // LPC_IOCON->TDO_PIO0_13   |= 0x02;

    /* P0.14 = ADC3 */
    // LPC_IOCON->TRST_PIO0_14  &= ~0x9F;
    // LPC_IOCON->TRST_PIO0_14  |= 0x02;

    /* P0.15 = ADC4 ... this is also SWDIO so be careful with this pin! */
    // LPC_IOCON->SWDIO_PIO0_15 &= ~0x9F;
    // LPC_IOCON->SWDIO_PIO0_15 |= 0x02;

    /* P0.16 = ADC5 */
    // LPC_IOCON->PIO0_16 &= ~0x9F;
    // LPC_IOCON->PIO0_16 |= 0x01;

    /* P0.22 = ADC6 */
    LPC_IOCON->PIO0_22 &= ~0x9F;
    LPC_IOCON->PIO0_22 |= 0x01;

    /* P0.23 = ADC7 */
    LPC_IOCON->PIO0_23 &= ~0x9F;
    LPC_IOCON->PIO0_23 |= 0x01;

    SystemCoreClockUpdate();

    /* Setup the ADC clock, conversion mode, etc. */
    // LPC_ADC->CR = (1 << 6) |                               // ADC6
    //               ((SystemCoreClock / ADC_CLK - 1) << 8) | // CLKDIV = Fpclk / Fadc - 1
    //               (0 << 16) |                              // BURST = 0, no BURST, software controlled
    //               (0 << 19) |                              // 10 bits (11 clocks)
    //               (4 << 24) |                              // ADC convertion started by CT32B0
    //               (0 << 27);                               // EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion)


    LPC_ADC->CR = (1 << 6) | (1 << 7) |                    // ADC6
                  ((div - 1) << 8) |                       // CLKDIV = Fpclk / Fadc - 1
                  (1 << 16) |                              // BURST = 1
                  (0 << 19);                               // 10 bits (11 clocks)

    NVIC_EnableIRQ(ADC_IRQn);
    LPC_ADC->INTEN = (1 << 7);   // Enable ADC7 interrupt
}

static void adc_fini()
{

    // LPC_ADC->CR &= ~(0x7 << 24); /* stop ADC */
    LPC_ADC->CR = 0;

    /* Enable Power down bit to the ADC block. */
    LPC_SYSCON->PDRUNCFG |= (0x1 << 4);

    /* Disable AHB clock to the ADC. */
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 13);

    NVIC_DisableIRQ(ADC_IRQn);
}

// set CT32B0 MAT0 to trigger ADC convertion
static void timer_init()
{
    uint32_t ticks;

    /* Make sure 32-bit timer 0 is enabled */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 9);

    //   /* Setup the external match register (clear on match) */
    //   LPC_CT32B0->EMR =  (1<<10) | (1<<8) | (1<<6) | (1<<4) |
    //                      (1<<0)  | (1<<1) | (1<<2) | (1<<3);

    /* Enable PWM function */
    LPC_CT32B0->PWMC = (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);

    /* Reset Functionality on MR3 controlling the PWM period */
    LPC_CT32B0->MCR = 1 << 10;

    SystemCoreClockUpdate();
    ticks = (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)100000) / (uint64_t)1000000);
    LPC_CT32B0->MR3 = ticks;

    LPC_CT32B0->MR0 = LPC_CT32B0->MR3 >> 1;

    LPC_CT32B0->TCR = 1;
}

static void timer_set_period(uint32_t us)
{
    uint32_t ticks = (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)us) / (uint64_t)1000000);
    LPC_CT32B0->MR3 = ticks;
    LPC_CT32B0->TCR = (0x1 << 1); // reset
    LPC_CT32B0->TCR = (0x1 << 0); // start
}

static void timer_fini()
{
    LPC_CT32B0->TCR = 0;

    /* Disable AHB clock to the CT32B0 */
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 9);
}

static void amp_init()
{
    // enable clock for GPIO
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6);

    LPC_IOCON->PIO1_15 &= ~0x9F;

    LPC_GPIO->DIR[AMP_PORT] |= (1 << AMP_PIN);
}

static void amp_set(uint8_t amp)
{
    if (amp) {
        LPC_GPIO->SET[AMP_PORT] |= (1 << AMP_PIN);
    } else {
        LPC_GPIO->CLR[AMP_PORT] |= (1 << AMP_PIN);
    }
}

#endif // ENABLE_VOLTMETER
