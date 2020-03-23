
#include "LPC11Uxx.h"

// #define ENABLE_VOLTMETER

#ifdef ENABLE_VOLTMETER

#define ADC_DONE (0x80000000)
#define ADC_CHANNELS (8)
#define ADC_CLK (4200000) /* about 4.2MHz */
// #define ADC_CLK (10000) /* about 10KHz */

#define RANGE_PORT 1
#define RANGE_PIN 15

volatile uint8_t current_range = 0;        // current measurement range
volatile uint32_t voltmeter_period = 100000; // us

static void timer_init(void);
static void timer_fini(void);
static void timer_set_period(uint32_t us);
static void adc_init(void);
static void adc_fini(void);
static void range_init(void);
static void range_set(uint8_t range);

__WEAK void voltmeter_isr(uint8_t channel, uint16_t value)
{
}

void voltmeter_start(uint32_t options)
{
    uint8_t range = current_range;
    uint32_t period = voltmeter_period;

    if (options & 0x40000000) {
        // uint8_t channels = (uint8_t)options;
        range = (options >> 8) & 0xF;
        period = ((options >> 16) & 0x3FF) * 1000;
    }
    range_init();
    range_set(range);

    adc_init();
    timer_init();
    timer_set_period(period);
}

void voltmeter_stop()
{
    timer_fini();
    adc_fini();
}

void voltmeter_set_period(uint32_t us)
{
    timer_set_period(us);

    voltmeter_period = us;
}

int voltmeter_convert(char *buf, int size, uint16_t value)
{
    const char *hex = "0123456789ABCDEF";
    if (size < 5)
    {
        return 0;
    }

    buf[0] = hex[value >> 12];
    buf[1] = hex[(value >> 8) & 0xF];
    buf[2] = hex[(value >> 4) & 0xF];
    buf[3] = hex[value & 0xF];

    buf[4] = '\n';

    return 5;
}

void ADC_IRQHandler(void)
{
    uint32_t data = LPC_ADC->DR[6];
    uint16_t value = (uint16_t)data;
    uint8_t channel = (data >> 24) & 0x7;

    voltmeter_isr(channel, value);
}

static void adc_init(void)
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
    // LPC_IOCON->PIO0_16       &= ~0x9F;
    // LPC_IOCON->PIO0_16       |= 0x01;

    /* P0.22 = ADC6 */
    LPC_IOCON->PIO0_22 &= ~0x9F;
    LPC_IOCON->PIO0_22 |= 0x01;

    /* P0.23 = ADC7 */
    // LPC_IOCON->PIO0_23       &= ~0x9F;
    // LPC_IOCON->PIO0_23       |= 0x01;

    SystemCoreClockUpdate();

    /* Setup the ADC clock, conversion mode, etc. */
    LPC_ADC->CR = (1 << 6) |                               // ADC6
                  ((SystemCoreClock / ADC_CLK - 1) << 8) | // CLKDIV = Fpclk / Fadc - 1
                  (0 << 16) |                              // BURST = 0, no BURST, software controlled
                  (0 << 19) |                              // 10 bits (11 clocks)
                  (4 << 24) |                              // ADC convertion started by CT32B0
                  (0 << 27);                               // EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion)


    // LPC_ADC->CR = (1 << 6) |                               // ADC6
    //               ((SystemCoreClock / ADC_CLK - 1) << 8) | // CLKDIV = Fpclk / Fadc - 1
    //               (1 << 16) |                              // BURST = 1
    //               (0 << 19) |                              // 10 bits (11 clocks)
    //               (0 << 24) |                              // ADC convertion started
    //               (0 << 27);                               // EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion)

    NVIC_EnableIRQ(ADC_IRQn);
    LPC_ADC->INTEN = 1 << 6; /* Enable ADC6 interrupt */
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
    ticks = (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)voltmeter_period) / (uint64_t)1000000);
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

static void range_init()
{
    // enable clock for GPIO
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6);

    LPC_IOCON->PIO1_15 &= ~0x9F;

    LPC_GPIO->DIR[RANGE_PORT] |= (1 << RANGE_PIN);
}

static void range_set(uint8_t range)
{
    current_range = range;
    if (range)
    {
        LPC_GPIO->CLR[RANGE_PORT] |= (1 << RANGE_PIN);
    }
    else
    {
        LPC_GPIO->SET[RANGE_PORT] |= (1 << RANGE_PIN);
    }
}

#endif // ENABLE_VOLTMETER
