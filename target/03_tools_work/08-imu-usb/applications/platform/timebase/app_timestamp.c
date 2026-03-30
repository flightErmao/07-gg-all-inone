#include "app_timestamp.h"

#include "main.h"

static volatile rt_bool_t s_app_timestamp_ready = RT_FALSE;
static rt_uint32_t s_last_counter = 0U;
static rt_uint32_t s_wrap_count = 0U;

static rt_uint32_t app_timestamp_timer_clock_hz(void)
{
    RCC_ClkInitTypeDef clock_config;
    rt_uint32_t flash_latency = 0U;
    rt_uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();

    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
    if (clock_config.APB1CLKDivider == RCC_HCLK_DIV1)
    {
        return pclk1_hz;
    }
    return pclk1_hz * 2U;
}

static int app_timestamp_init(void)
{
    rt_uint32_t timer_clock_hz;
    rt_uint32_t prescaler;

    if (s_app_timestamp_ready == RT_TRUE)
    {
        return RT_EOK;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();

    timer_clock_hz = app_timestamp_timer_clock_hz();
    if (timer_clock_hz < 1000000U)
    {
        return -RT_ERROR;
    }

    prescaler = (timer_clock_hz / 1000000U) - 1U;
    TIM2->CR1 = 0U;
    TIM2->CR2 = 0U;
    TIM2->SMCR = 0U;
    TIM2->DIER = 0U;
    TIM2->SR = 0U;
    TIM2->PSC = prescaler;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;

    s_last_counter = 0U;
    s_wrap_count = 0U;
    s_app_timestamp_ready = RT_TRUE;
    return RT_EOK;
}
INIT_DEVICE_EXPORT(app_timestamp_init);

rt_uint64_t app_timestamp_now_us(void)
{
    rt_base_t level;
    rt_uint32_t counter_now;
    rt_uint64_t timestamp_us;

    if ((s_app_timestamp_ready == RT_FALSE) && (app_timestamp_init() != RT_EOK))
    {
        return 0ULL;
    }

    level = rt_hw_interrupt_disable();
    counter_now = TIM2->CNT;
    if (counter_now < s_last_counter)
    {
        s_wrap_count++;
    }
    s_last_counter = counter_now;
    timestamp_us = (((rt_uint64_t)s_wrap_count) << 32) | counter_now;
    rt_hw_interrupt_enable(level);

    return timestamp_us;
}

rt_uint32_t app_timestamp_now_ms(void)
{
    return (rt_uint32_t)(app_timestamp_now_us() / 1000ULL);
}
