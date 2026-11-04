#include "ccd.h"
#include "adc.h"

/* ====================== 可在 Keil Watch 中观察的变量 ====================== */
volatile uint16_t CCD_Raw[CCD_PIXEL_COUNT] = {0};

volatile uint16_t CCD_Min       = 0;
volatile uint16_t CCD_Max       = 0;
volatile uint16_t CCD_Threshold = 0;
volatile uint16_t CCD_Left      = 0xFFFFU;
volatile uint16_t CCD_Right     = 0xFFFFU;
volatile uint16_t CCD_Center    = 0xFFFFU;
volatile uint8_t  CCD_LineValid = 0U;

/* DWT: Cortex-M4 cycle counter for controlled us delay */
static uint8_t s_dwt_ready = 0U;

static void CCD_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_ready = 1U;
}

static void CCD_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (s_dwt_ready == 0U)
    {
        CCD_DWT_Init();
    }

    ticks = (SystemCoreClock / 1000000U) * us;
    start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
    }
}

void CCD_Init(void)
{
    CCD_DWT_Init();

    HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
}

/*
 * Read one TSL1401 frame (SI + 128 pixel clocks).
 * ADC: ADC1_IN4 (PA4), 12-bit, software trigger, single conversion.
 */
HAL_StatusTypeDef CCD_ReadFrame(void)
{
    uint32_t i;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
    CCD_DelayUs(2U);

    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
    CCD_DelayUs(2U);

    HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
    CCD_DelayUs(2U);

    HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
    CCD_DelayUs(2U);

    for (i = 0U; i < CCD_PIXEL_COUNT; i++)
    {
        HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);
        CCD_DelayUs(2U);

        status = HAL_ADC_Start(&hadc1);
        if (status != HAL_OK)
        {
            HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
            return status;
        }

        status = HAL_ADC_PollForConversion(&hadc1, 10U);
        if (status == HAL_OK)
        {
            CCD_Raw[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }

        (void)HAL_ADC_Stop(&hadc1);

        if (status != HAL_OK)
        {
            HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
            return status;
        }

        HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
        CCD_DelayUs(2U);
    }

    return HAL_OK;
}

/*
 * Black-line detect on white background (ADC below threshold = dark).
 * Valid only when:
 *   contrast (max-min) >= CCD_MIN_CONTRAST
 *   width (right-left) in [CCD_MIN_WIDTH, CCD_MAX_WIDTH]
 * Edges use 3-bright / 3-dark runs to reduce single-pixel noise.
 */
void CCD_ProcessFrame(void)
{
    uint16_t min_value = 0xFFFFU;
    uint16_t max_value = 0U;
    uint16_t threshold;
    uint16_t i;
    int32_t j;
    uint16_t left = 0xFFFFU;
    uint16_t right = 0xFFFFU;
    uint8_t left_found = 0U;
    uint8_t right_found = 0U;
    uint16_t contrast;
    uint16_t width;

    CCD_LineValid = 0U;
    CCD_Left = 0xFFFFU;
    CCD_Right = 0xFFFFU;
    CCD_Center = 0xFFFFU;

    for (i = 5U; i < (CCD_PIXEL_COUNT - 5U); i++)
    {
        if (CCD_Raw[i] < min_value)
        {
            min_value = CCD_Raw[i];
        }
        if (CCD_Raw[i] > max_value)
        {
            max_value = CCD_Raw[i];
        }
    }

    threshold = (uint16_t)(((uint32_t)min_value + (uint32_t)max_value) / 2U);

    CCD_Min = min_value;
    CCD_Max = max_value;
    CCD_Threshold = threshold;

    contrast = (uint16_t)((uint32_t)max_value - (uint32_t)min_value);
    if (contrast < CCD_MIN_CONTRAST)
    {
        return;
    }

    /* Left edge: bright,bright,bright -> dark,dark,dark */
    for (i = 5U; i < (CCD_PIXEL_COUNT - 5U); i++)
    {
        if ((CCD_Raw[i]      > threshold) &&
            (CCD_Raw[i + 1U] > threshold) &&
            (CCD_Raw[i + 2U] > threshold) &&
            (CCD_Raw[i + 3U] < threshold) &&
            (CCD_Raw[i + 4U] < threshold) &&
            (CCD_Raw[i + 5U] < threshold))
        {
            left = i;
            left_found = 1U;
            break;
        }
    }

    /* Right edge: dark,dark,dark -> bright,bright,bright (rightmost) */
    for (j = (int32_t)CCD_PIXEL_COUNT - 6; j >= 5; j--)
    {
        if ((CCD_Raw[j]     < threshold) &&
            (CCD_Raw[j + 1] < threshold) &&
            (CCD_Raw[j + 2] < threshold) &&
            (CCD_Raw[j + 3] > threshold) &&
            (CCD_Raw[j + 4] > threshold) &&
            (CCD_Raw[j + 5] > threshold))
        {
            right = (uint16_t)j;
            right_found = 1U;
            break;
        }
    }

    CCD_Left = left;
    CCD_Right = right;

    if ((left_found != 0U) && (right_found != 0U) && (right > left))
    {
        width = (uint16_t)(right - left);
        if ((width >= CCD_MIN_WIDTH) && (width <= CCD_MAX_WIDTH))
        {
            CCD_Center = (uint16_t)(((uint32_t)left + (uint32_t)right) / 2U);
            CCD_LineValid = 1U;
        }
    }
}
