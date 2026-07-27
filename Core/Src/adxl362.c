/********************************************************************************
 Author : CAC (China Applications Support Team)

 Date :   May, 2014

 File name :  ADXL362.c

 Description :	 ADXL362 SPI communication driver

 Hardware plateform : 	EVAL-ADuCM360MKZ and EVAL-ADXL362Z

 Connection:
                 EVAL-ADuCM360MKZ       EVAL-ADXL362Z

                 P1.4:MISO,             MISO
                 P1.5:SCLK,             SCLK
                 P1.6:MOSI,             MOSI
                 P1.7:GPIO as CS,       CS
********************************************************************************/

#include "math.h"
#include "adxl362.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "cc1101.h"

axis_info_int16_t three_axis_info[_AXIS_LEN];
axis_info_int16_t diff_three_axis_info[_DIFF_CNT];
axis_info_int32_t three_axis_average_info;
axis_info_int32_t sum_info;
threshold_judge_t threshold_judge;
action_classify_t action_classify;
uint8_t action_classify_array[6];
uint8_t action;

int32_t memory_array[_MEM_ROWS][_MEM_COLS];
uint8_t memory_index = 0;
uint8_t memory_index_o;
uint8_t mid_index = 0;

uint8_t movement_cnt = 0;
uint8_t climb_cnt = 0;
uint8_t rest_cnt1 = 0;
uint8_t rest_cnt2 = 0;
uint8_t ingestion_cnt = 0;
uint8_t rest_cnt = 0;
uint8_t deta_a_cnt = 0;
uint8_t jicha_cnt = 0;
int16_t eighteen_average = 0;
uint16_t sum_eighteen_average = 0;

int16_t x_max_val = -2000;
int16_t x_min_val = 2000;
int16_t y_max_val = -2000;
int16_t y_min_val = 2000;
int16_t z_max_val = -2000;
int16_t z_min_val = 2000;

uint8_t fifo[_FIFO_LEN];
step_t step;

extern uint8_t ErrorIndex;

#if (_PANT_DETECT == 1)
/*------------------------------------------------------------------------------
  Panting / heat-stress spectral rule -- Davison et al. 2020, Agriculture 10, 210.

  Bin mapping.  The FIFO watermark delivers exactly 150 tri-axial samples at
  25 Hz, i.e. 6.000 s, so a DFT bin sits at k*fs/N = k/6 Hz and both bands of
  the paper land on integer bins -- Goertzel is then an exact DFT:

      k =  6..11  ->  1.000 .. 1.833 Hz     (the paper's 1-2 Hz band)
      k = 12..17  ->  2.000 .. 2.833 Hz     (the paper's 2-3 Hz band)

  All twelve are evaluated, but k=11 and k=12 are excluded from the sums and
  kept as a guard band: the Hann main lobe is three bins wide and the two
  bands are adjacent, so without the guard a tone at 1.833 Hz spills a quarter
  of its power into the bottom bin of the opposite band, where it stands alone
  and makes F(2-3) look peakier than F(1-2).  Simulation showed that inverts
  the decision between 1.78 and 2.10 Hz.  The remaining bands hold five bins
  each, which keeps the division-free comparison below symmetric.
------------------------------------------------------------------------------*/
#define _PANT_NBIN        12u                      /* Goertzel bins evaluated   */
#define _PANT_BAND_LO_BEG  0u                      /* k= 6 .. 10 (index 0..4)   */
#define _PANT_BAND_LO_END  5u
#define _PANT_BAND_HI_BEG  7u                      /* k=13 .. 17 (index 7..11)  */
#define _PANT_BAND_HI_END 12u
#define _PANT_BAND_BINS    5u                      /* bins per band, both equal */
#define _PANT_HANN_SHIFT  11                       /* see note in PantWindowAdd */

/* 2*cos(2*pi*k/150) in Q14, k = 6..17.  cos is taken over 14.4..40.8 degrees so
   the coefficient stays inside 1.514..1.937 and the Q14 form fits int16.       */
static const int16_t pant_coef_q14[_PANT_NBIN] =
{
    31739, 31369, 30945, 30467, 29935, 29351,
    28715, 28029, 27293, 26510, 25680, 24805,
};

/* Periodic Hann, round(32767 * 0.5 * (1 - cos(2*pi*n/150))).  Periodic (n/150)
   rather than symmetric (n/149) is what makes the window's DFT vanish for all
   k >= 2, so the residue of the truncated mean cannot leak into the bands.     */
static const uint16_t pant_hann_q15[150] =
{
    0, 14, 57, 129, 229, 358, 515, 699, 911, 1151,
    1416, 1709, 2027, 2370, 2737, 3129, 3544, 3981, 4440, 4921,
    5421, 5940, 6478, 7033, 7605, 8192, 8793, 9408, 10035, 10673,
    11321, 11978, 12642, 13314, 13990, 14671, 15355, 16040, 16727, 17412,
    18096, 18777, 19453, 20125, 20789, 21446, 22094, 22732, 23359, 23974,
    24575, 25162, 25734, 26289, 26827, 27346, 27846, 28327, 28786, 29223,
    29638, 30030, 30397, 30740, 31058, 31351, 31616, 31856, 32068, 32252,
    32409, 32538, 32638, 32710, 32753, 32767, 32753, 32710, 32638, 32538,
    32409, 32252, 32068, 31856, 31616, 31351, 31058, 30740, 30397, 30030,
    29638, 29223, 28786, 28327, 27846, 27346, 26827, 26289, 25734, 25162,
    24575, 23974, 23359, 22732, 22094, 21446, 20789, 20125, 19453, 18777,
    18096, 17412, 16727, 16040, 15355, 14671, 13990, 13314, 12642, 11978,
    11321, 10673, 10035, 9408, 8793, 8192, 7605, 7033, 6478, 5940,
    5421, 4921, 4440, 3981, 3544, 3129, 2737, 2370, 2027, 1709,
    1416, 1151, 911, 699, 515, 358, 229, 129, 57, 14,
};

uint8_t panting_detected;

static uint64_t pant_pow[_PANT_NBIN];   /* accumulated bin power over the window */
static uint8_t  pant_seg_count;         /* segments accepted into this window     */
static uint32_t pant_last_seg_tick;     /* HAL tick of the last accepted segment  */

/*******************************************************************
  @brief Multiply by a Q14 coefficient without a 64-bit multiply.
         Identical bit pattern to (int32_t)(((int64_t)c * s) >> 14):
         s splits exactly as (s >> 14) * 2^14 + (s & 0x3FFF) in two's
         complement because the arithmetic shift floors and the mask
         is non-negative.  Verified against the 64-bit form over the
         whole reachable state range.
*******************************************************************/
static inline int32_t PantCoefMul(int32_t c, int32_t s)
{
    return c * (s >> 14) + ((c * (s & 0x3FFF)) >> 14);
}

/*******************************************************************
  @brief floor(sqrt(n)).  Restoring bit search, no divide, no table.
         Result fits 16 bits for any n, 16 iterations.
*******************************************************************/
static uint32_t PantIsqrt(uint32_t n)
{
    uint32_t c = 0x8000u;
    uint32_t g = 0x8000u;

    for (;;)
    {
        if (g * g > n)
            g ^= c;

        c >>= 1;

        if (c == 0u)
            return g;

        g |= c;
    }
}

/*******************************************************************
  @brief Right-shift a 64-bit value into 32 bits using 32-bit shifts
         only, so that __aeabi_llsr is not linked in.
*******************************************************************/
static uint32_t PantShr64(uint64_t v, uint8_t sh)
{
    uint32_t lo = (uint32_t)v;
    uint32_t hi = (uint32_t)(v >> 32);

    if (sh == 0u)
        return lo;

    if (sh < 32u)
        return (lo >> sh) | (hi << (32u - sh));

    return hi >> (sh - 32u);
}

/*******************************************************************
  @brief void PantWindowReset(void)
         Drop a partially accumulated window.
*******************************************************************/
static void PantWindowReset(void)
{
    memset(pant_pow, 0, sizeof(pant_pow));
    pant_seg_count = 0;
}

/*******************************************************************
  @brief void PantWindowAdd(void)
         Fold one 6 s segment of three_axis_info[] into the running
         spectrum.  Must be called before the difference loop, which
         overwrites three_axis_info[] in place.  The caller guarantees
         the segment is axis-aligned, so all 150 triplets are usable
         and N stays exactly 150, keeping the bins on exact DFT points.
*******************************************************************/
static void PantWindowAdd(void)
{
    uint32_t sum = 0;
    int32_t  mean;
    int32_t  s1[_PANT_NBIN];
    int32_t  s2[_PANT_NBIN];
    uint16_t n;
    uint8_t  j;

    /* pass 1: mean of the vector magnitude, the paper's E = sqrt(dx^2+dy^2+dz^2) */
    for (n = 0; n < 150u; n++)
    {
        int32_t x = three_axis_info[n].x;
        int32_t y = three_axis_info[n].y;
        int32_t z = three_axis_info[n].z;
        sum += PantIsqrt((uint32_t)(x * x + y * y + z * z));
    }

    mean = (int32_t)((sum + 75u) / 150u);

    memset(s1, 0, sizeof(s1));
    memset(s2, 0, sizeof(s2));

    /* pass 2: remove the mean, apply Hann, run all bins in one sweep.
       The shift is 11 rather than 15 on purpose: a typical panting AC
       amplitude is only 5-30 counts, and >>15 would leave 2-15 LSB, with
       quantisation noise comparable to the signal.  The state bound is
       computed with >>11 and still leaves int32 two orders of margin. */
    for (n = 0; n < 150u; n++)
    {
        int32_t x = three_axis_info[n].x;
        int32_t y = three_axis_info[n].y;
        int32_t z = three_axis_info[n].z;
        int32_t mag = (int32_t)PantIsqrt((uint32_t)(x * x + y * y + z * z));
        int32_t xw = ((mag - mean) * (int32_t)pant_hann_q15[n]) >> _PANT_HANN_SHIFT;

        for (j = 0; j < _PANT_NBIN; j++)
        {
            int32_t prev = s1[j];
            int32_t s0 = PantCoefMul(pant_coef_q14[j], prev) - s2[j] + xw;
            s2[j] = prev;
            s1[j] = s0;
        }
    }

    /* |X_k|^2 = s1^2 + s2^2 - coeff*s1*s2.  The coefficient is folded into s1
       first: the literal form (int64)s1*s2*coeff overflows int64 by about 1%. */
    for (j = 0; j < _PANT_NBIN; j++)
    {
        int32_t t = PantCoefMul(pant_coef_q14[j], s1[j]);
        int64_t p = (int64_t)s1[j] * s1[j] + (int64_t)s2[j] * s2[j] - (int64_t)t * s2[j];

        if (p < 0)
            p = 0;                      /* coefficient quantisation can undershoot */

        pant_pow[j] += (uint64_t)p;
    }

    pant_seg_count++;
}

/*******************************************************************
  @brief void PantWindowDecide(void)
         Evaluate the paper's rule once the window is complete.

         The paper normalises a peak *amplitude* to the band mean, so
         the accumulated powers are square-rooted first; comparing
         powers directly is not equivalent and can flip the verdict.
         Because the test is homogeneous of degree two in amplitude, a
         common scale factor cancels and the normalising shift never
         has to be undone.
*******************************************************************/
static void PantWindowDecide(void)
{
    uint64_t mx = 0;
    uint32_t amp[_PANT_NBIN];
    uint32_t s12 = 0, s23 = 0, m12 = 0, m23 = 0;
    uint32_t hi;
    uint8_t  sh = 0;
    uint8_t  j;

    for (j = 0; j < _PANT_NBIN; j++)
        if (pant_pow[j] > mx)
            mx = pant_pow[j];

    /* smallest even shift bringing the largest bin under 2^32 (ARMv6-M has no CLZ) */
    hi = (uint32_t)(mx >> 32);

    while (hi != 0u)
    {
        hi >>= 1;
        sh += 2u;
    }

    for (j = 0; j < _PANT_NBIN; j++)
        amp[j] = PantIsqrt(PantShr64(pant_pow[j], sh));

    for (j = _PANT_BAND_LO_BEG; j < _PANT_BAND_LO_END; j++)
    {
        s12 += amp[j];

        if (amp[j] > m12)
            m12 = amp[j];
    }

    for (j = _PANT_BAND_HI_BEG; j < _PANT_BAND_HI_END; j++)
    {
        s23 += amp[j];

        if (amp[j] > m23)
            m23 = amp[j];
    }

    if (s12 == 0u || s23 == 0u)
    {
        panting_detected = 0u;          /* a zero band would pass the rule vacuously */
    }
    else
    {
        /* F(1-2) > F(2-3), cleared of its division.  Both bands hold the same
           number of bins, so the bin count cancels; the identity also holds if
           the band mean is taken to exclude its own peak.
           The gate F(1-2) >= _PANT_GATE_NUM/_PANT_GATE_DEN is what keeps a
           still animal out: on noise alone F(1-2) and F(2-3) are identically
           distributed and the bare rule is a coin toss, flagging panting on
           about half of all windows. */
        panting_detected = (((uint64_t)m12 * s23 > (uint64_t)m23 * s12)
                            && ((uint64_t)(_PANT_BAND_BINS * _PANT_GATE_DEN) * m12
                                >= (uint64_t)_PANT_GATE_NUM * s12)) ? 1u : 0u;
    }

    rfid_printf("pant m12=%lu s12=%lu m23=%lu s23=%lu sh=%d -> %d\n",
                (unsigned long)m12, (unsigned long)s12,
                (unsigned long)m23, (unsigned long)s23, sh, panting_detected);

    PantWindowReset();
}
#endif /* _PANT_DETECT */

/*******************************************************************
  @brief unsigned char ADXL362RegisterRead(unsigned char Address)
         Read a register value from ADXL362
  @param
         unsigned char Address:       Register address
  @return
         unsigned int  ReceiveValue:  read register value from ADXL362
*******************************************************************/
unsigned char ADXL362RegisterRead(unsigned char Address)
{
    unsigned char SendTemp[2];
    unsigned char ReceiveTemp[2];
    unsigned char ReceiveValue;

    ADXL362_CSN_LOW();              		//CS down
    SendTemp[0] = 0x0B;                 //0x0B: read register command
    SendTemp[1] = Address;              //address byte
    SpiFunction(SPI2, SendTemp, ReceiveTemp, 2, 1);
    ReceiveValue = ReceiveTemp[0];
    ADXL362_CSN_HIGH();
    return(ReceiveValue);               //CS up
}

/*******************************************************************
  @brief void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue)
         send SPI command to ADXL362
  @param
         unsigned char Address:       Register address
         unsigned char SendValue:     Value written to ADXL362 register
  @return
         none
*******************************************************************/
void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue)
{
    unsigned char SendTemp[3];
    unsigned char ReceiveTemp[3];

    ADXL362_CSN_LOW();              			//CS down
    SendTemp[0] = 0x0A;                 //0x0A: write register
    SendTemp[1] = Address;              //address byte
    SendTemp[2] = SendValue;

    SpiFunction(SPI2, SendTemp, ReceiveTemp, 3, 0);
    ADXL362_CSN_HIGH();              		//CS up
}

/*******************************************************************
  @brief void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte read from ADXL362
  @param
         unsigned char Address:           Register address
         unsigned char NumberofRegisters: Register numbers to be read
         unsigned char *RegisterData:     Buffer save the read value
  @return
         none
*******************************************************************/
void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
    unsigned char SendTemp[2];

    ADXL362_CSN_LOW();         			//CS down
    SendTemp[0] = 0x0B;            	//0x0B: read register
    SendTemp[1] = Address;         	//address byte
    SpiFunction(SPI2, SendTemp, RegisterData, 2, NumberofRegisters);
    ADXL362_CSN_HIGH();        			//CS up
}

/*******************************************************************
  @brief void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte write to ADXL362
  @param
         unsigned char Address:           Register address
         unsigned char NumberofRegisters: Register numbers to be written
         unsigned char *RegisterData:     Buffer save the written value
  @return
         none
*******************************************************************/
void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
    unsigned char SendTemp[256];
    unsigned char ReceiveTemp[2];
    unsigned char RegisterIndex;

    ADXL362_CSN_LOW();										//CS down
    SendTemp[0] = 0x0A;                 //0x0A: write register
    SendTemp[1] = Address;              //address byte

    for (RegisterIndex = 0; RegisterIndex < NumberofRegisters; RegisterIndex++)
    {
        SendTemp[2 + RegisterIndex] = *(RegisterData + RegisterIndex);
    }

    SpiFunction(SPI2, SendTemp, ReceiveTemp, (2 + NumberofRegisters), 0);
    ADXL362_CSN_HIGH();              		//CS up
}

/*******************************************************************
  @brief void ADXL362FifoRead(unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte read from ADXL362 FIFO
  @param
         unsigned char NumberofRegisters: Register numbers to be read
         unsigned char *RegisterData:     Buffer save the read value
  @return
         none
*******************************************************************/
void ADXL362FifoRead(unsigned int NumberofRegisters, unsigned char *RegisterData)
{
    unsigned char SendTemp[1];
    ADXL362_CSN_LOW();         			//CS down
    SendTemp[0] = 0x0D;            	//0x0D: read register
    SpiFunction(SPI2, SendTemp, RegisterData, 1, NumberofRegisters);
    ADXL362_CSN_HIGH();        			//CS up
}

/*******************************************************************
  @brief void ADXL362FifoEntries(void)
         read ADXL362 FIFO ENTRIES REGISTERS
  @param
         none
  @return
         none
*******************************************************************/
uint16_t ADXL362FifoEntries(void)
{
    uint16_t ReadValueTemp;
    ReadValueTemp = (0x00ff & ADXL362RegisterRead(XL362_FIFO_ENTRIES_L)) + (0x0300 & ADXL362RegisterRead(XL362_FIFO_ENTRIES_H));
    return(ReadValueTemp);
}

/*******************************************************************
  @brief void ADXL362_Init(void)
         initial and configure ADXL362
  @param
				 none
  @return
         none
*******************************************************************/
void ADXL362_Init(void)
{
    unsigned int ReadValueTemp;
    memset(&step, 0, sizeof(step));
    memset(&three_axis_info, 0, sizeof(three_axis_info));
    memset(memory_array, 0, sizeof(memory_array));

    ADXL362RegisterWrite(XL362_SOFT_RESET, 0x52);   						// software reset
    HAL_Delay(1000);

    #if (_Original_Data_Algorithm == 0)

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, 0x64);						//set active threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_L);
    rfid_printf("|*-set THRESH_ACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_ACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_H);
    rfid_printf("|*-set THRESH_ACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06);						//set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_L, 0x64);					//set inactive threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_L);
    rfid_printf("|*-set THRESH_INACT_L register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_H);
    rfid_printf("|*-set THRESH_INACT_H register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x06);						//set inactive time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_H);
    rfid_printf("|*-set TIME_INACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x3F);						//configure loop mode,enable active and inactive
    ReadValueTemp = ADXL362RegisterRead(XL362_ACT_INACT_CTL);
    rfid_printf("|*-set ACT_INACT_CTL register = 0x%02x--*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x3F)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP1, 0x10);									//configure act map INT1
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP1);
    rfid_printf("|*-set INTMAP1 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP2, 0x04);									//configure fifo_watermark map INT2
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP2);
    rfid_printf("|*-set INTMAP2 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x04)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x0A);						//Above Half, select fifo Steam Mode, not store Temperature Data to FIFO
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_CONTROL);
    rfid_printf("|*-set FIFO_CONTROL register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x0A)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0xC2);						//select fifo sample number//0x1C2 = 450
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_SAMPLES);
    rfid_printf("|*-set FIFO_SAMPLES register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0xC2)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x51);             	//select 4g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x51)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02);              	//select measurement mode
    ReadValueTemp = ADXL362RegisterRead(XL362_POWER_CTL);
    rfid_printf("|*-set POWER_CTL register = 0x%02x------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x02)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    #else

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");
    ADXL362RegisterWrite(XL362_THRESH_ACT_L, 0x64);						//set active threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_L);
    rfid_printf("|*-set THRESH_ACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_ACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_ACT_H);
    rfid_printf("|*-set THRESH_ACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_ACT, 0x06);						//set active time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_ACT);
    rfid_printf("|*-set TIME_ACT register = 0x%02x-------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_L, 0x64);					//set inactive threshold equip 200mg
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_L);
    rfid_printf("|*-set THRESH_INACT_L register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x64)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_THRESH_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_THRESH_INACT_H);
    rfid_printf("|*-set THRESH_INACT_H register = 0x%02x-*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_L, 0x06);						//set inactive time equip 6/25s
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_L);
    rfid_printf("|*-set TIME_INACT_L register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x06)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_TIME_INACT_H, 0x00);
    ReadValueTemp = ADXL362RegisterRead(XL362_TIME_INACT_H);
    rfid_printf("|*-set TIME_INACT_H register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x00)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_ACT_INACT_CTL, 0x3F);						//configure loop mode,enable active and inactive
    ReadValueTemp = ADXL362RegisterRead(XL362_ACT_INACT_CTL);
    rfid_printf("|*-set ACT_INACT_CTL register = 0x%02x--*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x3F)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP1, 0x10);									//configure act map INT1
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP1);
    rfid_printf("|*-set INTMAP1 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x10)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_INTMAP2, 0x04);									//configure fifo_watermark map INT2
    ReadValueTemp = ADXL362RegisterRead(XL362_INTMAP2);
    rfid_printf("|*-set INTMAP2 register = 0x%02x--------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x04)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x0A);						//Above Half, select fifo Steam Mode, not store Temperature Data to FIFO
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_CONTROL);
    rfid_printf("|*-set FIFO_CONTROL register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x0A)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FIFO_SAMPLES, 0xC2);						//select fifo sample number//0x1C2 = 450
    ReadValueTemp = ADXL362RegisterRead(XL362_FIFO_SAMPLES);
    rfid_printf("|*-set FIFO_SAMPLES register = 0x%02x---*|\n", ReadValueTemp);

    if(ReadValueTemp != 0xC2)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    ADXL362RegisterWrite(XL362_FILTER_CTL, 0x51);             	//select 4g range,ODR:25Hz
    ReadValueTemp = ADXL362RegisterRead(XL362_FILTER_CTL);
    rfid_printf("|*-set FILTER_CTL register = 0x%02x-----*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x51)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL, 0x02);              	//select measurement mode
    ReadValueTemp = ADXL362RegisterRead(XL362_POWER_CTL);
    rfid_printf("|*-set POWER_CTL register = 0x%02x------*|\n", ReadValueTemp);

    if(ReadValueTemp != 0x02)
    {
        ErrorIndex = 0x03;
        Error_Handler();
    }

    #endif

    rfid_printf("|**************************************|\n");

    HAL_Delay(200);
}

/*******************************************************************
  @brief void ADXL362FifoProcess(void)
         Multibyte read from ADXL362 FIFO
  @param
				 none
  @return
				 none
*******************************************************************/
void ADXL362FifoProcess(void)
{
    #if (_Original_Data_Algorithm == 0)
    // 1.init acceler array
    memset(three_axis_info, 0, sizeof(three_axis_info));

    // 2.If the fifo data is not aligned. Organize the data
    if ((fifo[1] >> 6 & 0x03) == 0x1)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 4; i++)
        {
            fifo[i] = fifo[i + 4];
        }
    }
    else if ((fifo[1] >> 6 & 0x03) == 0x2)
    {
        for(uint16_t i = 0; i < sizeof(fifo) - 2; i++)
        {
            fifo[i] = fifo[i + 2];
        }
    }

    // 3.To 16-bit complement
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) * 3; i++)
    {
        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x0)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("X[%d] = %hd, %hx ", i/3, xAxis[i/3], xAxis[i/3]);
//            rfid_printf("samples[%d] :%hd,", i / 3, three_axis_info[i / 3].x);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x1)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("Y[%d] = %hd, %hx ", i/3, yAxis[i/3], yAxis[i/3]);
//            rfid_printf("%hd,", three_axis_info[i / 3].y);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x2)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("Z[%d] = %hd, %hx\n", i/3, zAxis[i/3], zAxis[i/3]);
//            rfid_printf("%hd\n", three_axis_info[i / 3].z);
        }
    }

    // 4.Average Filter
    // 5.Dynamic Threshold

    CC1101Send3AxisHandler();

    #else
    // 1.init acceler array

    // 2.To 16-bit complement
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6) * 3; i++)
    {
        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x0)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].x = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("X[%d] = %hd, %hx ", i/3, xAxis[i/3], xAxis[i/3]);
//            rfid_printf("samples[%d] :%hd,", i / 3, three_axis_info[i / 3].x);
        }

        if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x1)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].y = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

//            rfid_printf("samples[%d] :%hd\n", i / 3, three_axis_info[i / 3].y);
        }
        else if ((fifo[2 * i + 1] >> 6 & 0x03) == 0x2)
        {
            if ((fifo[2 * i + 1] & 0x08))
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)) + 0xf000);
            else
                three_axis_info[i / 3].z = (short int)(fifo[2 * i] + (0x0f00 & (fifo[2 * i + 1] << 8)));

            //				rfid_printf("Z[%d] = %hd, %hx\n", i/3, zAxis[i/3], zAxis[i/3]);
//            rfid_printf("%hd\n", three_axis_info[i / 3].z);
        }
    }

    // 3.Average Calc

    #if (_PANT_DETECT == 1)
    /* Steps 1-7 of the paper's heat-stress rule.  This has to run here: the
       difference loop below overwrites three_axis_info[] in place, and the
       spectrum needs the raw magnitudes.  Nothing above is modified, so the
       rest/ingestion/movement/climb classifier is bit-for-bit unchanged. */
    {
        uint32_t pant_now = HAL_GetTick();

        if ((pant_seg_count != 0u) && ((pant_now - pant_last_seg_tick) > _PANT_STALE_MS))
            PantWindowReset();      /* a transmit gap must not weld two behaviours into one window */

        if (((fifo[1] >> 6) & 0x03u) != 0x00u)
        {
            /* The watermark was overrun and the axis tags have slipped, so
               three_axis_info[] now mixes axes from adjacent samples.  The
               legacy classifier tolerates that; a vector magnitude does not.
               Drop the whole window rather than spectrally analyse a splice. */
            PantWindowReset();
        }
        else
        {
            PantWindowAdd();
            pant_last_seg_tick = pant_now;

            if (pant_seg_count >= _PANT_WIN_SEG)
                PantWindowDecide();
        }
    }
    #endif

    // 4.Difference Derivation And Threshold Judge
    for(uint16_t i = 0; i < (_FIFO_SAMPLES_LEN / 6); i++)
    {
        diff_three_axis_info[1] = diff_three_axis_info[0];
        diff_three_axis_info[0] = three_axis_info[i];

        if (three_axis_info[i].x > x_max_val)
            x_max_val = three_axis_info[i].x;

        if (three_axis_info[i].x < x_min_val)
            x_min_val = three_axis_info[i].x;

        if (three_axis_info[i].y > y_max_val)
            y_max_val = three_axis_info[i].y;

        if (three_axis_info[i].y < y_min_val)
            y_min_val = three_axis_info[i].y;

        if (three_axis_info[i].z > z_max_val)
            z_max_val = three_axis_info[i].z;

        if (three_axis_info[i].z < z_min_val)
            z_min_val = three_axis_info[i].z;

        three_axis_average_info.x += three_axis_info[i].x;

        three_axis_info[i].x = diff_three_axis_info[0].x - diff_three_axis_info[1].x;
        three_axis_info[i].y = diff_three_axis_info[0].y - diff_three_axis_info[1].y;
        three_axis_info[i].z = diff_three_axis_info[0].z - diff_three_axis_info[1].z;

        sum_info.x += abs(three_axis_info[i].x);
        sum_info.y += abs(three_axis_info[i].y);
        sum_info.z += abs(three_axis_info[i].z);

        rfid_printf("samples[%d] :%hd\n", i, three_axis_info[i].x);

        if (abs(three_axis_info[i].x) <= 10)
            threshold_judge.low ++;
        else if (abs(three_axis_info[i].x) <= 100)
            threshold_judge.normal ++;
        else if (abs(three_axis_info[i].x) <= 200)
            threshold_judge.abovenormal ++;
        else
            threshold_judge.high ++;

        if (((i + 1) / 25) != 0 && ((i + 1) % 25) == 0)
        {
            rfid_printf("threshold_judge.low = %d\n", threshold_judge.low);
            rfid_printf("threshold_judge.normal = %d\n", threshold_judge.normal);
            rfid_printf("threshold_judge.abovenormal = %d\n", threshold_judge.abovenormal);
            rfid_printf("threshold_judge.high = %d\n", threshold_judge.high);

            three_axis_average_info.x = three_axis_average_info.x / 25;
            rfid_printf("average_info.y = %d\n", three_axis_average_info.x);
            rfid_printf("sum_info.y = %d\n", sum_info.x);

            if (threshold_judge.low >= 24)
                action = 1;             // rest
            else if ((threshold_judge.normal + threshold_judge.abovenormal) > 11 && threshold_judge.high == 0 &&
                     three_axis_average_info.x >= 200)
                action = 2;             // ingestion
            else if (three_axis_average_info.x > -200 && three_axis_average_info.x < 100 &&
                     (sum_info.x + sum_info.y + sum_info.z) / 3 > 400 &&
                     (((x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val)) / 3) > 150)
                action = 3;             // movement
            else if (threshold_judge.high > 0 && three_axis_average_info.x <= -200)
                action = 4;             // climb
            else
                action = 6;             //other

            rfid_printf("action = %d\n", action);
            rfid_printf("action_classify.rest = %d\n", action_classify.rest);
            rfid_printf("action_classify.ingestion = %d\n", action_classify.ingestion);
            rfid_printf("action_classify.movement = %d\n", action_classify.movement);
            rfid_printf("action_classify.climb = %d\n", action_classify.climb);
            rfid_printf("action_classify.other = %d\n", action_classify.other);

            // circular storage
            memory_array[memory_index][0] = action;
            memory_array[memory_index][1] = three_axis_average_info.x;
            memory_array[memory_index][2] = (sum_info.x + sum_info.y + sum_info.z) / 3;
            memory_array[memory_index][3] = ((x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val)) / 3;

            rfid_printf("memory_array1:\n");

            // print memory_array
            for (uint8_t i = 0; i < _MEM_ROWS; i++)
            {
                for (uint8_t j = 0; j < _MEM_COLS; j++)
                {
                    rfid_printf("%d ", memory_array[i][j]);
                }

                rfid_printf("\n");
            }

            // redundant movement among rest
            movement_cnt = 0;
            rest_cnt1 = 0;
            rest_cnt2 = 0;

            if (memory_index >= 9)
                mid_index = memory_index - 9;
            else
                mid_index = memory_index + 9;

            rfid_printf("memory_index = %d\n", memory_index);
            rfid_printf("mid_index = %d\n", mid_index);

            if(memory_array[mid_index][0] == 3)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    if(memory_array[i][0] == 3)
                        movement_cnt += 1;

                rfid_printf("movement_cnt = %d\n", movement_cnt);

                if (movement_cnt == 1)
                {
                    if(mid_index < memory_index)
                    {
                        for (uint8_t i = mid_index; i < memory_index; i++)
                            if(memory_array[i][0] == 1)
                                rest_cnt1 += 1;

                        for (uint8_t i = 0; i < mid_index; i++)
                            if(memory_array[i][0] == 1)
                                rest_cnt2 += 1;

                        for (uint8_t i = memory_index; i < _MEM_ROWS; i++)
                            if(memory_array[i][0] == 1)
                                rest_cnt2 += 1;
                    }
                    else if (memory_index < mid_index)
                    {
                        for (uint8_t i = memory_index; i < mid_index; i++)
                            if(memory_array[i][0] == 1)
                                rest_cnt2 += 1;

                        for (uint8_t i = 0; i < memory_index; i++)
                            if(memory_array[i][0] == 1)
                                rest_cnt1 += 1;

                        for (uint8_t i = mid_index; i < _MEM_ROWS; i++)
                            if(memory_array[i][0] == 1)
                                rest_cnt1 += 1;
                    }

                    rfid_printf("rest_cnt1 = %d\n", rest_cnt1);
                    rfid_printf("rest_cnt2 = %d\n", rest_cnt2);

                    if (rest_cnt1 >= 4 && rest_cnt2 >= 4)
                        memory_array[mid_index][0] = 1;
                }
            }

            // redundant climb if movement less than 4
            // eliminate redundant climb
            movement_cnt = 0;
            climb_cnt = 0;

            if(memory_array[memory_index][0] == 4)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                {
                    if(memory_array[i][0] == 3)
                        movement_cnt += 1;

                    if(memory_array[i][0] == 4)
                        climb_cnt += 1;
                }

                if (movement_cnt < 4)
                    memory_array[memory_index][0] = 6;
                else if(climb_cnt >= 2)
                    memory_array[memory_index][0] = 3;
            }

            // index update
            if (memory_index >= 17)
                memory_index = 0;
            else
                memory_index += 1;

            // eliminate redundant climb
//            for (uint8_t i = 0; i < _MEM_ROWS; i++)
//                if(memory_array[i][0] == 4)
//                {
//                    for (uint8_t j = (i + 1); j < _MEM_ROWS; j++)
//                    {
//                        if (memory_array[j][0] == 4)
//                            memory_array[j][0] = 3;
//                    }

//                    rfid_printf("memory_array2:\n");

//                    // print memory_array
//                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
//                    {
//                        for (uint8_t j = 0; j < _MEM_COLS; j++)
//                        {
//                            rfid_printf("%d ", memory_array[i][j]);
//                        }

//                        rfid_printf("\n");
//                    }
//                }

            // if ingestion_cnt >= 2 ==> 3 to 6
            ingestion_cnt = 0;

            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if(memory_array[i][0] == 2)
                    ingestion_cnt += 1;

            if (ingestion_cnt >= 2)
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                    if(memory_array[i][0] == 3)
                        memory_array[i][0] = 6;

            // ruminate logic
            rest_cnt = 0;
            deta_a_cnt = 0;
            jicha_cnt = 0;
            eighteen_average = 0;
            sum_eighteen_average = 0;

            for (uint8_t i = 0; i < _MEM_ROWS; i++)
                if(memory_array[i][0] == 1)
                    rest_cnt += 1;

            if (rest_cnt <= 4)
            {
                for (uint8_t i = 0; i < _MEM_ROWS; i++)
                {
                    if(memory_array[i][2] > 130 && memory_array[i][2] < 700)
                        deta_a_cnt += 1;

                    if(memory_array[i][3] < 120)
                        jicha_cnt += 1;
                }

                if (deta_a_cnt >= 14 && jicha_cnt >= 14)
                {
                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        eighteen_average += memory_array[i][1];

                    eighteen_average = eighteen_average / _MEM_ROWS;

                    for (uint8_t i = 0; i < _MEM_ROWS; i++)
                        sum_eighteen_average += abs(eighteen_average - memory_array[i][1]);

                    if (sum_eighteen_average <= 400 && eighteen_average < 150)
                        for (uint8_t i = 0; i < _MEM_ROWS; i++)
                            memory_array[i][0] = 5;
                }
            }

            // delay 18s output
            if (memory_index == 17)
                memory_index_o = 0;
            else
                memory_index_o = memory_index + 1;

            action_classify_array[((i + 1) / 25) - 1] = memory_array[memory_index_o][0];

            #if (_PANT_DETECT == 1)

            /* Step 9 of the paper: where rumination has been identified, re-classify
               it as heat stress if F(1-2) > F(2-3).  The 18 s spectral window covers
               exactly the 18 s held in memory_array, and the ruminate rule stamps all
               18 rows at once, so the two line up without a delay correction. */
            if (memory_array[memory_index_o][0] == 5 && panting_detected != 0u)
            {
                action_classify_array[((i + 1) / 25) - 1] = 7;
                action_classify.panting ++;
                #if (_PANTING_REPLACES_RUMINATE == 0)
                action_classify.ruminate ++;    /* keep the RF frame bit-identical */
                #endif
            }
            else
            #endif
            if (memory_array[memory_index_o][0] == 1) // rest
                action_classify.rest ++;
            else if (memory_array[memory_index_o][0] == 2) // ingestion
                action_classify.ingestion ++;
            else if (memory_array[memory_index_o][0] == 3) // movement
                action_classify.movement ++;
            else if (memory_array[memory_index_o][0] == 4) // climb
                action_classify.climb ++;
            else if (memory_array[memory_index_o][0] == 5) // ruminate
                action_classify.ruminate ++;
            else //other
                action_classify.other ++;

            memset(&threshold_judge, 0, sizeof(threshold_judge));
            memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
            memset(&sum_info, 0, sizeof(sum_info));
            x_max_val = -2000;
            x_min_val = 2000;
            y_max_val = -2000;
            y_min_val = 2000;
            z_max_val = -2000;
            z_min_val = 2000;
        }
    }

    for(uint8_t i = 0; i < sizeof(action_classify_array); i++)
    {
        rfid_printf("action_classify_array[%d] = %d\n", i, action_classify_array[i]);
    }

//    CC1101Send3AxisHandler();

//    ADXL362RegisterRead(XL362_STATUS);
//    rfid_printf("XL362_STATUS: %x\n", ADXL362RegisterRead(XL362_STATUS));

    #endif
}

