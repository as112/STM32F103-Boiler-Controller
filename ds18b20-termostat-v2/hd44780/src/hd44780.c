//------------------------------------------------------------------------------
// This is Open source software. You can place this code on your site, but don't
// forget a link to my YouTube-channel: https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// ��� ����������� ����������� ���������������� ��������. �� ������ ���������
// ��� �� ����� �����, �� �� �������� ������� ������ �� ��� YouTube-����� 
// "����������� � ���������" https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// �����: �������� ������ / Nadyrshin Ruslan
//------------------------------------------------------------------------------

#include "hd44780.h"


#if (HD44780_pcf8574_mode)
  #include "pcf8574.h"
  uint8_t pcf8574_Value = 0;
#endif

  
// ������� ��� ���������� ��������
#if (HD44780_pcf8574_mode)
  // ������� ��� ���������� ����������� ��� ������ ����� I2C-������� �� pcf8574 
  #define HD44780_RS_HIGH()             {pcf8574_Value |= HD44780_pcf8574_RS_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_RS_LOW()              {pcf8574_Value &= ~HD44780_pcf8574_RS_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_RW_HIGH()             {pcf8574_Value |= HD44780_pcf8574_RW_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_RW_LOW()              {pcf8574_Value &= ~HD44780_pcf8574_RW_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_E_HIGH()              {pcf8574_Value |= HD44780_pcf8574_E_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_E_LOW()               {pcf8574_Value &= ~HD44780_pcf8574_E_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_BL_HIGH()             {pcf8574_Value |= HD44780_pcf8574_BL_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_BL_LOW()              {pcf8574_Value &= ~HD44780_pcf8574_BL_mask; pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_SetDATA_4bit(val)     {pcf8574_Value &= ~(0xF << HD44780_Data_Shift); pcf8574_Value |= ((val & 0xF) << HD44780_Data_Shift); pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_GetDATA_4bit()        ((pcf8574_read(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr) >> HD44780_Data_Shift) & 0xF)
  #define HD44780_SetDATA_PinMode_In(mask)      {pcf8574_Value |= (mask); pcf8574_write(HD44780_pcf8574_i2c_periph, HD44780_pcf8574_addr, pcf8574_Value);}
  #define HD44780_SetDATA_PinMode_Out(mask)     {}
#else
  // ������� ��� ���������� ����������� ��� ������ �������� ����� GPIO ���������������� 
  #define HD44780_RS_HIGH()             GPIO_SetBits(HD44780_RS_Port, HD44780_RS_Pin)
  #define HD44780_RS_LOW()              GPIO_ResetBits(HD44780_RS_Port, HD44780_RS_Pin)
  #define HD44780_RW_HIGH()             GPIO_SetBits(HD44780_RW_Port, HD44780_RW_Pin)
  #define HD44780_RW_LOW()              GPIO_ResetBits(HD44780_RW_Port, HD44780_RW_Pin)
  #define HD44780_E_HIGH()              GPIO_SetBits(HD44780_E_Port, HD44780_E_Pin)
  #define HD44780_E_LOW()               GPIO_ResetBits(HD44780_E_Port, HD44780_E_Pin)
  #define HD44780_BL_HIGH()             GPIO_SetBits(HD44780_BL_Port, HD44780_BL_Pin)
  #define HD44780_BL_LOW()              GPIO_ResetBits(HD44780_BL_Port, HD44780_BL_Pin)
  #define HD44780_SetDATA_4bit(val)     {GPIO_ResetBits(HD44780_Data_Port, 0xF << HD44780_Data_Shift); GPIO_SetBits(HD44780_Data_Port, val << HD44780_Data_Shift);}
  #define HD44780_SetDATA_8bit(val)     {GPIO_ResetBits(HD44780_Data_Port, 0xFF); GPIO_SetBits(HD44780_Data_Port, val);}
  #define HD44780_GetDATA_4bit()        ((GPIO_ReadInputData(HD44780_Data_Port) >> HD44780_Data_Shift) & 0xF)
  #define HD44780_GetDATA_8bit()        (GPIO_ReadInputData(HD44780_Data_Port) && 0xFF)
  #define HD44780_SetDATA_PinMode_In(mask)      hd44780_SetGPIOmode_In(HD44780_Data_Port, mask)
  #define HD44780_SetDATA_PinMode_Out(mask)     hd44780_SetGPIOmode_Out(HD44780_Data_Port, mask)
#endif

uint8_t LastRow = 0;                                               // ��������� ������������� ������
char hd44780_StrBuff[HD44780_COLS * HD44780_ROWS + 8];          // ����� ������ ��� ������



//==============================================================================
// ��������� ��������� ����� ���������������� (�� �����) ��� ������ 
//==============================================================================
void hd44780_SetGPIOmode_In(GPIO_TypeDef* GPIOx, uint16_t Mask)
{
  GPIO_InitTypeDef InitStruct;
  InitStruct.GPIO_Pin = Mask;
  InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  InitStruct.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOx, &InitStruct);
}
//==============================================================================


//==============================================================================
// ��������� ��������� ����� ���������������� (�� �����) ��� ������� 
//==============================================================================
void hd44780_SetGPIOmode_Out(GPIO_TypeDef* GPIOx, uint16_t Mask)
{
  GPIO_InitTypeDef InitStruct;
  InitStruct.GPIO_Pin = Mask;
  InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOx, &InitStruct);
}
//==============================================================================


//==============================================================================
// ������� ������ ����� �� hd44780
// - IsCmd - ������ ���������� ����� (����� ����� ������)
//==============================================================================
uint8_t hd44780_read(int8_t IsCmd)
{
  unsigned char Data = 0;
  
  if (IsCmd)
  {
    HD44780_RS_LOW();
  }
  else
  {
    HD44780_RS_HIGH();
  }
  
  HD44780_RW_HIGH();
  
#if HD44780_4bitMode
  // ��������� ������� �������
  HD44780_E_HIGH();
  delay_us(HD44780_ShortDelayUs);
  Data = HD44780_GetDATA_4bit() << 4;
  HD44780_E_LOW();

  delay_us(HD44780_ShortDelayUs);

  // ��������� ������� �������
  HD44780_E_HIGH();
  delay_us(HD44780_ShortDelayUs);
  Data |= HD44780_GetDATA_4bit();
  HD44780_E_LOW();
#else
  HD44780_E_HIGH();
  delay_us(HD44780_ShortDelayMks);
  // ��������� ���� � ���� ������
  Data = HD44780_GetDATA_8bit();
  HD44780_E_LOW();
#endif
  
  HD44780_RW_LOW();

  return Data;
}
//==============================================================================


//==============================================================================
// ��������� ������ ����� �� hd44780
// - IsCmd - ������ ���������� ����� (����� ����� ������)
//==============================================================================
void hd44780_write(uint8_t Data, int8_t IsCmd)
{
  if (IsCmd)
  {
    HD44780_RS_LOW();
  }
  else
  {
    HD44780_RS_HIGH();
  }
  
#if HD44780_4bitMode
  // ����������� ���� ������ ��� ������
  HD44780_SetDATA_PinMode_Out(0xF << HD44780_Data_Shift);
  
  // ����� ������� �������
  HD44780_SetDATA_4bit(Data >> 4);
  HD44780_E_HIGH();
  delay_us(HD44780_ShortDelayUs);
  HD44780_E_LOW();

  delay_us(HD44780_ShortDelayUs);

  // ����� ������� �������
  HD44780_SetDATA_4bit(Data & 0xF);
  HD44780_E_HIGH();
  delay_us(HD44780_ShortDelayUs);
  HD44780_E_LOW();
  
  // ����������� ���� ������ ��� �����
  HD44780_SetDATA_PinMode_In(0xF << HD44780_Data_Shift);
#else
  // ����������� ���� ������ ��� ������
  HD44780_SetDATA_PinMode_Out(0xFF);
  
  // ����� ����
  HD44780_SetDATA_8bit(Data);
  HD44780_E_HIGH();
  delay_us(HD44780_ShortDelayUs);
  HD44780_E_LOW();
  
  // ����������� ���� ������ ��� �����
  HD44780_SetDATA_PinMode_In(0xFF);
#endif
}
//==============================================================================


//==============================================================================
// ������� �������� ������������ ����������� ������� (���� HD44780_WaitBisyFlag=1)
//==============================================================================
#if (HD44780_WaitBisyFlag)
uint8_t hd44780_waitbisy(uint8_t tick)
{
  while ((hd44780_read(1) & 0x80) && (tick)) 
  {
    tick--;
  }
  
  return (tick) ? 0 : 1;
}
#endif
//==============================================================================


//==============================================================================
// ��������� ������ ������� � ��������� ������������ ����������� �������
//==============================================================================
void hd44780_write_cmd(uint8_t Data)
{
#if (HD44780_WaitBisyFlag)
  hd44780_waitbisy(100);
#endif
  
  hd44780_write(Data, 1);

#if (!HD44780_WaitBisyFlag)
  delay_us(HD44780_BisyDelayUs);
#endif
}
//==============================================================================


//==============================================================================
// ��������� ������ ����� ������ � ��������� ������������ ����������� �������
//==============================================================================
void hd44780_write_data(uint8_t Data)
{
#if (HD44780_WaitBisyFlag)
  hd44780_waitbisy(100);
#endif

  hd44780_write(Data, 0);

#if (!HD44780_WaitBisyFlag)
  delay_us(HD44780_BisyDelayUs);
#endif
}
//==============================================================================


//==============================================================================
// ��������� ������ ������������ GPIO
//==============================================================================
void PortClockStart(GPIO_TypeDef *GPIOx)
{
  if (GPIOx == GPIOA)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  else if (GPIOx == GPIOB)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  else if (GPIOx == GPIOC)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  else if (GPIOx == GPIOD)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  else if (GPIOx == GPIOE)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  else if (GPIOx == GPIOF)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
  else if (GPIOx == GPIOG)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
}
//==============================================================================


//==============================================================================
// ��������� ���������� ���������� ������� (����� I2C-������� ��� ����� ����� GPIO)
//==============================================================================
void hd44780_backlight_set(uint8_t val)
{
  if (val)
  {
    HD44780_BL_HIGH();
  }
  else
  {
    HD44780_BL_LOW();
  }
}
//==============================================================================


//==============================================================================
// ��������� ������������� ��� GPIO/I2C-�������� ��� ������ � hd44780
//==============================================================================
void hd44780_bus_init(void)
{
#if (HD44780_pcf8574_mode)
  pcf8574_bus_init(I2C1);
#else
  // �������� ������������ ��������� ������
  PortClockStart(HD44780_RS_Port);
  PortClockStart(HD44780_RW_Port);
  PortClockStart(HD44780_E_Port);
  PortClockStart(HD44780_BL_Port);
  PortClockStart(HD44780_Data_Port);
  
  // ��� ����, ����������� ����� ����������� ��� ������
  // ������ E
  hd44780_SetGPIOmode_Out(HD44780_E_Port, HD44780_E_Pin);
  HD44780_E_LOW();
  // ������ RS
  hd44780_SetGPIOmode_Out(HD44780_RS_Port, HD44780_RS_Pin);
  HD44780_RS_LOW();

#if (HD44780_BackLightCtrl)
  // ������ ���������� ����������
  hd44780_SetGPIOmode_Out(HD44780_BL_Port, HD44780_BL_Pin);
  HD44780_BL_LOW();
#endif
  
#if (HD44780_WaitBisyFlag)
  // ������ RW
  hd44780_SetGPIOmode_Out(HD44780_RW_Port, HD44780_RW_Pin);
#else
  /*
  hd44780_SetGPIOmode_Out(HD44780_RW_Port, HD44780_RW_Pin);
  GPIO_ResetBits(HD44780_RW_Port, HD44780_RW_Pin);
  */
#endif
  
#endif
}
//==============================================================================


//==============================================================================
// ��������� ������������� ������� (�������� ������������������ ������ � �������)
//==============================================================================
void hd44780_start(void)
{
#if HD44780_4bitMode
  hd44780_write_cmd(0x02);       // ������ �������, ������ ���� ������
  uint8_t Reg = 0x20;
#else
  uint8_t Reg = 0x30;
#endif

#if (HD44780_ROWS > 1)
  Reg |= 0x08;
#endif
  
  hd44780_write_cmd(Reg);       // ������ �������, ������ ���� ������
  delay_ms(10);
  hd44780_write_cmd(0x0C);      // �������� �������
  delay_ms(10);
  hd44780_write_cmd(0x06);      // ������������� ������
  hd44780_clear();              // ������� ������
}
//==============================================================================


//==============================================================================
// ��������� ������������� �������
//==============================================================================
void hd44780_init(void)
{
  hd44780_bus_init();
  delay_ms(100);
  hd44780_start();
}
//==============================================================================


//==============================================================================
// ��������� ��������� ������� �������
//==============================================================================
void hd44780_goto_xy(uint8_t Row, uint8_t Col)
{
  // ��������� ��������� ����� ������ ������
  uint8_t Adr = 0;            
  if (Row & 1)                  // �������� ������ � �.�. hd44780 - ������ ������
    Adr = 0x40;                 // ������ �������� �� ������ ������ hd44780
  if (Row > 1)                  // ��� 4-��������� ������� ��� ������ ������ > 2
    Adr += HD44780_COLS;        // ������ ����� ������ �� ����� ������

  // ��������� ����� � ������
  Adr += Col;
  
  // ����� ���������� ����� DRAM � hd44780
  hd44780_write_cmd(Adr | 0x80);        // ������� ��������� ������ � DRAM
  
  LastRow = Row;
}
//==============================================================================


//==============================================================================
// ��������� ������� �������
//==============================================================================
void hd44780_clear(void)
{
  hd44780_write_cmd(0x01);      // ������� ������

#if (!HD44780_WaitBisyFlag)
  delay_ms(2);
#endif
}
//==============================================================================


//==============================================================================
// ��������� ������ � ������� ������� ����
//==============================================================================
void hd44780_write_buff(char *pBuff, uint8_t Len)
{
  while (Len--)
  {
    hd44780_write_data(*(pBuff++));
  }
}
//==============================================================================


//==============================================================================
// ��������� �������� ANSI-������ � ������� ������� �������
//==============================================================================
void hd44780_puts(char *str)
{
  char i;
  
  while (*str != '\0')
  {
    switch (*str)
    {
    case '\n':  // ������� �� ����� ������
      LastRow++;
      hd44780_goto_xy(LastRow, 0);
      break;
    case '\r':
      hd44780_goto_xy(LastRow, 0);
      break;
    case '\t':
      for (i = 0; i < 4; i++)
        hd44780_write_data(0x20);
      break;
    default:
        hd44780_write_data(*str);
      break;
    }
    str++;
  }
}
//==============================================================================


//==============================================================================
// ��������� ���������������� ������ ������� � ������� ������� �������
//==============================================================================
void hd44780_printf(const char *args, ...)
{
  va_list ap;
  va_start(ap, args);
  char len = vsnprintf(hd44780_StrBuff, sizeof(hd44780_StrBuff), args, ap);
  va_end(ap);

  // ��������������� ������� �������� � ��������� EPSON ��� �������� hd44780
#if (SOURCE_CODEPAGE == CP1251)
  cp1251_to_epson_convert(hd44780_StrBuff);
#else
  cp866_to_epson_convert(hd44780_StrBuff);
#endif
  
  hd44780_puts(hd44780_StrBuff);
}
//==============================================================================
