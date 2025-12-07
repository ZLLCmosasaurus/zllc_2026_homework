#include "control_frame.h"

void ControlFrame_Init(ControlFrame_t *f)
{
  if (f == NULL)
  {
    return;
  }

  f->ch0 = 1024U;
  f->ch1 = 1024U;
  f->ch2 = 1024U;
  f->ch3 = 1024U;

  f->s1 = 3U;
  f->s2 = 3U;

  f->mouse_x = 0;
  f->mouse_y = 0;
  f->mouse_z = 0;

  f->mouse_left  = 0U;
  f->mouse_right = 0U;

  f->key     = 0U;
  f->reserve = 0U;
}

void ControlFrame_Pack(uint8_t buf[18], const ControlFrame_t *f)
{
  if ((buf == NULL) || (f == NULL))
  {
    return;
  }

  for (int i = 0; i < 18; i++)
  {
    buf[i] = 0U;
  }

  uint16_t ch0 = (uint16_t)(f->ch0 & 0x07FFU);
  uint16_t ch1 = (uint16_t)(f->ch1 & 0x07FFU);
  uint16_t ch2 = (uint16_t)(f->ch2 & 0x07FFU);
  uint16_t ch3 = (uint16_t)(f->ch3 & 0x07FFU);

  buf[0] = (uint8_t)(ch0 & 0xFFU);
  buf[1] = (uint8_t)((ch0 >> 8) | ((ch1 & 0x001FU) << 3));
  buf[2] = (uint8_t)((ch1 >> 5) | ((ch2 & 0x0003U) << 6));
  buf[3] = (uint8_t)((ch2 >> 2) & 0xFFU);
  buf[4] = (uint8_t)((ch2 >> 10) | ((ch3 & 0x007FU) << 1));
  buf[5] = (uint8_t)((ch3 >> 7) & 0x0FU);

  buf[5] |= (uint8_t)((f->s2 & 0x03U) << 4);
  buf[5] |= (uint8_t)((f->s1 & 0x03U) << 6);

  buf[6]  = (uint8_t)(f->mouse_x & 0xFF);
  buf[7]  = (uint8_t)((f->mouse_x >> 8) & 0xFF);
  buf[8]  = (uint8_t)(f->mouse_y & 0xFF);
  buf[9]  = (uint8_t)((f->mouse_y >> 8) & 0xFF);
  buf[10] = (uint8_t)(f->mouse_z & 0xFF);
  buf[11] = (uint8_t)((f->mouse_z >> 8) & 0xFF);

  buf[12] = (uint8_t)(f->mouse_left  & 0xFFU);
  buf[13] = (uint8_t)(f->mouse_right & 0xFFU);

  buf[14] = (uint8_t)(f->key & 0xFFU);
  buf[15] = (uint8_t)((f->key >> 8) & 0xFFU);

  buf[16] = (uint8_t)(f->reserve & 0xFFU);
  buf[17] = (uint8_t)((f->reserve >> 8) & 0xFFU);
}

void ControlFrame_Decode(const uint8_t buf[18], ControlFrame_t *f)
{
  if ((buf == NULL) || (f == NULL))
  {
    return;
  }

  const uint8_t *pData = buf;

  f->ch0 = (uint16_t)(((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF);

  f->ch1 = (uint16_t)((((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF);

  f->ch2 = (uint16_t)((((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF);

  f->ch3 = (uint16_t)((((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF);

  f->s1 = (uint8_t)((pData[5] >> 4 & 0x000C) >> 2);
  f->s2 = (uint8_t)(pData[5] >> 4 & 0x0003);

  f->mouse_x = (int16_t)((int16_t)pData[6]  | ((int16_t)pData[7]  << 8));
  f->mouse_y = (int16_t)((int16_t)pData[8]  | ((int16_t)pData[9]  << 8));
  f->mouse_z = (int16_t)((int16_t)pData[10] | ((int16_t)pData[11] << 8));

  f->mouse_left  = pData[12];
  f->mouse_right = pData[13];

  f->key     = (uint16_t)((uint16_t)pData[14] | ((uint16_t)pData[15] << 8));
  f->reserve = (uint16_t)((uint16_t)pData[16] | ((uint16_t)pData[17] << 8));
}

HAL_StatusTypeDef ControlFrame_ReceiveAndDecode(UART_HandleTypeDef *huart, ControlFrame_t *f, uint8_t buf[18], uint32_t timeout)
{
  if ((huart == NULL) || (f == NULL) || (buf == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_UART_Receive(huart, buf, 18U, timeout);

  if (status == HAL_OK)
  {
    ControlFrame_Decode(buf, f);
  }

  return status;
}
