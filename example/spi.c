#include <usart.h>
#include <spi.h>
#include <llcm.h>
#include "stm32f411xe.h"

volatile uint32_t delay_ms_time = 0;
void delay(uint32_t ticks){
  delay_ms_time = ticks;
  while(delay_ms_time){}
}
#define USART USART1

void hspi_w8(SPI_TypeDef* SPIx, uint8_t dat) {
  // Wait for TXE.
  while (!(SPIx->SR & SPI_SR_TXE)) {};
  // Send the byte.
  *(uint8_t*)&(SPIx->DR) = dat;
}

void hspi_w16(SPI_TypeDef *SPIx, uint16_t dat) {
  // Wait for TXE.
  while (!(SPIx->SR & SPI_SR_TXE)) {};
  // Send the data.
  // (Flip the bytes for the little-endian ARM core.)
  dat = (((dat & 0x00FF) << 8) | ((dat & 0xFF00) >> 8));
  *(uint16_t*)&(SPIx->DR) = dat;
  // hspi_w8(SPIx, (uint8_t)(dat >> 8));
  // hspi_w8(SPIx, (uint8_t)(dat & 0xFF));
}

void hspi_cmd(SPI_TypeDef *SPIx, uint8_t cmd) {
  while ((SPIx->SR & SPI_SR_BSY)) {};
  GPIOB->ODR &= ~(1 << 2);
  hspi_w8(SPIx, cmd);
  while ((SPIx->SR & SPI_SR_BSY)) {};
  GPIOB->ODR |=  (1 << 2);
}

#define SPIx SPI1

int main(){
  initSW();
  init_clock_HSE(25, 200, pllp_div2, AHB_div1, APB_div2, APB_div1);
  init_SysTick(1000);
  init_usart(USART1, eBaudRate115200);

  // init_spi1_gpio(1,0,1);
  ili9341_spi1_init();
  // spi_select();
  // (Display off)
  //hspi_cmd(SPIx, 0x28);
  // Issue a series of initialization commands from the
  // Adafruit library for a simple 'known good' test.
  // (TODO: Add named macro definitions for these hex values.)

  // RESET
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);                      //enable GPIOC
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE0, GPIO_MODER_MODE3_0);  // set as output
  CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT0_Msk);                   //select push-pull mode
  CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD0_Msk);                   //select no push no pull mode
    
  //DC
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);                      //enable GPIOC
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE2, GPIO_MODER_MODE2_0);  // set as output
  CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT2_Msk);                   //select push-pull mode
  CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD2_Msk);                   //select no push no pull mode

  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);                      //enable GPIOC
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE1, GPIO_MODER_MODE1_0);  // set as output
  CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT1_Msk);                   //select push-pull mode
  CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD1_Msk);                   //select no push no pull mode

  GPIOB->ODR |=  (1 << 1);
  //   (See the 'sspi_cmd' method for 'DC' pin info.)
  GPIOB->ODR |=  (1 << 2);
  // Set SCK high to start
  GPIOA->ODR |=  (1 << 5);
  // Reset the display by pulling the reset pin low,
  // delaying a bit, then pulling it high.
  GPIOB->ODR &= ~(1 << 0);
  // Delay at least 100ms; meh, call it 2 million no-ops.
  delay(200);
  GPIOB->ODR |=  (1 << 0);
  delay(200);

  hspi_cmd(SPIx, 0xEF);
  hspi_w8(SPIx, 0x03);
  hspi_w8(SPIx, 0x80);
  hspi_w8(SPIx, 0x02);
  hspi_cmd(SPIx, 0xCF);
  hspi_w8(SPIx, 0x00);
  hspi_w8(SPIx, 0xC1);
  hspi_w8(SPIx, 0x30);
  hspi_cmd(SPIx, 0xED);
  hspi_w8(SPIx, 0x64);
  hspi_w8(SPIx, 0x03);
  hspi_w8(SPIx, 0x12);
  hspi_w8(SPIx, 0x81);
  hspi_cmd(SPIx, 0xE8);
  hspi_w8(SPIx, 0x85);
  hspi_w8(SPIx, 0x00);
  hspi_w8(SPIx, 0x78);
  hspi_cmd(SPIx, 0xCB);
  hspi_w8(SPIx, 0x39);
  hspi_w8(SPIx, 0x2C);
  hspi_w8(SPIx, 0x00);
  hspi_w8(SPIx, 0x34);
  hspi_w8(SPIx, 0x02);
  hspi_cmd(SPIx, 0xF7);
  hspi_w8(SPIx, 0x20);
  hspi_cmd(SPIx, 0xEA);
  hspi_w8(SPIx, 0x00);
  hspi_w8(SPIx, 0x00);
  // PWCTR1
  hspi_cmd(SPIx, 0xC0);
  hspi_w8(SPIx, 0x23);
  // PWCTR2
  hspi_cmd(SPIx, 0xC1);
  hspi_w8(SPIx, 0x10);
  // VMCTR1
  hspi_cmd(SPIx, 0xC5);
  hspi_w8(SPIx, 0x3E);
  hspi_w8(SPIx, 0x28);
  // VMCTR2
  hspi_cmd(SPIx, 0xC7);
  hspi_w8(SPIx, 0x86);
  // MADCTL
  hspi_cmd(SPIx, 0x36);
  hspi_w8(SPIx, 0x48);
  // VSCRSADD
  hspi_cmd(SPIx, 0x37);
  hspi_w8(SPIx, 0x00);
  // PIXFMT
  hspi_cmd(SPIx, 0x3A);
  hspi_w8(SPIx, 0x55);
  // FRMCTR1
  hspi_cmd(SPIx, 0xB1);
  hspi_w8(SPIx, 0x00);
  hspi_w8(SPIx, 0x18);
  // DFUNCTR
  hspi_cmd(SPIx, 0xB6);
  hspi_w8(SPIx, 0x08);
  hspi_w8(SPIx, 0x82);
  hspi_w8(SPIx, 0x27);
  hspi_cmd(SPIx, 0xF2);
  hspi_w8(SPIx, 0x00);
  // GAMMASET
  hspi_cmd(SPIx, 0x26);
  hspi_w8(SPIx, 0x01);
  // (Actual gamma settings)
  hspi_cmd(SPIx, 0xE0);
  hspi_w8(SPIx, 0x0F);
  hspi_w8(SPIx, 0x31);
  hspi_w8(SPIx, 0x2B);
  hspi_w8(SPIx, 0x0C);
  hspi_w8(SPIx, 0x0E);
  hspi_w8(SPIx, 0x08);
  hspi_w8(SPIx, 0x4E);
  hspi_w8(SPIx, 0xF1);
  hspi_w8(SPIx, 0x37);
  hspi_w8(SPIx, 0x07);
  hspi_w8(SPIx, 0x10);
  hspi_w8(SPIx, 0x03);
  hspi_w8(SPIx, 0x0E);
  hspi_w8(SPIx, 0x09);
  hspi_w8(SPIx, 0x00);
  hspi_cmd(SPIx, 0xE1);
  hspi_w8(SPIx, 0x00);
  hspi_w8(SPIx, 0x0E);
  hspi_w8(SPIx, 0x14);
  hspi_w8(SPIx, 0x03);
  hspi_w8(SPIx, 0x11);
  hspi_w8(SPIx, 0x07);
  hspi_w8(SPIx, 0x31);
  hspi_w8(SPIx, 0xC1);
  hspi_w8(SPIx, 0x48);
  hspi_w8(SPIx, 0x08);
  hspi_w8(SPIx, 0x0F);
  hspi_w8(SPIx, 0x0C);
  hspi_w8(SPIx, 0x31);
  hspi_w8(SPIx, 0x36);
  hspi_w8(SPIx, 0x0F);
  // Exit sleep mode.
  hspi_cmd(SPIx, 0x11);
  delay(200);
  // Display on.
  hspi_cmd(SPIx, 0x29);
  delay(200);
  // 'Normal' display mode.
  hspi_cmd(SPIx, 0x13);

  while(1){
    // Main loop - empty the screen as a test.
    int tft_iter = 0;
    int tft_on = 0;
    // Set column range.
    hspi_cmd(SPI1, 0x2A);
    hspi_w16(SPI1, 0x0000);
    hspi_w16(SPI1, (uint16_t)(239));
    // Set row range.
    hspi_cmd(SPI1, 0x2B);
    hspi_w16(SPI1, 0x0000);
    hspi_w16(SPI1, (uint16_t)(319));
    // Set 'write to RAM'
    hspi_cmd(SPI1, 0x2C);
    while (1) {
      // Write 320 * 240 pixels.
      for (tft_iter = 0; tft_iter < (320*240); ++tft_iter) {
        // Write a 16-bit color.
        if (tft_on) {
          hspi_w16(SPI1, 0xF800);
        }
        else {
          hspi_w16(SPI1, 0x001F);
        }
      }
      tft_on = !tft_on;
    }
  }
}

void SysTick_Handler(void){
  if(delay_ms_time){
    --delay_ms_time;
  }
}

void USART1_IRQHandler(void) {
  if (isRXNE(USART)) {  //if data may be read
    if ('F' == USART->DR) {
      GPIOC->ODR ^= 1 << 13;
      const char* msg = "Hello world!\r\n";
      usart_send((uint8_t*)msg, strlen(msg), USART);
    }
  }
  if (isIDLE(USART)) {
      USART->DR;  // reset IDLE flag
      const char* msg = "IDLE LINE DETECTED\r\n";
      usart_send((uint8_t*)msg, strlen(msg), USART);
  }
}
