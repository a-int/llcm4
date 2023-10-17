#include <llcm_gpio.h>
#include <spi.h>
#include "stm32f411xe.h"
#include "stm32f4xx.h"

void init_spi1_gpio(uint8_t isMOSIUsed, uint8_t isMISOUsed, uint8_t isNSSUsed) {
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);  // enable gpio A for spi 1
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);   // enable spi 1

  if (isNSSUsed) {
    GPIOA->MODER &= ~(GPIO_MODER_MODE4);                                       // clear default mode bits
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODE4_1);                                 // set as alternative function
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL4, AF5 << GPIO_AFRL_AFSEL4_Pos);  // select Alternative function 5 (SPI_NSS)
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR4);                           // select High speed
  }

  GPIOA->MODER &= ~(GPIO_MODER_MODE5);                                       // clear default mode bits
  SET_BIT(GPIOA->MODER, GPIO_MODER_MODE5_1);                                 // set as alternative function
  MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL5, AF5 << GPIO_AFRL_AFSEL5_Pos);  // select Alternative function 5 (SPI_SCK)
  SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR5);                           // select High speed

  if (isMISOUsed) {
    GPIOA->MODER &= ~(GPIO_MODER_MODE6);                                       // clear default mode bits
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODE6_1);                                 // set as alternative function
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL6, AF5 << GPIO_AFRL_AFSEL6_Pos);  // select Alternative function 5 (SPI_MISO)
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR6);                           // select High speed
  }
  if (isMOSIUsed) {
    GPIOA->MODER &= ~(GPIO_MODER_MODE7);                                       // clear default mode bits
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODE7_1);                                 // set as alternative function
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL7, AF5 << GPIO_AFRL_AFSEL7_Pos);  // select Alternative function 5 (SPI MOSI)
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR7);                           // select High speed
  }
}

void init_spi1_fullDuplex(SPI_CPOL cpol, SPI_CPHA cpha, SPI_BR baudRate, SPI_MODE mode, SPI_SBFIRST sbFirst, SPI_DFF dff,
                                 SPI_SSM ssm) {
  SPI1->CR1 = 0;
  SPI1->CR2 = 0;

  CLEAR_BIT(SPI1->CR1, SPI_CR1_SSI);    // clear internal slave select
  CLEAR_BIT(SPI1->CR2, SPI_CR2_FRF);    // set motorola spi mode
  CLEAR_BIT(SPI1->CR1, SPI_CR1_CRCEN);  // disable CRS calculation

  if (mode == spiMaster) {
    SET_BIT(SPI1->CR2, SPI_CR2_SSOE);  // SS work as output if master mode (low on input lead to error)
  } else {
    CLEAR_BIT(SPI1->CR2, SPI_CR2_SSOE);  // SS work as input
  }

  //set full-duplex mode
  CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIMODE);
  CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
  CLEAR_BIT(SPI1->CR1, SPI_CR1_RXONLY);

  MODIFY_REG(SPI1->CR1, SPI_CR1_DFF, (dff << SPI_CR1_DFF_Pos));                // select frame size
  MODIFY_REG(SPI1->CR1, SPI_CR1_SSM, (ssm << SPI_CR1_SSM_Pos));                // select software/hardware slave selection
  MODIFY_REG(SPI1->CR1, SPI_CR1_LSBFIRST, (sbFirst << SPI_CR1_LSBFIRST_Pos));  // select LSB or MSB first
  MODIFY_REG(SPI1->CR1, SPI_CR1_BR, (baudRate << SPI_CR1_BR_Pos));             // select baud rate
  MODIFY_REG(SPI1->CR1, SPI_CR1_MSTR, (mode << SPI_CR1_MSTR_Pos));
  MODIFY_REG(SPI1->CR1, SPI_CR1_CPOL, (cpol << SPI_CR1_CPOL_Pos));
  MODIFY_REG(SPI1->CR1, SPI_CR1_CPHA, (cpha << SPI_CR1_CPHA_Pos));

  SET_BIT(SPI1->CR1, SPI_CR1_SPE);  // enable SPI
}

void spi_sendByte(uint8_t data, SPI_TypeDef* SPIx) {
  while (!(SPIx->SR & SPI_SR_TXE)) {}  //wait until ready to transmit
  *(uint8_t*)&SPIx->DR = data;
}

void spi_send(uint8_t* data, uint32_t size, SPI_TypeDef* SPIx) {
  while (size--) {
    spi_sendByte(*data++, SPIx);
  }
}

void spi_select(GPIO_TypeDef* GPIOx, uint8_t pin){
  while(SPI1->SR & SPI_SR_BSY){}
  GPIOx->ODR &= ~(1<<pin);
}
void spi_unselect(GPIO_TypeDef* GPIOx, uint8_t pin){
  while(SPI1->SR & SPI_SR_BSY){}
  GPIOx->ODR |= (1<<pin);
}
