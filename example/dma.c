#include <llcm.h>
#include <usart.h>

void init_DMA();

volatile uint32_t delay_ms_time = 0;

#define USART_RX_DATA_MAX 10
uint8_t rx_data[USART_RX_DATA_MAX];

int main() {
  initSW();                  //setup serial wire
  init_clock_100();          //initialize MPU to work at 100 MHz
  init_SysTick(1000);        //initialize SysTick to work at ms
  init_usart2_dma_115200();  //initialize usart2 to work with dma
  init_DMA();
  while (1) {}
}

void SysTick_Handler(void) {
  if (delay_ms_time > 0) {
    --delay_ms_time;
  }
}

void init_DMA() {
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);  // enable DMA1 in AHB1

  CLEAR_BIT(DMA1_Stream5->CR, DMA_SxCR_EN_Msk);                  //stop DMA if enabled
  while (READ_BIT(DMA1_Stream5->CR, (1 << DMA_SxCR_EN_Pos))) {}  //wait for disabling

  WRITE_REG(DMA1_Stream5->PAR, ((uint32_t)&USART2->DR));                    //set the peripheral address
  WRITE_REG(DMA1_Stream5->M0AR, ((uint32_t)&rx_data));                      //set the memory address
  MODIFY_REG(DMA1_Stream5->CR, DMA_SxCR_PSIZE, (0 << DMA_SxCR_PSIZE_Pos));  //select peripheral data size 1 byte
  MODIFY_REG(DMA1_Stream5->CR, DMA_SxCR_MSIZE, (0 << DMA_SxCR_MSIZE_Pos));  //select memory data size 1 byte
  MODIFY_REG(DMA1_Stream5->NDTR, DMA_SxNDT, USART_RX_DATA_MAX);             //number of bytes in incoming data

  MODIFY_REG(DMA1_Stream5->CR, DMA_SxCR_DIR, (0 << DMA_SxCR_DIR_Pos));      //select peripheral to memory direction
  CLEAR_BIT(DMA1_Stream5->CR, DMA_SxCR_PINC);                               //peripheral address is constant
  SET_BIT(DMA1_Stream5->CR, DMA_SxCR_MINC);                                 //memory address is automatically incremented
  CLEAR_BIT(DMA1_Stream5->CR, DMA_SxCR_CT);                                 //current memory target M0AR
  SET_BIT(DMA1_Stream5->CR, DMA_SxCR_CIRC);                                 //enable circular mode
  MODIFY_REG(DMA1_Stream5->CR, DMA_SxCR_PL, (0 << DMA_SxCR_PL_Pos));  //select low priority (doesnt matter if only one DMA at the time)
  MODIFY_REG(DMA1_Stream5->CR, DMA_SxCR_CHSEL, (4 << DMA_SxCR_CHSEL_Pos));   //select 4th Chanel
  SET_BIT(DMA1_Stream5->CR, DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_TCIE);  //enable interrupts

  SET_BIT(DMA1_Stream5->CR, DMA_SxCR_EN_Msk);                     //enable DMA
  while (!READ_BIT(DMA1_Stream5->CR, (1 << DMA_SxCR_EN_Pos))) {}  //wait for enabling

  NVIC_EnableIRQ(DMA1_Stream5_IRQn);  // enable IRQ for Stream 5
}

void DMA1_Stream5_IRQHandler(void) {
  if (READ_BIT(DMA1->HISR, (1 << DMA_HISR_HTIF5_Pos))) {  // if half of the data is recieved
    char msg[] = "Half done\n";
    usart_send((uint8_t*)msg, sizeof(msg) / sizeof(msg[0]) - 1);
    SET_BIT(DMA1->HIFCR, DMA_HIFCR_CHTIF5);  // clear interrupt flag
  }
  if (READ_BIT(DMA1->HISR, (1 << DMA_HISR_TCIF5_Pos))) {  // if data recieved completely
    char msg[] = "Complete\n";
    usart_send((uint8_t*)msg, sizeof(msg) / sizeof(msg[0]) - 1);
    SET_BIT(DMA1->HIFCR, DMA_HIFCR_CTCIF5);  // clear interrupt flag
  }
  if (READ_BIT(DMA1->HISR, (1 << DMA_HISR_TEIF5_Pos))) {  // if an error occurred
    char msg[] = "Error\n";
    usart_send((uint8_t*)msg, sizeof(msg) / sizeof(msg[0]) - 1);
    SET_BIT(DMA1->HIFCR, DMA_HIFCR_CTEIF5);  // clear interrupt flag
  }
}