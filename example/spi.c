#include <usart.h>
#include <spi.h>
#include <llcm.h>

#define USART USART1

int main(){
  initSW();
  init_clock_HSE(25, 200, pllp_div2, AHB_div1, APB_div2, APB_div1);
  init_SysTick(1000);
  init_usart(USART1, eBaudRate115200);

  init_spi1_gpio(0,0);
  init_spi1_fullDuplex(spiCpol0, spiCpha0, spiBaudRateDiv2, spiMaster, spiMsbfirst, spiDff8, spiHardwareSlaveManagement);
  
  while(1){}
}

void SysTick_Handler(void){}

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
