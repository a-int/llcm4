#include <llcm.h>
#include <usart.h>

volatile uint32_t delay_ms_time = 0;

int main() {
  initSW();           // setup pins for Serial Wire debug
  init_clock_100();    // initialize MPU to work at 100 MHz
  init_SysTick(1000);  // initialize SysTick to work at ms
  init_usart2_115200();
  while (1) {}
}

void SysTick_Handler(void) {
  if (delay_ms_time > 0) {
    --delay_ms_time;
  }
}

void USART2_IRQHandler(void) {
  if (READ_BIT(USART2->SR, USART_SR_RXNE)) {  //if data may be read
    if ('F' == USART2->DR) {
      const char* msg = "Hello world!\n";
      usart_send((uint8_t*)msg, 13);
    }
  }
  if (READ_BIT(USART2->SR, USART_SR_IDLE)) {
    USART2->DR;  // reset IDLE flag
  }
}
