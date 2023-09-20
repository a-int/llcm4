#include <flash.h>
#include <llcm.h>
#include <usart.h>

volatile uint32_t delay_ms_time = 0;

#define USART_RX_DATA_MAX 20

struct USART {
  uint8_t rx_data[USART_RX_DATA_MAX];
  uint32_t rx_data_counter;
};
struct USART huart2;

int main() {
  initSW();              // init serial wire
  init_clock_HSE(25, 200, pllp_div2, AHB_div1, APB_div2, APB_div1);      //initialize MPU to work at 100 MHz
  init_SysTick(1000);    // initialize SysTick to work at ms
  init_usart2(115200);  // initialize USART2 with Baudrate = 115200
  while (1) {}
}

void SysTick_Handler(void) {
  if (delay_ms_time > 0) {
    --delay_ms_time;
  }
}

void USART2_IRQHandler() {
  if (READ_BIT(USART2->SR, USART_SR_RXNE)) {  //if data may be read
    if (huart2.rx_data_counter < USART_RX_DATA_MAX) {
      huart2.rx_data[huart2.rx_data_counter++] = (uint8_t)USART2->DR;
    }
    if (huart2.rx_data_counter == USART_RX_DATA_MAX) {  //the whole chunk was recieved
      uint32_t* destination = (uint32_t*)0x8008000;     //start from sector 3 (0x08008000)
      __disable_irq();                                  //disable all irqs
      FLASH_unlock();
      FLASH_clear_section(2);  // sector 3: 0x0800_8000  -  0x0800_bfff
      FLASH_write((uint32_t*)huart2.rx_data, destination, huart2.rx_data_counter/4);
      FLASH_lock();
      __enable_irq();  //enable irqs back
    }
  }
  if (READ_BIT(USART2->SR, USART_SR_IDLE)) {
    USART2->DR;  //reset IDLE flag
  }
}
