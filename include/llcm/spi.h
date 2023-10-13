#pragma once
#include "stm32f4xx.h"

typedef enum {
  spiBaudRateDiv2,
  spiBaudRateDiv4,
  spiBaudRateDiv8,
  spiBaudRateDiv16,
  spiBaudRateDiv32,
  spiBaudRateDiv64,
  spiBaudRateDiv128,
  spiBaudRateDiv256
} SPI_BR;

typedef enum { spiDff8, spiDff16 } SPI_DFF;
typedef enum { spiCpol0, spiCpol1 } SPI_CPOL;
typedef enum { spiCpha0, spiCpha1 } SPI_CPHA;
typedef enum { spiMaster, spiSlave } SPI_MODE;
typedef enum { spiMsbfirst, spiLsbfirst } SPI_SBFIRST;
typedef enum { spiHardwareSlaveManagement, spiSoftwareSlaveManagement } SPI_SSM;

void init_spi1_gpio(uint8_t isMisoUsed, uint8_t isNssUsed);
void init_spi1_fullDuplex(SPI_CPOL cpol, SPI_CPHA cpha, SPI_BR baudRate, SPI_MODE mode, SPI_SBFIRST sbFirst, SPI_DFF dff, SPI_SSM ssm);
