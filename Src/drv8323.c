#include "drv8323.h"
#include <stdio.h>
#include "hw_config.h"

//DRV8353 = 1Mhz

uint16_t drv_spi_write(DRVStruct * drv, uint16_t val){
	drv->spi_tx_word = val;
	HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&DRV_SPI, (uint8_t*)drv->spi_tx_buff, (uint8_t *)drv->spi_rx_buff, 1, 10);
	while(DRV_SPI.State == HAL_SPI_STATE_BUSY);  					// wait for transmission complete
	HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
	return (drv->spi_rx_word)&0x7FF;
}
uint16_t drv_read_FSR1(DRVStruct drv){
	return drv_spi_write(&drv, 0x8000|(FSR1<<11));
}

uint16_t drv_read_FSR2(DRVStruct drv){
	return drv_spi_write(&drv, 0x8000|(FSR2<<11));
}

uint16_t drv_read_register(DRVStruct drv, int reg){
	return drv_spi_write(&drv, 0x8000|(reg<<11));
}
void drv_write_register(DRVStruct drv, int reg, int val){
	drv_spi_write(&drv, (reg<<11)|val);
}
void drv_write_DCR(DRVStruct drv, int OCP_ACT, int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT){
	uint16_t val = (DCR<<11) | (OCP_ACT<<10) | (DIS_CPUV<<9) | (DIS_GDF<<8) | (OTW_REP<<7) | (PWM_MODE<<5) | (PWM_COM<<4) | (PWM_DIR<<3) | (COAST<<2) | (BRAKE<<1) | CLR_FLT;
	drv_spi_write(&drv, val);
	printf("DCR_WRITE = 0x%04X\r\n", val&0x7FF);
}
void drv_write_HSR(DRVStruct drv, int LOCK, int IDRIVEP_HS, int IDRIVEN_HS){
	uint16_t val = (HSR<<11) | (LOCK<<8) | (IDRIVEP_HS<<4) | IDRIVEN_HS;
	drv_spi_write(&drv, val);
	printf("HSR_WRITE = 0x%04X\r\n", val&0x7FF);
}
void drv_write_LSR(DRVStruct drv, int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS){
	uint16_t val = (LSR<<11) | (CBC<<10) | (TDRIVE<<8) | (IDRIVEP_LS<<4) | IDRIVEN_LS;
	drv_spi_write(&drv, val);
	printf("LSR_WRITE = 0x%04X\r\n", val&0x7FF);
}
void drv_write_OCPCR(DRVStruct drv, int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL){
	uint16_t val = (OCPCR<<11) | (TRETRY<<10) | (DEAD_TIME<<8) | (OCP_MODE<<6) | (OCP_DEG<<4) | VDS_LVL;
	drv_spi_write(&drv, val);
	printf("OCPCR_WRITE = 0x%04X\r\n", val&0x7FF);
}
void drv_write_CSACR(DRVStruct drv, int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL){
	uint16_t val = (CSACR<<11) | (CSA_FET<<10) | (VREF_DIV<<9) | (LS_REF<<8) | (CSA_GAIN<<6) | (DIS_SEN<<5) | (CSA_CAL_A<<4) | (CSA_CAL_B<<3) | (CSA_CAL_C<<2) | SEN_LVL;
	drv_spi_write(&drv, val);
	printf("CSACR_WRITE = 0x%04X\r\n", val&0x7FF);
}

void drv_write_CAL_MODE(DRVStruct drv, int CAL_MODE) {
    uint16_t val = (CAL << 11) | (CAL_MODE & 0x01);
    drv_spi_write(&drv, val);
}
void drv_enable_gd(DRVStruct drv){
	uint16_t val = (drv_read_register(drv, DCR)) & (~(0x1<<2));
	drv_write_register(drv, DCR, val);
}
void drv_disable_gd(DRVStruct drv){
	uint16_t val = (drv_read_register(drv, DCR)) | (0x1<<2);
	drv_write_register(drv, DCR, val);
}
void drv_calibrate(DRVStruct drv){
	uint16_t val = (0x1<<4) + (0x1<<3) + (0x1<<2);
	drv_write_register(drv, CSACR, val);
}
void drv_print_faults(DRVStruct drv){
    uint16_t val1 = drv_read_FSR1(drv);
    uint16_t val2 = drv_read_FSR2(drv);

    if(val1 & (1<<10)){printf("\n\rFAULT\n\r");}

    if(val1 & (1<<9)){printf("VDS_OCP\r\n");}
    if(val1 & (1<<8)){printf("GDF\r\n");}
    if(val1 & (1<<7)){printf("UVLO\r\n");}
    if(val1 & (1<<6)){printf("OTSD\r\n");}
    if(val1 & (1<<5)){printf("VDS_HA\r\n");}
    if(val1 & (1<<4)){printf("VDS_LA\r\n");}
    if(val1 & (1<<3)){printf("VDS_HB\r\n");}
    if(val1 & (1<<2)){printf("VDS_LB\r\n");}
    if(val1 & (1<<1)){printf("VDS_HC\r\n");}
    if(val1 & (1)){printf("VDS_LC\r\n");}

    if(val2 & (1<<10)){printf("SA_OC\r\n");}
    if(val2 & (1<<9)){printf("SB_OC\r\n");}
    if(val2 & (1<<8)){printf("SC_OC\r\n");}
    if(val2 & (1<<7)){printf("OTW\r\n");}
    if(val2 & (1<<6)){printf("CPUV\r\n");}
    if(val2 & (1<<5)){printf("VGS_HA\r\n");}
    if(val2 & (1<<4)){printf("VGS_LA\r\n");}
    if(val2 & (1<<3)){printf("VGS_HB\r\n");}
    if(val2 & (1<<2)){printf("VGS_LB\r\n");}
    if(val2 & (1<<1)){printf("VGS_HC\r\n");}
    if(val2 & (1)){printf("VGS_LC\r\n");}

}

void drv_print_faults_values(DRVStruct drv) {
    uint16_t val1 = drv_read_FSR1(drv);
    HAL_Delay(1);
    uint16_t val2 = drv_read_FSR2(drv);
    
    printf("\n--- FSR1 Register (0x%04X) ---\r\n", val1);
    printf("Bit 10 (FAULT): %d\r\n", (val1 >> 10) & 1);
    printf("Bit 9 (VDS_OCP): %d\r\n", (val1 >> 9) & 1);
    printf("Bit 8 (GDF): %d\r\n", (val1 >> 8) & 1);
    printf("Bit 7 (UVLO): %d\r\n", (val1 >> 7) & 1);
    printf("Bit 6 (OTSD): %d\r\n", (val1 >> 6) & 1);
    printf("Bit 5 (VDS_HA): %d\r\n", (val1 >> 5) & 1);
    printf("Bit 4 (VDS_LA): %d\r\n", (val1 >> 4) & 1);
    printf("Bit 3 (VDS_HB): %d\r\n", (val1 >> 3) & 1);
    printf("Bit 2 (VDS_LB): %d\r\n", (val1 >> 2) & 1);
    printf("Bit 1 (VDS_HC): %d\r\n", (val1 >> 1) & 1);
    printf("Bit 0 (VDS_LC): %d\r\n", (val1 >> 0) & 1);

    printf("\n--- FSR2 Register (0x%04X) ---\r\n", val2);
    printf("Bit 10 (SA_OC): %d\r\n", (val2 >> 10) & 1);
    printf("Bit 9 (SB_OC): %d\r\n", (val2 >> 9) & 1);
    printf("Bit 8 (SC_OC): %d\r\n", (val2 >> 8) & 1);
    printf("Bit 7 (OTW): %d\r\n", (val2 >> 7) & 1);
    printf("Bit 6 (CPUV): %d\r\n", (val2 >> 6) & 1);
    printf("Bit 5 (VGS_HA): %d\r\n", (val2 >> 5) & 1);
    printf("Bit 4 (VGS_LA): %d\r\n", (val2 >> 4) & 1);
    printf("Bit 3 (VGS_HB): %d\r\n", (val2 >> 3) & 1);
    printf("Bit 2 (VGS_LB): %d\r\n", (val2 >> 2) & 1);
    printf("Bit 1 (VGS_HC): %d\r\n", (val2 >> 1) & 1);
    printf("Bit 0 (VGS_LC): %d\r\n", (val2 >> 0) & 1);
}


void drv_print_settings(DRVStruct drv) {
   uint16_t dcr = drv_read_register(drv, DCR);
   uint16_t lsr = drv_read_register(drv, LSR);

   printf("\n--- DRV8353 Settings ---\r\n");

   // Drive Control Register (DCR)
   printf("DCR (0x%04X):\r\n", dcr);
   printf("- DIS_CPUV: %d\r\n", (dcr >> 9) & 0x1);
   printf("- DIS_GDF: %d\r\n", (dcr >> 8) & 0x1);
   printf("- OTW_REP: %d\r\n", (dcr >> 7) & 0x1);
   printf("- PWM_MODE: %d\r\n", (dcr >> 5) & 0x3);
   printf("- PWM_COM: %d\r\n", (dcr >> 4) & 0x1);
   printf("- PWM_DIR: %d\r\n", (dcr >> 3) & 0x1);
   printf("- COAST: %d\r\n", (dcr >> 2) & 0x1);
   printf("- BRAKE: %d\r\n", (dcr >> 1) & 0x1);
   printf("- CLR_FLT: %d\r\n", dcr & 0x1);

   // Gate Drive LS Register (LSR)
   printf("\nLSR (0x%04X):\r\n", lsr);
   printf("- CBC: %d\r\n", (lsr >> 10) & 0x1);
   printf("- TDRIVE: %d\r\n", (lsr >> 8) & 0x3);
   printf("- IDRIVEP_LS: %d\r\n", (lsr >> 4) & 0xF);
   printf("- IDRIVEN_LS: %d\r\n", lsr & 0xF);

}

void drv_print_settings(DRVStruct drv) {
    // Chỉ DCR và LSR có khả năng readback
    uint16_t dcr = drv_read_register(drv, DCR);
    uint16_t lsr = drv_read_register(drv, LSR);

    printf("\n--- DRV8353 Settings ---\r\n");

    // Drive Control Register (DCR)
    printf("DCR (0x%04X):\r\n", dcr);
    printf("- DIS_CPUV: %d\r\n", (dcr >> 9) & 0x1);
    printf("- DIS_GDF: %d\r\n", (dcr >> 8) & 0x1);
    printf("- OTW_REP: %d\r\n", (dcr >> 7) & 0x1);
    printf("- PWM_MODE: %d\r\n", (dcr >> 5) & 0x3);
    printf("- PWM_COM: %d\r\n", (dcr >> 4) & 0x1);
    printf("- PWM_DIR: %d\r\n", (dcr >> 3) & 0x1);
    printf("- COAST: %d\r\n", (dcr >> 2) & 0x1);
    printf("- BRAKE: %d\r\n", (dcr >> 1) & 0x1);
    printf("- CLR_FLT: %d\r\n", dcr & 0x1);

    // Gate Drive LS Register (LSR) có khả năng đọc lại
    printf("\nLSR (0x%04X):\r\n", lsr);
    printf("- CBC: %d\r\n", (lsr >> 10) & 0x1);
    printf("- TDRIVE: %d\r\n", (lsr >> 8) & 0x3);
    printf("- IDRIVEP_LS: %d\r\n", (lsr >> 4) & 0xF);
    printf("- IDRIVEN_LS: %d\r\n", lsr & 0xF);
}




void drv_check_spi(DRVStruct drv) {

	uint16_t response = 0;
    
    response = drv_spi_write(&drv, 0x8000|(DCR<<11));
    printf("SPI Response: 0x%04X\r\n", response);

//    printf("CS Pin: %d\r\n", HAL_GPIO_ReadPin(DRV_CS));
//    printf("Enable Pin: %d\r\n", HAL_GPIO_ReadPin(ENABLE_PIN));

    if(response == 0 || response == 0x7FF) {
        printf("WARNING: SPI returning zero - possible communication issues\r\n");
        while(1) {
         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
         HAL_Delay(1000);
        }
    }
}
