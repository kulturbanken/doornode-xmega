#ifndef ADC_H
#define ADC_H

void adc_init();
int16_t adc_get_cpu_temp();
uint16_t adc_read(uint8_t channel);

#endif/*ADC_H*/
