#ifndef _BME280_H_
#define _BME280_H_

typedef struct
{
  uint32_t task_idx;
  i2c_port_t i2c_port_num;
} bme280_task_param_t;

extern void bme280_task(void *arg);

#endif //_BME280_H_