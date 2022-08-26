/*
 * File:   adc.h
 * Author: user
 *
 * Created on 2020年5月5日, 上午 9:41
 */

#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    AN3,
    AN0,
    AN1,
    AN2
};

void ADC1Initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
