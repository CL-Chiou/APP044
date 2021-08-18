#ifndef INTERRUPTHANDLE_H
#define	INTERRUPTHANDLE_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    void ADC1_AD1SetIntHandler(void *handler);
    void ADC1_AD1DefInterruptHandler(void);
    
    void ADC1_DMA2SetIntHandler(void *handler);
    void ADC1_DMA2DefInterruptHandler(void);
    
    void SPI1_SetIntHandler(void *handler);
    void SPI1_DefInterruptHandler(void);
    
    void Timer1_SetIntHandler(void *handler);
    void Timer1_DefInterruptHandler(void);
    
    void Timer3_SetIntHandler(void *handler);
    void Timer3_DefInterruptHandler(void);
    
    void UART1_U1SetIntHandler(void *handler);
    void UART1_U1DefInterruptHandler(void);
    
    void UART1_DMA0SetIntHandler(void *handler);
    void UART1_DMA0DefInterruptHandler(void);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* INTERRUPTHANDLE_H */

