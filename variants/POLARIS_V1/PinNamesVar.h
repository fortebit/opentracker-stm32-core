    /* SYS_WKUP */
#ifdef PWR_WAKEUP_PIN1
    SYS_WKUP1 = PA_0,
#endif
#ifdef PWR_WAKEUP_PIN2
    SYS_WKUP2 = PC_13,
#endif
#ifdef PWR_WAKEUP_PIN3
    SYS_WKUP3 = PE_6, /* SOS Button */
#endif
#ifdef PWR_WAKEUP_PIN4
    SYS_WKUP4 = PA_2,
#endif
#ifdef PWR_WAKEUP_PIN5
    SYS_WKUP5 = PC_5, /* Ignition Detect */
#endif
#ifdef PWR_WAKEUP_PIN6
    SYS_WKUP6 = NC,
#endif
#ifdef PWR_WAKEUP_PIN7
    SYS_WKUP7 = NC,
#endif
#ifdef PWR_WAKEUP_PIN8
    SYS_WKUP8 = NC,
#endif

#ifdef ADC_CHANNEL_0
    ADC_CH0 = 0xA0,
#endif
#ifdef ADC_CHANNEL_VBAT
    ADC_VBAT = 0xA1,
#endif
#ifdef ADC_CHANNEL_VREFINT
    ADC_VREF = 0xA2,
#endif
#ifdef ADC_CHANNEL_TEMPSENSOR
    ADC_VTEMP = 0xA3,
#endif
