/*
 * SlamTec Base Ref Design
 * Copyright 2009 - 2017 RoboPeak
 * Copyright 2013 - 2017 Shanghai SlamTec Co., Ltd.
 * http://www.slamtec.com
 * All rights reserved.
 */
/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 @breif PCB REF REV 3.0
 *   
 *    I/O     DEFINITION                   I/O    DEFINITION                 I/O     DEFINITION                    I/O    DEFINITION              I/O     DEFINITION
 *
 *    PA0     CHARGE_PWM(ADC123_IN0)       PB0                               PC0                                   PD0                            PE0 
 *    PA1                                  PB1                               PC1     BOTTOM_IR_R2(ADC123_IN11)     PD1    GROUND_DETECT_L(GPIO)   PE1 
 *    PA2     DISPLAY_RX(USART2_TX)        PB2                               PC2     BOTTOM_IR_R1(ADC123_IN12)     PD2    ENCODER_SENSOR_R(EXTI)  PE2     BATT_READY(GPIO)      
 *    PA3     DISPLAY_TX(USART2_RX)        PB3                               PC3     BATT_MONITOR(ADC123_IN13)     PD3    ENCODER_SENSOR_L(EXTI)  PE3     BATT_CHRG(GPIO)     
 *    PA4     BOTTOM_IR_R4(ADC12_IN4)      PB4                               PC4     BOTTOM_IR_R3(ADC12_IN14)      PD4    MOTO_LF_EN(GPIO)        PE4     BATT_FAULT(GPIO)        
 *    PA5                                  PB5    BUMP_DETECT_L(GPIO)        PC5     MOTO_RI_MONITOR(GPIO)         PD5    MOTO_LI_MONITOR(GPIO)   PE5     SONAR_ECHO1(GPIO)      
 *    PA6     BATT_DETECT(ADC12_IN6)       PB6                               PC6                                   PD6    MOTO_RF_EN(GPIO)        PE6     
 *    PA7                                  PB7                               PC7     BOTTOM_IR_E(BASIC TIMER)      PD7    MOTO_RB_EN(GPIO)        PE7     SONAR_ECHO2
 *    PA8                                  PB8    HOCHARGE_DETECT(GPIO)      PC8                                   PD8                            PE8     SONAR_ECHO3
 *    PA9     PCIE_CRX(USART1_TX)          PB9    DCCHARGE_DETECT(GPIO)      PC9     PCIE_nCCMD(GPIO)              PD9    MOTO_LB_EN(GPIO)        PE9     SONAR_ECHO4
 *    PA10    PCIE_CTX(USART1_RX)          PB10                              PC10    PC10_TX(USART3_TX)            PD10   GROUND_DETECT_R(GPIO)   PE10    SONAR_TRIG1
 *    PA11                                 PB11                              PC11    PC11_RX(USART3_RX)            PD11                           PE11    SONAR_TRIG2
 *    PA12    PCIE_CBUSY(GPIO)             PB12   LED_WS2812(GPIO)           PC12                                  PD12   HOME_IR_R1(TIM4_CH1)    PE12    SONAR_TRIG3
 *    PA13    PROGRAMMER(SWDIO)            PB13   BUMP_DETECT_R(GPIO)        PC13                                  PD13   HOME_IR_R2(TIM4_CH2)    PE13    MOTO_R_PWM(TIM1_CH3)
 *    PA14    PROGRAMMER(SWCLK)            PB14                              PC14                                  PD14   HOME_IR_R3(TIM4_CH3)    PE14    MOTO_L_PWM(TIM1_CH4)
 *    PA15    BEEP_PWM(TIM2_CH1_ETR)       PB15   CURRENT_SET(GPIO)          PC15                                  PD15                           PE15    SONAR_TRIG4
 *
 * Change since PCB REV 1.0
 *    PE5, PE7 ~ PE12, PE15 as SONAR interface
 *    MOTO_L_PWM move to PE14
 *    BOTTOM_IR_R1 move to PC2
 *    PC10, PC11 as UART
 *    Remove FRONT_IR
 *    PB12 as color LED WS2812 control.
 */
 
#include "common/common.h"
#include "drv/serial_channel.h"
#include "drv/time.h"
#include "drv/beep.h"
#include "drv/battery.h"
#include "drv/motor.h"
#include "drv/bump.h"
#include "drv/distir.h"
#include "drv/homeir.h"
#include "drv/watchdog.h"
#include "drv/sonar.h"
#include "drv/led.h"
#include "drv/drv_ctrlbus.h"
#include "drv/health_monitor.h"
#include "bump_monitor.h"
/*
 * Low battery alarm health monitor callback function
 */
static _u8 battery_low_cb(void)
{
    _u8 percent = get_electricitypercentage();

    if (percent < 10) {
        return (SLAMWARECORECB_HEALTH_FLAG_ERROR);
    } else if (percent < 30) {
        return (SLAMWARECORECB_HEALTH_FLAG_WARN);
    }
    return (SLAMWARECORECB_HEALTH_FLAG_OK);
}
/*
* Initialize board-level peripheral functions
*/
static _s32 init_dev(void)
{
#ifdef _DEBUG
    usart_begin(GET_USART(DBG_USART_ID), 115200);                       //Initialize debug serial port 2
#endif
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    drv_led_init();
#endif
    drv_serialchannel_init(GET_USART(USART_CTRLBUS_ID), 115200);        //Initialize the serial port that communicates with slamcore 1
    net_bind(drv_serialchannel_getchannel());                           //Bind the serial port to the interchip protocol
    init_battery();                                                     //Initialize battery power, charging related
    init_beep();                                                        //Initialize the buzzer for various sound prompts
    init_drv_ctrlbus();                                                 //Initialize ctrlbus relate
    init_distir();                                                      //Initialize ctrlbus relate
    init_homeir();                                                      //Initial IR ranging related, including 3 Homeir
    init_brushmotor();                                                  //
    init_walkingmotor();                                                //Initialize the two-way walking motors, the output speed resolution is -1000 ~ 1000
    init_walkingmotor_odometer();                                       //Initialize the encoders on the two-way walking motors and interrupt the input
    set_walkingmotor_speed(0, 0);                                       //Speed ​​setting Left: 0mm/s Right: 0mm/s
    init_ontheground_detect();                                          //Initialize the detection foot whether it is on the ground, which is used to judge whether the machine is on the ground or overhead
    init_bump_detect();                                                 //Initialize the collision detection foot
    init_bumpermonitor();
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    init_sonar();
#endif
    health_monitor_init();

    /* Register the low battery alarm callback function. */
    health_monitor_register(BASE_POWER_LOW(NONE),
                            "Low battery.",
                            battery_low_cb);
    return 1;
}

_u32 shutdownHeartbeatFrequency = 0;
/*
 *  Connect with slamcore to maintain decision function
 */
void shutdown_heartbeat(void)
{
    if ((getms() - shutdownHeartbeatFrequency) > 1000)
    {
        set_walkingmotor_speed(0, 0);                           //Disconnect from slamcore and stop walking
    }
}
/*
 *  Close device function
 * Some peripherals can be turned off here, depending on the situation
 */
static void dev_shutdown(void)
{
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    drv_led_shutdown();
#endif
    drv_serialchannel_shutdown(GET_USART(DATAOUT_USART_ID));
    shutdown_bumpermonitor();
}
/*
 * Module loop processing function
 */
static void dev_heartbeat(void)
{
    heartbeat_battery();
    heartbeat_bumpermonitor();
    heartbeat_beep();
    heartbeat_distir();
    heartbeat_homeir();

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    heartbeat_sonar();
#endif
    shutdown_heartbeat();
    speedctl_heartbeat();
    health_monitor_heartbeat();
}
/*
 * Failure mode handling function
 */
static void on_abort_mode(void)                                 //Fault mode processing, here you can shut down, shut down and interrupt, etc., depending on the specific situation
{
    dev_shutdown();
    cli();
    while (1);
}

extern void on_host_request(infra_channel_desc_t * channel);

/*
 * Main loop function
 */
static inline _s32 loop(void)
{

    if (net_poll_request(drv_serialchannel_getchannel())) {    //Listen to interchip protocol messages from slamcore

        on_host_request(drv_serialchannel_getchannel());        //Respond to the interchip protocol message from slamcore
    }
    dev_heartbeat();                                           //Process each functional module
    return 1;
}
/*
 * Main program function
 */
int main(void)
{
    board_set_abort_proc(on_abort_mode);                //Set fault handling function
    _delay_ms(100);                                     //Wait for the power supply to stabilize

    init_board();                                       //MCU low-level initialization
    if (!init_dev()) {                                  //Initialize all peripherals
        goto _on_fail;
    }

    play_poweron();                                     //Sound at boot
    enable_watchdog();

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    DBG_OUT("Slamware base breakout rev %d.0.\r\n", CONFIG_BREAKOUT_REV);
#endif

    while (loop()) {
        mark_watchdog();
    }

  _on_fail:
    disable_watchdog();
    board_abort_mode();
    return 0;
}
