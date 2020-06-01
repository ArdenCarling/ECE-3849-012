/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 * Arden Carling    4/2020
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "driverlib/interrupt.h"
#include "ADC1.h"
#include "buttons.h"
#include "fifo.h"
#include "Crystalfontz128x128_ST7735.h"
#include "HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.h"
#include "grlib/grlib.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"


#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"

#include <math.h>

#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))
bool kissMode = 0;

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define VIN_RANGE 3.3
#define PIXELS_PER_DIV 20
#define ADC_BITS 12
#define ADC_BUFFER_WRAP(i) ((i) & (2048 - 1)) // index wrapping macro
/*
 *  ======== main ========
 */
uint16_t xpos = 0;
bool trig = 0;
bool box;
char* LocButtons;
uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint16_t edge_pos = 64;
volatile uint16_t BuffBuff[128], DispBuff1[128], DispBuff2[128], out_db[128], Bufft[NFFT]; //assorted buffers
uint16_t p, r, y2, y, x, g, x_pos, Vset=3, fps; //p stands for pixel, g for graph-line, r for render, Vset sets v range
uint32_t frameC=0, ms5C=0; //64 bits so you can measure fps for millions of years
float fVoltsPerDiv = 1; //volts per div
float fScale; //diplay scaling
//string buffers
char str[25], str2[25], str3[25], str4[25], str5[25];
uint8_t Grid[7] = {4, 24, 44, 64, 84, 104, 124}; //

const char * const gVoltageScaleStr[] = {
 "100 mV // 20 us", "200 mV // 20 us", "500 mV // 20 us", " 1 V // 20 us", " 2 V // 20 us"
};

const float Vmodes[] = {
                       .1,.2,.5,1,2
};

tContext sContext;
tRectangle rectFullScreen;


int main(void)
{
    IntMasterDisable();

    // hardware initialization goes here
    LocButtons = (char*) malloc(sizeof(char)); //Copy of the button mailbox must have a static address

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_RIGHT); // set screen orientation

    ButtonInit();


    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    // configure M0PWM3, at GPIO PF3, BoosterPack 1 header C1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3,
     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    ADCInit();
    /* Start BIOS */
    BIOS_start();

    return (0);
}

void enableInt(void) //enables interrupts, posts a semaphore for wavCpy and then waits forever
{

    while(1){
        IntMasterEnable();
        Semaphore_post(copyTrig);
        Semaphore_pend(IntSema, BIOS_WAIT_FOREVER);
    }
}

void wavCpy(void) //copies global buffer for other tasks, runs trigger search, posts for data management
{
    while(1){
        Semaphore_pend(copyTrig, BIOS_WAIT_FOREVER);
        if(!kissMode){
            if(trig){
                x_pos = FallingTrigger();
            }
            else{
                x_pos = RisingTrigger();
            }
            for(p=0; p<128; p++){
                BuffBuff[p] = gADCBuffer[ADC_BUFFER_WRAP(p + x_pos - 64)];
            }}//Osc. mode ends
        else{
            int index = gADCBufferIndex;
            for(p=0; p<1024; p++){
                Bufft[p] = gADCBuffer[ADC_BUFFER_WRAP(p + index - 1024)];
            }
        }

        Semaphore_post(mgmTrig);
    }
}

void datMgm(void) //Performs fft calculations, loads display buffers with data
{
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT


    while(1){
        Semaphore_pend(mgmTrig, BIOS_WAIT_FOREVER);

        if(!kissMode){
        fVoltsPerDiv = Vmodes[Vset];
        fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv);

        for(x=0; x<127; x++){
            DispBuff1[x] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * (float)(BuffBuff[x] - 2048));
            DispBuff2[x] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * (float)(BuffBuff[x+1] - 2048));
        }} //end of Ocill. mode calculations
        else{
            for (i = 0; i < NFFT; i++) { // generate an input waveform
             in[i].r = Bufft[i]; // real part of waveform
             in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            for(i=0; i<128; i++){// convert first 128 bins of out[] to dB for display
                out_db[i] = 10 * log10f(out[i].r * out[i].r + out[i].i * out[i].i);
            } //end of fft calculations
        }
        Semaphore_post(dispTrig);
    }
}

void datDisp(void) //All graphics calls go here
{
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    while(1){
        Semaphore_pend(dispTrig, BIOS_WAIT_FOREVER);
        frameC++;
        fps = (int)((float)frameC / ((float)ms5C / 200));
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrBlue);
        for(g=0; g<7; g++){//gridlines
            GrLineDrawH(&sContext, 0, 128, Grid[g]);//gridlines
            GrLineDrawV(&sContext, Grid[g], 0, 128);
        }
        GrContextForegroundSet(&sContext, ClrRed);//center line red
        GrLineDrawV(&sContext, 64, 0, 128);
        if(trig){
            GrLineDraw(&sContext, 100, 0, 105, 10);//trigger direction indicator
            GrLineDraw(&sContext, 105, 10, 110, 0);
        }
        else{
            GrLineDraw(&sContext, 100, 10, 105, 0);//trigger direction indicator
            GrLineDraw(&sContext, 105, 0, 110, 10);
        }
        GrContextForegroundSet(&sContext, ClrWhite);

        if(!kissMode){
        for(r=0; r<128; r++){
            GrLineDrawV(&sContext, r, DispBuff1[r], DispBuff2[r]);//draws the waveform
        }
        snprintf(str, sizeof(str), gVoltageScaleStr[Vset]); //Vscale display
        snprintf(str2, sizeof(str2), "Errors %u", gADCErrors); //number of missed deadlines
        snprintf(str3, sizeof(str3), "FPS %u", fps); //pcmr lol

        GrStringDraw(&sContext, str, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);
        GrStringDraw(&sContext, str2, /*length*/ -1, /*x*/ 0, /*y*/ 10, /*opaque*/ false);
        GrStringDraw(&sContext, str3, /*length*/ -1, /*x*/ 0, /*y*/ 118, /*opaque*/ false);
        }
        else{
            for(r=0; r<127; r++){
                GrLineDrawV(&sContext, r, 196-out_db[r], 196-out_db[r+1]); //It seems that I flipped my waveform upside down somewhere, this unflips it
            }
        }



        GrFlush(&sContext);
        Semaphore_post(copyTrig);
    }
}

void usrInput(void){ //handles user inpout
    while(1){
        Mailbox_pend(ButtonBox, LocButtons, BIOS_WAIT_FOREVER);
        if(*LocButtons == 16){//joystick press of trig direction
            trig ^= 1;
        }

        if(*LocButtons == 8){//S2 to go up a vscale
            Vset = (Vset+1)%5;
        }
        else if(*LocButtons == 4){//S1 to go down a vscale
            Vset = (Vset+4)%5;
        }
        else if(*LocButtons == 32){//stick down (toward screen for fft)
            kissMode ^= 1; //turns fft on/off
        }
        *LocButtons = 0; //resets copy of button presses, just for safety

    }
}

void InputSignal(void){
    ms5C++; //counts number of clocks for average fps counter
    Semaphore_post(ScanButton); //triggers ButtonISR (not an isr anymore)
}


