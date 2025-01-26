/*
The execution file for Kalman filter.
Last update: 12 Jan 2024
*/

#include "mbed.h"
#include "XNucleoIKS01A2.h"
#include <stdio.h>
#include <string.h>
#include <array>
#include <cstdint>
#include "src/utils.h"
#include "src/interrupt.h"
#include "src/kalmanfilter.h"



#define d2r 0.0174
#define G 9.81

DigitalOut rst(D7);                 // reset pin
DigitalOut wkp(D13);                // wake up to wake from sleep mode
DigitalIn pps(D6);                  // internal timer pulse per second blinking red light on the board
Serial gnss(PA_9,PA_10,230400);     // GNSS connection
Serial pc(USBTX,USBRX,921600);      // Serrial communication
DigitalIn btn(USER_BUTTON);         // blue button
DigitalOut led(LED1);               // LED


Timer tim;                          // SAMPLING PERIOD
bool init = true;
std::array<float, 9> STATE ={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                                    // Type should be defined
kfm::mechout DELTA;
const float ST = 0.4;                     // Sampleing time
                                    // Sensor board object --> gyroscope object
static XNucleoIKS01A2 *XNA2 = XNucleoIKS01A2::instance(D14,D15,D4,D5);
static LSM6DSLSensor *acc_gyro = XNA2->acc_gyro;

int16_t xRaw[3],gRaw[3];            // for raw values of gyroscope
float sX,sG;                        // sensetivity

bool rcvdMsg = false;
std::array<float, 6> GPSmeas;
// -------------------------------------- Q definition ------------------
float val1 = std::pow(((0.09*d2r/60)),2);
float val2 = std::pow((0.008/60),2);
float val3 = ((3.2e-6*G)/std::sqrt(100))*2;
float val4 = std::pow(((0.8*d2r/3600)/std::sqrt(300)),2);
const std::array<std::array<float, 15>, 15> Q= {{{val1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, val1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, val1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, val2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, val2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val3, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val3, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val4, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val4, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, val4}}};

// --------------------------------------- H ---------------------------------------------
const std::array<std::array<float, 15>, 6> H = {{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                           {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                           {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                           {0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                           {0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                           {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}};

// -------------------------------------- R ----------------------------------------------
const std::array<std::array<float, 6>, 6> R = {{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}};
// -------------------------------------- P initialization --------------------------------
auto P = kfm::eye15(0.0001f);

std::array<float, 15> dX{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f};

kfm::Params Params{
    6378137,           // R0: Earth average radius
    0.08182,           // eccentricity
    7.292e-5,          // wie
    9.78033,           // gam
    0.001932           // kgamma
    };

int main(){
    pc.puts("Enabling accelerometer and gyroscope streams ...");
    acc_gyro->enable_x();               // Enable accelerometer data stream
    acc_gyro->enable_g();               // Enable gyroscope data stream
    pc.puts("Getting the sensitivity of accelerometer and gyroscope streams ...");
    acc_gyro->get_x_sensitivity(&sX);   // Sensitivity constant of streams
    acc_gyro->get_x_sensitivity(&sG);
    rst.write(1);                       // reset microcontroller
    
    pc.puts("Waiting you for press key k on your keyboard ...");
    while (pc.getc()!='k');             // waiting for a command ('k') from user or host
    
    //tim.start();                        // Timer for:                      // Timer for:

    while(!pps);                         
    while(pps);
    while(!pps);
                                        // Attached the gnss Serial object with a callback to data receiver
    gnss.attach(&KF::Rx_interrupt(led, gnss, rcvdMsg), Serial::RxIrq);


    while(1){

        if (init){
            pc.putc("Waiting for initial state");
            if(rcvdMsg) {
                KF::StateInit(STATE, KF::NMEA_parse(bufferRX));
                init = false;
                }
            rcvdMsg = false;
            continue;
            }

        //tim.reset();
        acc_gyro->get_x_axes_raw(xRaw);
        acc_gyro->get_g_axes_raw(gRaw);

        std::array<float,3> XCC = kfm::scale_g(xRaw, sX*G);
        std::array<float,3> GCC = kfm::scale_g(gRaw, sG*d2r);

        kfm::vec3_add(XCC, std::array<float,3>{dX[9], dX[10], dX[11]});
        kfm::vec3_add(GCC, std::array<float,3>{dX[12], dX[13], dX[14]});

        // Prediction step: StateEvol & CovEvol
        DELTA = KF::StateEvol(STATE, GCC, XCC, Params, ST);
        KF::CovEvol(DELTA, STATE, XCC, dX, P, Q, Params, ST);

        // Correction using GPS
        if(rcvdMsg) {
            GPSmeas = KF::NMEA_parse(bufferRX);
            KF::StateCorr(STATE, GPSmeas, DELTA.cbn, dX, H, P, R);
            rcvdMsg = false;
            }

        // Sending data to serial terminal
        for (const float& value : STATE) {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
            for (int i = 0; i < sizeof(float); ++i) {
                pc.putc(bytes[i]);
            }
        }

        //double time = tim.read();
        //if(time<ST) {                     
        //    wait(ST-time);
        //    }
    }
    return 0;
}