#ifndef INTERRUPT_H
#define INTERRUPT_H
#define MSGSIZE 150

#include <stdio.h>
#include <string.h>
#include <iostream>

char carat;
char buffer[MSGSIZE];
char bufferRX[MSGSIZE];
char msg[] = "$PSTMPV";
bool firstTime = true;
uint8_t cnt = 0, cntRX;
int8_t res;


namespace KF {
    template<typename T1, typename T2>
    void Rx_interrupt(T1* led, T2* gnss, bool rcvdMsg){
        led = 1;
        if (firstTime) {
            memset(buffer,0,MSGSIZE);
            firstTime = false;
            }

        carat = gnss.getc();
        if(carat!=10) {
            buffer[cnt++] = carat;
        } else {
            buffer[cnt++] = carat;
            res = memcmp(buffer,msg,sizeof(msg));
        }
        if(res==1) {
            rcvdMsg = true;
            memcpy(bufferRX, buffer, cnt+1);
            cntRX = cnt+1;
            cnt = 0;
            res = 0;
        }
        led = 0;
        return;
    }
}


#endif