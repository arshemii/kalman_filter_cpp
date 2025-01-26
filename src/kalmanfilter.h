#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#define d2r 0.0174
#define MSGSIZE 150

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <array>
#include <vector>
#include "src/utils.h"


auto EYE = kfm::eye3(1.0f);
kfm::mechout MechRes;

namespace KF{

    std::array<float, 6> NMEA_parse(const char* buffer) {
        std::array<float, 6> GPS_meas = {0};

        float lat = 0.0f, lon = 0.0f, alt = 0.0f;
        float vn = 0.0f, ve = 0.0f, vd = 0.0f;

        int fieldIndex = 0;
        const char* ptr = buffer;

        while (*ptr) {
            if (*ptr == ',') {
                fieldIndex++;
                ptr++;
                continue;
            }

            if (fieldIndex == 2) {
                lat = strtof(ptr, (char**)&ptr);
            }
            else if (fieldIndex == 4) {
                lon = strtof(ptr, (char**)&ptr);
            }
            else if (fieldIndex == 6) {
                alt = strtof(ptr, (char**)&ptr);
            }
            else if (fieldIndex == 8) {
                vn = strtof(ptr, (char**)&ptr);
            }
            else if (fieldIndex == 9) {
                ve = strtof(ptr, (char**)&ptr);
            }
            else if (fieldIndex == 10) {
                vd = strtof(ptr, (char**)&ptr);
            }
            else {
                ptr++;
            }
        }

        GPS_meas[3] = lat;
        GPS_meas[4] = lon;
        GPS_meas[5] = alt;
        GPS_meas[0] = vn;
        GPS_meas[1] = ve;
        GPS_meas[2] = vd;

        return GPS_meas;
    }

    void StateInit(std::array<float, 9>& state, const std::array<float, 6>& GPS){
        for (size_t i = 0; i<9; ++i){
            if (i<3){
                state[i] = 0.0f;
            } else{
                state[i] = GPS[i-3];
            }
        }
    }

    kfm::mechout StateEvol(std::array<float, 9>& STATE,
                        const std::array<float, 3>& GCC,
                        const std::array<float, 3>& XCC,
                        const kfm::Params& Params,
                        const float& ST){
            

            // ----------- State evolution ------------------------
            // This function predict the state by means of mechanization and frame transformation
            // System Model


                        // STATE[0], STATE[1], STATE[2]: Heading, Pitch, Roll
                        // STATE[3], STATE[4], STATE[5]: Velocity
                        // STATE[6], STATE[7], STATE[8]: Lat, Lon, Alt

            std::array<std::array<float, 3>, 3> Cbn = kfm::Eul2Mat(STATE[0], STATE[1], STATE[2], "zyx");
            std::array<std::array<float, 3>, 3> Omegaibb = kfm::RotVec2Mat(GCC);

            std::array<float, 3> wien = {cos(STATE[6])*Params.wie, 0, -sin(STATE[6])*Params.wie};

            std::array<std::array<float, 3>, 3> Omegaien = kfm::RotVec2Mat(wien);

            float Rn = (Params.R0 * (1.0 - pow(Params.eccentricity, 2))) / 
                        pow(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2), 1.5);
            float Re = Params.R0 / sqrt(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2));

            float pos0_dot = STATE[3] / (Rn + STATE[8]);
            float pos1_dot = STATE[4] / (cos(STATE[6]) * (Re + STATE[8]));

            std::array<float, 3> wenn = {cos(STATE[6])*pos1_dot, -pos0_dot, -sin(STATE[6])*pos1_dot};

            std::array<std::array<float, 3>, 3> Omegaenn = kfm::RotVec2Mat(wenn);

            Cbn = kfm::mat3_mat3_mul(Cbn, kfm::mat_mat_add3(EYE, kfm::sc_mat3_mul(Omegaibb, ST))) -
                        kfm::sc_mat3_mul(kfm::mat3_mat3_mul(kfm::mat_mat_add3(Omegaien, Omegaenn), Cbn) ,ST);
                        
            // Specific force
            std::array<float, 3> fibn = kfm::mat3_vec3_mul(Cbn, XCC);

            float gamma0 = Params.gam * (1.0 + (Params.kgamma * pow(sin(STATE[6]), 2))) /
                            sqrt(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2)); // m/s2

            float gamma = gamma0 * Rn * Re /
                            pow(sqrt(Rn + Re) + STATE[8], 2);

            std::array<float, 3> gn = {static_cast<float>(sin(STATE[6]) * cos(STATE[6]) * pow(Params.wie, 2) * (STATE[8] + sqrt(Rn*Re))),
                                        0,
                                        static_cast<float>(gamma - pow(Params.wie * cos(STATE[6]), 2) * (STATE[8] + sqrt(Rn*Re)))};


            // Velocity update
            std::vector <float> v_fact = kfm::mat3_vec3_mul(kfm::mat_mat_add3(Omegaenn, kfm::sc_mat3_mul(Omegaien, 2.0f)),
                                                            std::array<float, 3>{STATE[3], STATE[4], STATE[5]});


            STATE[3] = STATE[3] + ST * (fibn[0] + gn[0] - v_fact[0]);
            STATE[4] = STATE[4] + ST * (fibn[1] + gn[1] - v_fact[1]);  
            STATE[5] = STATE[5] + ST * (fibn[2] + gn[2] - v_fact[2]);

            // Position update
            STATE[6] -= ST*STATE[5];
            STATE[7] += ST*STATE[3]/(Rn+STATE[8]);

            Re = Params.R0 / sqrt(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2));

            STATE[8] = STATE[8] + ST*STATE[4]/(cos(STATE[6]) * (Rn + STATE[8]));
            std::array<float, 3> eul_up = kfm::Mat2Eul(Cbn, "zyx");
            
            STATE[0] = eul_up[0];
            STATE[1] = eul_up[1];
            STATE[2] = eul_up[2];

            MechRes.cbn = Cbn;
            MechRes.re = Re;
            MechRes.fibn = fibn;
            MechRes.state = STATE;

            return MechRes;
    }

    void CovEvol(kfm::mechout& DELTA,
                        std::array<float, 9>& STATE,
                        const std::array<float, 3>& XCC,
                        std::array<float, 15>& dX,
                        std::array<std::array<float, 15>, 15>& P,
                        const std::array<std::array<float, 15>, 15>& Q,
                        const kfm::Params& Params,
                        const float& ST){
            /*
            ---------------------------- Covariance evolution ----------------------------
            This function update the covariance matrix based on the new predicted state
            ------------------------------------------------------------------------------
            Guide

            struct mechout {
                float re;
                std::array<float, 9> state;
                std::array<std::array<float,3>,3> cbn;
                std::array<float, 3> fibn;
            };
            */


            float Rn = (Params.R0 * (1.0 - pow(Params.eccentricity, 2))) / 
                        pow(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2), 1.5);
            // float Re = Params.R0 / sqrt(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2));
            float gamma0 = Params.gam * (1.0 + (Params.kgamma * pow(sin(STATE[6]), 2))) /
                            sqrt(1.0 - pow(Params.eccentricity * sin(STATE[6]), 2)); // m/s2

            float ReSe = DELTA.re * std::sqrt(std::cos(STATE[6])*std::cos(STATE[6]) + std::pow(1 - std::pow(Params.eccentricity, 2), 2) * std::sin(STATE[6])*std::sin(STATE[6]));

            // ------------------------------------F ---------------------

            kfm::sc_vec3_mul(DELTA.fibn, -1.0f);
            std::array<std::array<float, 3>, 3> Faux = kfm::RotVec2Mat(DELTA.fibn);
            std::array<std::array<float, 15>, 15>  F = kfm::zero15();
            F[6][3] = ST/(Rn + STATE[8]);
            F[7][4] = ST/(DELTA.re + STATE[8]*cos(STATE[6]));
            F[8][5] = -1.0*ST;
            F[5][8] = -2.0 * ST * gamma0/ReSe;

            F[3][0] = Faux[0][0]* ST;
            F[3][1] = Faux[0][1]* ST;
            F[3][2] = Faux[0][2]* ST;
            F[4][0] = Faux[1][0]* ST;
            F[4][1] = Faux[1][1]* ST;
            F[4][2] = Faux[1][2]* ST;
            F[5][0] = Faux[2][0]* ST;
            F[5][1] = Faux[2][1]* ST;
            F[5][2] = Faux[2][2]* ST;

            F[0][12] = DELTA.cbn[0][0]* ST;
            F[1][12] = DELTA.cbn[1][0]* ST;
            F[2][12] = DELTA.cbn[2][0]* ST;
            F[0][13] = DELTA.cbn[0][1]* ST;
            F[1][13] = DELTA.cbn[1][1]* ST;
            F[2][13] = DELTA.cbn[2][1]* ST;
            F[0][14] = DELTA.cbn[0][2]* ST;
            F[1][14] = DELTA.cbn[1][2]* ST;
            F[2][14] = DELTA.cbn[2][2]* ST;

            F[3][9] = DELTA.cbn[0][0]* ST;
            F[4][9] = DELTA.cbn[1][0]* ST;
            F[5][9] = DELTA.cbn[2][0]* ST;
            F[3][10] = DELTA.cbn[0][1]* ST;
            F[4][10] = DELTA.cbn[1][1]* ST;
            F[5][10] = DELTA.cbn[2][1]* ST;
            F[3][11] = DELTA.cbn[0][2]* ST;
            F[4][11] = DELTA.cbn[1][2]* ST;
            F[5][11] = DELTA.cbn[2][2]* ST;

            // --------------------------------G --------------------------
            std::array<std::array<float, 15>, 15> G = kfm::eye15(1.0f);
            G[3][0] = DELTA.cbn[0][0];
            G[4][0] = DELTA.cbn[1][0];
            G[5][0] = DELTA.cbn[2][0];
            G[3][1] = DELTA.cbn[0][1];
            G[4][1] = DELTA.cbn[1][1];
            G[5][1] = DELTA.cbn[2][1];
            G[3][2] = DELTA.cbn[0][2];
            G[4][2] = DELTA.cbn[1][2];
            G[5][2] = DELTA.cbn[2][2];

            G[6][3] = -DELTA.cbn[0][0];
            G[7][3] = -DELTA.cbn[1][0];
            G[8][3] = -DELTA.cbn[2][0];
            G[6][4] = -DELTA.cbn[0][1];
            G[7][4] = -DELTA.cbn[1][1];
            G[8][4] = -DELTA.cbn[2][1];
            G[6][5] = -DELTA.cbn[0][2];
            G[7][5] = -DELTA.cbn[1][2];
            G[8][5] = -DELTA.cbn[2][2];

            // -----------------------------------------------------
            kfm::mat_mat_add15_inplace(F, kfm::eye15(1.0f));
            dX = kfm::mat15_vec15_mul(F, dX);

            P = kfm::mat15_mat15_mul(F, kfm::mat15_mat15_mul(P, kfm::mat15_T(F))) +
                kfm::sc_mat15_mul(kfm::mat15_mat15_mul(G, kfm::mat15_mat15_mul(Q, kfm::mat15_T(G))), ST);
    }

    


    void StateCorr(std::array<float, 9>& STATE,
                    std::array<float, 6>& GPSmeas,
                    std::array<std::array<float,3>,3>& cbn,
                    std::array<float, 15>& dX,
                    const std::array<std::array<float, 15>, 6>& H,
                    std::array<std::array<float, 15>, 15>& P,
                    const std::array<std::array<float, 6>, 6>& R){

            std::array<std::array<float, 6>,6> ResErr = kfm::Inv6(kfm::mat_mat_add6(kfm::mat6_15_mat15_6_mul(kfm::mat6_15_mat15_mul(H, P), kfm::mat6_15_T(H)), R));
            if (ResErr[3][3] == 0.1234){
                return;
            }

            std::array<std::array<float, 6>, 15> K = kfm::mat15_6_mat_6_mul(kfm::mat15_mat15_6_mul(P, kfm::mat6_15_T(H)), ResErr);

            P = kfm::mat15_mat15_mul(kfm::mat_mat_add15(kfm::eye15(1.0f), kfm::mat15_6_mat_6_15_mul(K, H)), P);
            P = kfm::sc_mat15_mul(kfm::mat_mat_add15(P, kfm::mat15_T(P)), 0.5f);

            for (size_t i = 0; i<6; ++i){
                GPSmeas[i] -= STATE[i+3];
            }

            dX = dX - kfm::mat15_6_vec6_mult(K, GPSmeas);
            /*

            dX(1:9) = zeros(9,1);
            */
            kfm::mat3_mat3_mul_inplace(cbn, kfm::mat_mat_add3(kfm::eye3(1.0f), kfm::RotVec2Mat(std::array<float, 3>{dX[0], dX[1], dX[2]}))); // cbn update

            for (size_t i=3; i<9; ++i){
                STATE[i] += dX[i];
            }

            kfm::dcm2angle(STATE, cbn);

            // zering out dX state variables
            for (size_t i=0; i<9; ++i){
                dX[i] = 0.0f;
            }
    }

}





#endif

