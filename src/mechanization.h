#ifndef MECHANIZATION_H
#define MECHANIZATION_H

#define d2r 0.0174
#define MSGSIZE 150

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <array>
#include "src/utils.h"

auto EYE = kfm::eye3(1.0f);
kfm::mechout MechRes;

namespace KF{
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
}












#endif