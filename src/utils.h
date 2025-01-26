#ifndef UTILS_H
#define UTILS_H


#define d2r 0.0174
#define MSGSIZE 150

#include <cstdint>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <array>
#include <vector>
#include <stdexcept>
#include "mbed.h"


namespace kfm {
    // This namespace cannot handle errors
    // Check the namespace before use
    // std::array is used due its efficiency

    // Mechanization output
    struct mechout {
        float re;
        std::array<float, 9> state;
        std::array<std::array<float,3>,3> cbn;
        std::array<float, 3> fibn;
    };


    // constant parameters
    struct Params {
        int32_t R0;
        float eccentricity;
        float wie;
        float gam;
        float kgamma;
        };

    // Zero matrix3
    std::array<std::array<float, 3>, 3> zero3(){
        std::array<std::array<float, 3>, 3> Mat = {{{0.0, 0.0, 0.0},
                                            {0.0, 0.0, 0.0},
                                            {0.0, 0.0, 0.0}}};
        return Mat;
    }

    // Zero matrix15
    std::array<std::array<float, 15>, 15> zero15(){
        std::array<std::array<float, 15>, 15> Mat = {{{0.0f}}};

        return Mat;
    }

    // Scalar* Matrix(3*3) inplace
    template <typename T> 
    void sc_mat3_mul_inplace(std::array<std::array<T, 3>, 3>& mat, const T mul){
        for (auto& row : mat) {
            for (auto& elem : row) {
                elem *= mul;
            }
        }
    }

    // Scalar* Matrix(3*3)
    template <typename T> 
    std::array<std::array<T, 3>, 3> sc_mat3_mul(const std::array<std::array<T, 3>, 3>& mat, const T mul){
        std::array<std::array<T, 3>, 3> result = kfm::zero3();
        for (size_t i = 0; i<3; ++i) {
            for (size_t j = 0; j <3; ++j) {
                result[i][j] = mat[i][j] * mul;
            }
        }
        return result;
    }

    // Scalar* Matrix(15*15) inplace
    template <typename T> 
    void sc_mat15_mul_inplace(std::array<std::array<T, 15>, 15>& mat, const T mul){
        for (auto& row : mat) {
            for (auto& elem : row) {
                elem *= mul;
            }
        }
    }

    // Scalar* Matrix(15*15)
    template <typename T> 
    std::array<std::array<T, 15>, 15> sc_mat15_mul(const std::array<std::array<T, 15>, 15>& mat, const T mul){
        std::array<std::array<T, 15>, 15> result = kfm::zero15();
        for (size_t i = 0; i<15; ++i) {
            for (size_t j = 0; j <15; ++j) {
                result[i][j] = mat[i][j] * mul;
            }
        }
        return result;
    }

    // Scalar + Matrix(3*3)
    template <typename T> 
    void sc_mat3_add(std::array<std::array<T, 3>, 3>& mat, const T add){
        for (auto& row : mat) {
            for (auto& elem : row) {
                elem += add;
            }
        }
    }

    // Scalar + Matrix(15*15)
    template <typename T> 
    void sc_mat15_add(std::array<std::array<T, 15>, 15>& mat, const T add){
        for (auto& row : mat) {
            for (auto& elem : row) {
                elem += add;
            }
        }
    }

    // Matrix + Matrix(3*3)_inplace
    template <typename T> 
    void mat_mat_add3_inplace(std::array<std::array<T, 3>, 3>& mat1,
                    const std::array<std::array<T, 3>, 3>& mat2) {
        // Pay attention to dims of inputs!
        for (size_t i = 0; i < mat1.size(); ++i) {
            for (size_t j = 0; j < mat1[i].size(); ++j) {
                mat1[i][j] += mat2[i][j];
            }
        }
    }

    // Matrix + Matrix(3*3)
    template <typename T> 
    std::array<std::array<float, 3>, 3> mat_mat_add3(const std::array<std::array<T, 3>, 3>& mat1,
                                                    const std::array<std::array<T, 3>, 3>& mat2) {
        // Pay attention to dims of inputs!
        std::array<std::array<float, 3>, 3> add = kfm::zero3();
        for (size_t i = 0; i < mat1.size(); ++i) {
            for (size_t j = 0; j < mat1[i].size(); ++j) {
                add[i][j] = mat1[i][j] + mat2[i][j];
            }
        }
        return add;
    }

    // Matrix + Matrix(15*15)_inplace
    template <typename T> 
    void mat_mat_add15_inplace(std::array<std::array<T, 15>, 15>& mat1,
                    const std::array<std::array<T, 15>, 15>& mat2) {
        // Pay attention to dims of inputs!
        for (size_t i = 0; i < mat1.size(); ++i) {
            for (size_t j = 0; j < mat1[i].size(); ++j) {
                mat1[i][j] += mat2[i][j];
            }
        }
    }

    // Matrix + Matrix(15*15)
    template <typename T> 
    std::array<std::array<float, 15>, 15> mat_mat_add15(const std::array<std::array<T, 15>, 15>& mat1,
                                                        const std::array<std::array<T, 15>, 15>& mat2) {
        // Pay attention to dims of inputs!
        std::array<std::array<float, 15>, 15> add = kfm::zero15();
        for (size_t i = 0; i < mat1.size(); ++i) {
            for (size_t j = 0; j < mat1[i].size(); ++j) {
                add[i][j] = mat1[i][j] + mat2[i][j];
            }
        }
        return add;
    }

    // Matrix + Matrix(6*6)
    template <typename T> 
    std::array<std::array<float, 15>, 15> mat_mat_add6(const std::array<std::array<T, 6>, 6>& mat1,
                                                        const std::array<std::array<T, 6>, 6>& mat2) {
        // Pay attention to dims of inputs!
        std::array<std::array<float, 6>, 6> add = kfm::zero15();
        for (size_t i = 0; i < mat1.size(); ++i) {
            for (size_t j = 0; j < mat1[i].size(); ++j) {
                add[i][j] = mat1[i][j] + mat2[i][j];
            }
        }
        return add;
    }

    // Scalar * Vector(3)
    template <typename T>
    void sc_vec3_mul(std::array<T, 3>& vec,const T mul){
        for (auto& elem : vec) {
            elem *= mul;
        }
    }

    // Scalar * Vector(9)
    template <typename T>
    void sc_vec9_mul(std::array<T, 9>& vec,const T mul){
        for (auto& elem : vec) {
            elem *= mul;
        }
    }

    // Scalar + Vector(3)
    template <typename T>
    void sc_vec3_add(std::array<T, 3>& vec,const T add){
        for (auto& elem : vec) {
            elem += add;
        }
    }

    // Scalar + Vector(9)
    template <typename T>
    void sc_vec9_add(std::array<T, 9>& vec,const T add){
        for (auto& elem : vec) {
            elem += add;
        }
    }

    // Matrix(3,3) * Matrix(3,3)
    template <typename T>
    std::array<std::array<T, 3>, 3> mat3_mat3_mul(const std::array<std::array<T, 3>, 3>& A,
                                                const std::array<std::array<T, 3>, 3>& B) {
        // Pay attention to pass only 3 by 3 matrix
        std::array<std::array<T, 3>, 3> C = kfm::zero3();

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return C;
    }


    // Matrix(3,3) * Matrix(3,3) in-place
    template <typename T>
    void mat3_mat3_mul_inplace(std::array<std::array<T, 3>, 3>& A,
                            const std::array<std::array<T, 3>, 3>& B) {
        // Pay attention to pass only 3 by 3 matrix
        std::array<std::array<T, 3>, 3> C = kfm::zero3();

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        A = C;
    }

    // Matrix(15,15) * Matrix(15,15)
    template <typename T>
    std::array<std::array<T, 15>, 15> mat15_mat15_mul(const std::array<std::array<T, 15>, 15>& A,
                                                const std::array<std::array<T, 15>, 15>& B) {
        // Pay attention to pass only 15 by 15 matrix
        std::array<std::array<T, 15>, 15> C = kfm::zero15();

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return C;
    }


    // Matrix(15,15) * Matrix(15,15) in-place
    template <typename T>
    void mat15_mat15_mul_inplace(std::array<std::array<T, 15>, 15>& A,
                            const std::array<std::array<T, 15>, 15>& B) {
        // Pay attention to pass only 15 by 15 matrix
        std::array<std::array<T, 15>, 15> C = kfm::zero15();

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        A = C;
    }


    // Matrix3 * Matrix3 * Matrix3
    template <typename T>
    std::array<std::array<T, 3>, 3> mat_mat_mat_mul3(std::array<std::array<T, 3>, 3>& A,
                                                     std::array<std::array<T, 3>, 3>& B,
                                                     const std::array<std::array<T, 3>, 3>& C) {
        // Make sure you pass the correct matrix to avoid dimaneion conflicts
        std::array<std::array<T, 3>, 3> D = kfm::zero3();
        kfm::mat3_mat3_mul_inplace(B, C);
        D = kfm::mat3_mat3_mul(A, B);

        return D;
    }

    // Matrix15 * Matrix15 * Matrix15
    template <typename T>
    std::array<std::array<T, 15>, 15> mat_mat_mat_mul15(std::array<std::array<T, 15>, 15>& A,
                                                     const std::array<std::array<T, 15>, 15>& B,
                                                    std::array<std::array<T, 15>, 15>& C) {
        // Make sure you pass the correct matrix to avoid dimaneion conflicts
        std::array<std::array<T, 15>, 15> D = kfm::zero15();
        D = kfm::mat15_mat15_mul(B, C);
        D = kfm::mat15_mat15_mul(A, D);

        return D;
    }

    // Mat(15,15)*mat(15*6)
    std::array<std::array<float, 6>, 15> mat15_mat15_6_mul(const std::array<std::array<float, 15>, 15>& mat1,
                                                        const std::array<std::array<float, 6>, 15>& mat2) {
        std::array<std::array<float, 6>, 15> result = {{{0.0f}}};

        for (size_t i = 0; i < 15; ++i) {
            for (size_t j = 0; j < 6; ++j) {
                float sum = 0.0f;
                for (size_t k = 0; k < 15; ++k) {
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }
        return result;
    }

    // Mat(6,15)*mat(15*15)
    std::array<std::array<float, 15>, 6> mat6_15_mat15_mul(const std::array<std::array<float, 15>, 6>& mat1,
                                                const std::array<std::array<float, 15>, 15>& mat2) {

        std::array<std::array<float, 15>, 6> result = {{{0.0f}}};

        for (size_t i = 0; i < 6; ++i) {
            for (size_t j = 0; j < 15; ++j) {
                float sum = 0.0f;
                for (size_t k = 0; k < 15; ++k) {
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }

        return result;
    }

    // Mat(6,15)*mat(15*6)
    std::array<std::array<float, 6>, 6> mat6_15_mat15_6_mul(const std::array<std::array<float, 15>, 6>& mat1,
                                                            const std::array<std::array<float, 6>, 15>& mat2) {

        std::array<std::array<float, 6>, 6> result = {{{0.0f}}};

        for (size_t i = 0; i < 6; ++i) { 
            for (size_t j = 0; j < 6; ++j) { 
                float sum = 0.0f;
                for (size_t k = 0; k < 15; ++k) { 
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }
        return result;
    }

    std::array<std::array<float, 6>, 15> mat15_6_mat_6_mul(const std::array<std::array<float, 6>, 15>& mat1,
                                                        const std::array<std::array<float, 6>, 6>& mat2) {
        

        std::array<std::array<float, 6>, 15> result = {{{0.0f}}};

        for (size_t i = 0; i < 15; ++i) {
            for (size_t j = 0; j < 6; ++j) {
                float sum = 0.0f;
                for (size_t k = 0; k < 6; ++k) {
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }

        return result;
    }

    std::array<std::array<float, 15>, 15> mat15_6_mat_6_15_mul(const std::array<std::array<float, 6>, 15>& mat1,
                                                                const std::array<std::array<float, 15>, 6>& mat2) {

        std::array<std::array<float, 15>, 15> result = {{{0.0f}}};

        for (size_t i = 0; i < 15; ++i) { 
            for (size_t j = 0; j < 15; ++j) { 
                float sum = 0.0f;
                for (size_t k = 0; k < 6; ++k) { 
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }

        return result;
    }

    std::array<float, 15> mat15_6_vec6_mult(const std::array<std::array<float, 6>, 15>& mat,
                                        const std::array<float, 6>& vec) {

        std::array<float, 15> result = {0.0f};

        for (size_t i = 0; i < 15; ++i) {
            float sum = 0.0f;
            for (size_t j = 0; j < 6; ++j) {
                sum += mat[i][j] * vec[j];
            }
            result[i] = sum;
        }

        return result;
    }

    // Matrix3 transpose
    template <typename T>
    std::array<std::array<T, 3>, 3> mat3_T(const std::array<std::array<T, 3>, 3>& mat) {
        std::array<std::array<T, 3>, 3> result = kfm::zero3();
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                result[i][j] = mat[j][i];
            }
        }
        return result;
    }

    // Transpose a 6x15 matrix
    std::array<std::array<float, 6>, 15> mat6_15_T(const std::array<std::array<float, 15>, 6>& mat) {
        std::array<std::array<float, 6>, 15> result = {{{0.0f}}};
        for (size_t i = 0; i < 6; ++i) {
            for (size_t j = 0; j < 15; ++j) {
                result[j][i] = mat[i][j];
            }
        }
        return result;
    }

    // Matrix15 transpose
    template <typename T>
    std::array<std::array<T, 15>, 15> mat15_T(const std::array<std::array<T, 15>, 15>& mat) {
        std::array<std::array<T, 15>, 15> result = kfm::zero15();
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                result[i][j] = mat[j][i];
            }
        }
        return result;
    }

    // Eye3 matrix with diagonal = val
    template <typename T>
    std::array<std::array<T, 3>, 3> eye3(const T val) {
        std::array<std::array<T, 3>, 3> eye =kfm::zero3();
        for (size_t i = 0; i < 3; ++i) {
            eye[i][i] = val;
        }

        return eye;
    }

    // Eye15 matrix with diagonal = val
    template <typename T>
    std::array<std::array<T, 15>, 15> eye15(const T val) {
        std::array<std::array<T, 15>, 15> eye =kfm::zero15();
        for (size_t i = 0; i < 15; ++i) {
            eye[i][i] = val;
        }

        return eye;
    }

    // vector*matrix (3 - 3*3)
    template <typename T>
    std::array<T, 3> vec3_mat3_mul(const std::array<T, 3>& vec,
                                const std::array<std::array<T, 3>, 3>& mat) {
        std::array<T, 3> result = {};

        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                result[i] += vec[j] * mat[i][j];
            }
        }

        return result;
    }

    // matrix*vector (3*3 - 3)
    template <typename T>
    std::array<T, 3> mat3_vec3_mul(const std::array<std::array<T, 3>, 3>& mat,
                                    const std::array<T, 3>& vec) {
        std::array<T, 3> result = {};

        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                result[i] += mat[i][j] * vec[j];
            }
        }

        return result;
    }

    // vector*matrix (15 - 15*15)
    template <typename T>
    std::array<T, 15> vec15_mat15_mul(const std::array<T, 15>& vec,
                                const std::array<std::array<T, 15>, 15>& mat) {
        std::array<T, 15> result = {};

        for (size_t i = 0; i < 15; ++i) {
            for (size_t j = 0; j < 15; ++j) {
                result[i] += vec[j] * mat[i][j];
            }
        }

        return result;
    }

    // matrix*vector (15*15 - 15)
    template <typename T>
    std::array<T, 15> mat15_vec15_mul(const std::array<std::array<T, 15>, 15>& mat,
                                    const std::array<T, 15>& vec) {
        std::array<T, 15> result = {};

        for (size_t i = 0; i < 15; ++i) {
            for (size_t j = 0; j < 15; ++j) {
                result[i] += mat[i][j] * vec[j];
            }
        }

        return result;
    }

    // vector add (3)
    template <typename T>
    void vec3_add(std::array<T, 3>& vec1, const std::array<T, 3>& vec2){
        for (size_t i = 0; i<3 ; ++i){
            vec1[i] += vec2[i];
        }
    }

    // vector add (9)
    template <typename T>
    void vec9_add(std::array<T, 9>& vec1, const std::array<T, 9>& vec2){
        for (size_t i = 0; i<9 ; ++i){
            vec1[i] += vec2[i];
        }
    }

    // vector add (15)
    template <typename T>
    void vec15_add(std::array<T, 15>& vec1, const std::array<T, 15>& vec2){
        for (size_t i = 0; i<15 ; ++i){
            vec1[i] += vec2[i];
        }
    }

    // scaling the values for sensors
    std::array<float,3> scale_g(int16_t raw[3], float mul){
        std::array<float,3> scaled = {};
        for (size_t i = 0; i<3; ++i){
            scaled[i] = raw[i]*mul;
        }
        return scaled;
    }


    // Mat decompose
    bool lu_decompose(std::array<std::array<float, 6>, 6>& matrix, std::array<int, 6>& index) {
        const int n = 6;
        std::array<float, 6> vv;
        float d = 1.0;

        for (int i = 0; i < n; ++i) {
            float big = 0.0;
            for (int j = 0; j < n; ++j) {
                if (matrix[i][j] > big) {
                    big = matrix[i][j];
                }
            }
            if (big == 0.0f) {
                return false; // Singular matrix
            }
            vv[i] = 1.0f / big;
        }

        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                float sum = matrix[i][j];
                for (int k = 0; k < i; ++k) {
                    sum -= matrix[i][k] * matrix[k][j];
                }
                matrix[i][j] = sum;
            }
        }

        for (int j = 0; j < n; ++j) {
            for (int i = n - 1; i >= 0; --i) {
                float sum = matrix[i][j];
                for (int k = i + 1; k < n; ++k) {
                    sum -= matrix[i][k] * matrix[k][j];
                }
                matrix[i][j] = sum / matrix[i][i];
            }
        }
        return true;
    }

    // inverse of 6*6 matrix
    std::array<std::array<float, 6>, 6> Inv6(const std::array<std::array<float, 6>, 6>& mat) {
        std::array<std::array<float, 6>, 6> matrix = mat;
        std::array<int, 6> index;
        
        std::array<std::array<float, 6>, 6> invMatrix = {{{0}}};
        // Perform LU Decomposition
        if (!kfm::lu_decompose(matrix, index)) {
            invMatrix[3][3] = 0.1234f;  // just to know its singular (no exception needs)
        }

        for (int j = 0; j < 6; ++j) {
            std::array<float, 6> b = {{0}};
            b[j] = 1.0f;

            for (int i = 0; i < 6; ++i) {
                float sum = b[i];
                for (int k = 0; k < i; ++k) {
                    sum -= matrix[i][k] * invMatrix[k][j];
                }
                invMatrix[i][j] = sum / matrix[i][i];
            }
        }
        return invMatrix;
    }


    // --------------------------- Mathematics ------------------------------------------

    // Single angle to rotation matrix
    std::array<std::array<float, 3>, 3> Single2Mat(const float& theta, const char& axis){
        // theta is rad
        // take care of the axis
        std::array<std::array<float, 3>, 3> Mat = kfm::zero3();
        if (axis == 'z'){
            // YAW
            Mat[0][0] = cos(theta);
            Mat[0][1] = -sin(theta);
            Mat[1][0] = sin(theta);
            Mat[1][1] = cos(theta);
            Mat[2][2] = 1.0;
        } else if (axis == 'y'){
            // Pitch
            Mat[0][0] = cos(theta);
            Mat[0][2] = sin(theta);
            Mat[1][1] = 1.0;
            Mat[2][0] = -sin(theta);
            Mat[2][2] = cos(theta);
        } else {
            // Roll
            Mat[0][0] = 1.0;
            Mat[1][1] = cos(theta);
            Mat[1][2] = -sin(theta);
            Mat[2][1] = sin(theta);
            Mat[2][2] = cos(theta);
        }
        return Mat;
    }

    // Euler angles to rotation matrix
    std::array<std::array<float, 3>, 3> Eul2Mat (const float& Heading, const float& Pitch, const float& Roll,
                                                const std::string& order){
        
        // take care the order

        std::array<std::array<float, 3>, 3> MatZ = kfm::Single2Mat(Heading, 'z');
        std::array<std::array<float, 3>, 3> MatY = kfm::Single2Mat(Pitch, 'y');
        std::array<std::array<float, 3>, 3> MatX = kfm::Single2Mat(Roll, 'x');
        if (order == "zyx"){
            kfm::mat3_mat3_mul_inplace(MatY, MatX);
            kfm::mat3_mat3_mul_inplace(MatZ, MatY);
            return MatZ;

        } else if(order == "xyz"){
            kfm::mat3_mat3_mul_inplace(MatY, MatZ);
            kfm::mat3_mat3_mul_inplace(MatX, MatY);
            return MatX;
        }
    }

    // Rotation matrix to euler angles
    std::array<float, 3> Mat2Eul (const std::array<std::array<float, 3>, 3> mat, const std::string& order){

        std::array<float, 3> eul;
        float Head = atan2(mat[1][0], mat[0][0]);
        float Pitch = atan2(-mat[2][0], sqrt(pow(mat[0][0],2) + pow(mat[1][0],2)));
        float Roll = atan2(mat[2][1], mat[2][2]);

        eul[0] = Head;
        eul[1] = Pitch;
        eul[2] = Roll;
        return eul;
    }

    // Conver a rotation vector to a matrix
    std::array<std::array<float, 3>, 3> RotVec2Mat(const std::array<float, 3>& RotVec){
        std::array<std::array<float, 3>, 3> Mat = kfm::zero3();

        Mat[0][1] = - RotVec[2];
        Mat[1][0] = RotVec[2];
        Mat[1][2] = - RotVec[2];
        Mat[2][0] = - RotVec[1];
        Mat[2][1] = RotVec[0];
        return Mat;
    }

    // Euler from DCM
    void dcm2angle(std::array<float, 9>& state,
                    const std::array<std::array<float,3>, 3>& Cbn) {
        // yaw
        state[0] = atan2f(Cbn[1][0], Cbn[0][0]);
        // pitch
        state[1] = asinf(-Cbn[2][0]);
        // roll
        state[2] = atan2f(Cbn[2][1], Cbn[2][2]);
    }

    
}


#endif