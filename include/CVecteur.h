#pragma once

#include <array>
#include <cmath>
#include <Eigen/Dense>

template <typename T, std::size_t N>
class CVecteur {
private:
    std::array<T, N> data_;

public:
    // Constructeurs
    CVecteur() { data_.fill(T{}); }
    CVecteur(std::initializer_list<T> list) {
        std::size_t i = 0;
        for (const auto& val : list) {
            if (i < N) data_[i++] = val;
        }
    }

    // Accesseurs
    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }

    // Opérations mathématiques
    // addition element par element
    CVecteur operator+(const CVecteur& other) const {
        CVecteur res;
        for(std::size_t i=0; i<N; ++i) res[i] = data_[i] + other[i];
        return res;
    }

    // soustraction element par element
    CVecteur operator-(const CVecteur& other) const {
        CVecteur res;
        for(std::size_t i=0; i<N; ++i) res[i] = data_[i] - other[i];
        return res;
    }

    // multiplication par un scalaire
    CVecteur operator*(T scalar) const {
        CVecteur res;
        for(std::size_t i=0; i<N; ++i) res[i] = data_[i] * scalar;
        return res;
    }

    // produit scalaire
    T dot(const CVecteur& other) const {
        T sum = 0;
        for(std::size_t i=0; i<N; ++i) sum += data_[i] * other[i];
        return sum;
    }

    // norme
    T norm() const {
        return std::sqrt(dot(*this));
    }

    // toEigen() : retourne Eigen::Matrix<T,N,1> via Eigen::Map (zero copie)
    Eigen::Matrix<T, N, 1> toEigen() const {
        return Eigen::Map<const Eigen::Matrix<T, N, 1>>(data_.data());
    }
};