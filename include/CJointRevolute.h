#pragma once

#include "CJoint.h"
#include <cmath> // Pour std::cos et std::sin

class CJointRevolute : public CJoint {
private:
    // Paramètre géométrique spécifique au joint de révolution : la longueur du bras (lien)
    double dx_; 

public:
    // Constructeur : on initialise la classe de base (CJoint) et notre attribut spécifique dx_
    CJointRevolute(double q_init, double q_min, double q_max, double dx)
        : CJoint(q_init, q_min, q_max), dx_(dx) {}

    // ========================================================================
    // IMPLÉMENTATION DES MÉTHODES VIRTUELLES PURES DE LA CLASSE MÈRE
    // (Le mot-clé 'override' est très important en C++ moderne : il dit au
    // compilateur de vérifier qu'on redéfinit bien une méthode existante)
    // ========================================================================

    // 1. Matrice de transformation cinématique
    Mat4 getTransform() const override {
        Mat4 T = Mat4::Identity(); // Crée une matrice identité 4x4
        
        // On récupère la position articulaire courante (l'angle theta)
        double theta = getQ(); 
        double c = std::cos(theta);
        double s = std::sin(theta);

        // On remplit la matrice selon la formule du sujet (T_rev) 
        T(0, 0) = c;  T(0, 1) = -s; T(0, 2) = 0; T(0, 3) = dx_;
        T(1, 0) = s;  T(1, 1) = c;  T(1, 2) = 0; T(1, 3) = 0;
        // Les lignes 2 et 3 sont déjà bonnes grâce à Mat4::Identity()
        // (0, 0, 1, 0) et (0, 0, 0, 1)

        return T;
    }

    // 2. Nom du type (utile pour l'affichage operator<< de la séance 2)
    std::string getTypeName() const override {
        return "Revolute";
    }

    // 3. Clonage polymorphique
    std::unique_ptr<CJoint> clone() const override {
        // Crée un nouveau pointeur unique qui copie l'instance courante (*this)
        return std::make_unique<CJointRevolute>(*this);
    }
};