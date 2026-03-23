#pragma once

#include "CJoint.h"
#include <memory>
#include <string>

class CJointPrismatic : public CJoint {
public:
    // Constructeur : on initialise la classe de base (CJoint).
    // Contrairement au Revolute, le sujet ne mentionne pas de lien fixe "dx" 
    // pour le joint prismatique, donc on ne prend que les butées et la position.
    CJointPrismatic(double q_min, double q_max, double q_init)
        : CJoint(q_min, q_max, q_init) {}

    // ========================================================================
    // IMPLÉMENTATION DES MÉTHODES VIRTUELLES PURES
    // Chacune redéfinit getTransform(), getTypeName() et clone().
    // ========================================================================

    // 1. Matrice de transformation cinématique
    Mat4 getTransform() const override {
        // On part d'une matrice identité 4x4, ce qui remplit déjà la diagonale de 1
        Mat4 T = Mat4::Identity();
        
        // On applique la translation sur l'axe Z. 
        // T(2, 3) correspond à la 3ème ligne, 4ème colonne (indexé à partir de 0).
        T(2, 3) = getQ(); // La valeur "d" de la matrice T_pris 
        
        return T;
    }

    // 2. Nom du type (utile pour l'affichage de la séance 2)
    std::string getTypeName() const override {
        return "Prismatic";
    }

    // 3. Clonage polymorphique
    // Permet à la classe CBras de copier ce joint sans connaître son type exact
    std::unique_ptr<CJoint> clone() const override {
        return std::make_unique<CJointPrismatic>(*this);
    }
};