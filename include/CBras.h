#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include "CJoint.h"

class CBras {
private:
    std::vector<std::unique_ptr<CJoint>> joints_;

public:
    // Constructeur par défaut
    CBras() = default;

    // --- Méthodes de gestion des joints ---

    // Ajoute un joint au bras. 
    void addJoint(std::unique_ptr<CJoint> joint);

    // Retourne le nombre de degrés de liberté (DDL)
    size_t getNbJoints() const;

    // Retourne une référence vers un joint spécifique (utile pour modifier son 'q')
    // Lève une exception si l'indice est invalide.
    CJoint& getJoint(size_t i);
    const CJoint& getJoint(size_t i) const;

    // --- Cinématique ---
    
    // Calcule la cinématique directe (Forward Kinematics)
    Mat4 computeFK() const;

    // --- Surcharge d'opérateur ---
    
    // Surcharge de l'opérateur << pour l'affichage avec std::cout
    friend std::ostream& operator<<(std::ostream& os, const CBras& bras);

};