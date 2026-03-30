#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include "CJoint.h"
#include <Eigen/Dense> 

class CBras {
private:
    std::vector<std::unique_ptr<CJoint>> joints_;

public:
    // Constructeur par défaut
    CBras() = default;

    // --- RÈGLE DES CINQ ---
    // 1. Constructeur de copie
    CBras(const CBras& other);
    
    // 2. Opérateur d'affectation (Copy-and-Swap)
    CBras& operator=(CBras other); // Passage par valeur volontaire ici
    
    // 3 & 4. Déplacement (Move)
    CBras(CBras&&) = default;
    CBras& operator=(CBras&&) = default;
    
    // 5. Destructeur
    ~CBras() = default;

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

    // --- INTERFACE EIGEN ---
    Eigen::VectorXd getQ() const; // retourne un vecteur de taille N avec les q_ de chaque joint
    void setQ(const Eigen::VectorXd& q); // applique q[i] a chaque joint via setQ(); leve
                                        // std::invalid_argument si q.size() != getNbJoints()
    Eigen::VectorXd random() const; // tire q[i] uniformement dans [qMin_,qMax_] pour chaque joint

};