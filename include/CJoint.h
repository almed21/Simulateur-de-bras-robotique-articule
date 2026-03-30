#pragma once // Remplace les vieux #ifndef / #define, c'est plus moderne et supporté partout

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <stdexcept>

// Alias demandé par le sujet pour coller à la matrice 4x4 d'Eigen
using Mat4 = Eigen::Matrix4d;

class CJoint {
protected:
    // Variables protégées pour que les classes dérivées (Revolute, Prismatic) puissent y accéder
    double q_;    // Position articulaire courante
    double qMin_; // Butée minimale
    double qMax_; // Butée maximale

public:
    // Petit constructeur pratique pour forcer l'initialisation propre des variables
    CJoint(double q_min, double q_max, double q_init) 
        : q_(q_init), qMin_(q_min), qMax_(q_max) {}

    // Destructeur virtuel pour assurer une destruction correcte des objets dérivés via un pointeur de base
    virtual ~CJoint() = default;

    // --- Accesseurs (Getters) ---
    double getQ() const { return q_; }
    double getMin() const { return qMin_; }
    double getMax() const { return qMax_; }

    // --- Mutateur (Setter) ---
    void setQ(double q) {
        // Lève une exception si on tente de forcer le robot au-delà de ses limites
        if (q < qMin_ || q > qMax_) {
            throw std::out_of_range("Erreur : Valeur hors des butées articulaires [qMin, qMax].");
        }
        q_ = q;
    }

    // ========================================================================
    // MÉTHODES VIRTUELLES PURES (= 0)
    // C'est ce qui rend la classe CJoint "abstraite". 
    // On ne pourra jamais faire "CJoint joint;", on devra instancier ses filles.
    // ========================================================================
    
    // Calcule et retourne la matrice de transformation homogène 4x4 
    virtual Mat4 getTransform() const = 0;

    // Retourne le type de joint en string (ex: "Revolute" ou "Prismatic") 
    virtual std::string getTypeName() const = 0;

    // Crée une copie profonde polymorphique 
    virtual std::unique_ptr<CJoint> clone() const = 0;
};