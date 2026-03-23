#include <iostream>
#include <cmath>
#include <memory>
#include "CBras.h"
#include "CJointRevolute.h"
#include "CJointPrismatic.h"

using namespace std;

int main() {
    // Instanciation du bras [cite: 45]
    CBras bras; 

    // Ajout des 3 joints avec std::make_unique (qui effectue le new en toute sécurité)
    // Ordre des paramètres de notre constructeur : q_init, q_min, q_max, dx
    bras.addJoint(make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));
    bras.addJoint(make_unique<CJointRevolute>(-M_PI, M_PI, 0.3, 0.3));
    
    // Pour le prismatique : q_init, q_min, q_max
    bras.addJoint(make_unique<CJointPrismatic>(0.0, 0.5, 0.1));

    // Affichage du bras grâce à notre surcharge de l'opérateur << [cite: 54]
    cout << bras << endl;

    // --- Bonus : Testons la cinématique directe ! ---
    cout << "----------------------------------------\n";
    cout << "Matrice de transformation (Cinématique Directe) :\n";
    cout << bras.computeFK() << endl;

    return 0;
}