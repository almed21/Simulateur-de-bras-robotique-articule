#include <iostream>
#include <cmath>
#include <memory>
#include "CBras.h"
#include "CJointRevolute.h"
#include "CJointPrismatic.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

using namespace std;

int main() {
    cout << "=== SÉANCE 4 : INTÉGRATION DU BRAS 4-DDL ===" << endl;

    // 1. Instanciation du bras 4-DDL (selon le tableau du sujet)
    CBras bras;
    // Rappel ordre : (qMin, qMax, q0, dx) pour Revolute, (qMin, qMax, q0) pour Prismatic
    bras.addJoint(make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.30));         // [0] Épaule
    bras.addJoint(make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.25));         // [1] Coude
    bras.addJoint(make_unique<CJointPrismatic>(0.0, 0.20, 0.0));                // [2] Ext.
    bras.addJoint(make_unique<CJointRevolute>(-M_PI/2.0, M_PI/2.0, 0.0, 0.05)); // [3] Poignet

    // 2. Vérification de l'indépendance de la copie profonde
    cout << "\n--- Test de la Règle des 5 (Copie et Déplacement) ---" << endl;
    CBras copie_bras = bras; // Appelle le constructeur de copie
    
    Eigen::VectorXd q_test(4); 
    q_test << 0.1, 0.0, 0.0, 0.0;
    copie_bras.setQ(q_test); // On modifie la copie
    
    cout << "q_original de l'épaule : " << bras.getQ()[0] << " rad (attendu: 0)" << endl;
    cout << "q_copie de l'épaule    : " << copie_bras.getQ()[0] << " rad (attendu: 0.1)" << endl;

    // 3. Test du déplacement (std::move)
    CBras bras_deplace = std::move(copie_bras);
    cout << "Nb joints dans la copie (après move) : " << copie_bras.getNbJoints() << " (attendu: 0)" << endl;
    cout << "Nb joints dans le bras_deplace     : " << bras_deplace.getNbJoints() << " (attendu: 4)" << endl;

    // 4. Balayage de l'épaule de -pi/2 à pi/2 par pas de pi/8
    cout << "\n--- Balayage de l'épaule ---" << endl;
    Eigen::VectorXd q = bras.getQ();
    
    // On ajoute 1e-5 à la condition de fin pour éviter les erreurs d'arrondi des flottants
    for (double theta = -M_PI/2.0; theta <= M_PI/2.0 + 1e-5; theta += M_PI/8.0) {
        try {
            q[0] = theta;
            bras.setQ(q);
            Mat4 T = bras.computeFK();
            Eigen::Vector3d pos = T.block<3, 1>(0, 3); // Extraction de la translation
            
            cout << "Theta = " << theta << " rad \t-> Pos effecteur : [" << pos.transpose() << "]" << endl;
        } catch (const std::out_of_range& e) {
            cout << "Erreur de butée : " << e.what() << endl;
        }
    }

    // 5. Test d'exception sur un dépassement de butée explicite
    cout << "\n--- Test de dépassement de butée ---" << endl;
    try {
        q[0] = 5.0; // 5.0 rad est supérieur à pi (3.14)
        bras.setQ(q);
    } catch (const std::out_of_range& e) {
        cout << "Exception attrapée avec succès : " << e.what() << endl;
    }

    cout << "\n=== VALIDATION PINOCCHIO (UR5) ===" << endl;

    // 1. Initialisation Pinocchio (adaptez le chemin vers votre fichier URDF)
    const string urdf_path = "ur5.urdf"; 
    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        // 2. Configuration : q0 = 0.1 rad, le reste à 0
        Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model.nq);
        q_pin[0] = 0.1; 

        // 3. Calcul Pinocchio
        pinocchio::forwardKinematics(model, data, q_pin);
        
        // L'effecteur de l'UR5 est généralement le 6ème joint (index 6)
        Mat4 T_pinocchio = data.oMi[6].toHomogeneousMatrix(); 

        // 4. Comparaison avec CBras (Il faudrait instancier un CBras aux dimensions de l'UR5 ici)
        // Pour l'exemple, on affiche juste la matrice Pinocchio :
        cout << "Matrice Pinocchio pour q0=0.1 :" << endl;
        cout << T_pinocchio << endl;

    } catch (const std::invalid_argument& e) {
        cout << "Fichier URDF introuvable. Placez ur5.urdf dans le dossier." << endl;
    }

    cout << "\n=== EXERCICE 3 (BONUS) : RÉGULATEUR PROPORTIONNEL ===" << endl;

    // 1. Définition de la cible (x*, y*) et des paramètres
    Eigen::Vector2d cible(0.2, 0.2); // Une cible atteignable en x et y
    double Kp = 0.5;
    double epsilon = 0.01;
    Eigen::VectorXd q_actuel = bras.getQ();
    
    int iteration = 0;
    int max_iter = 1000; // Sécurité pour éviter une boucle infinie
    
    while (iteration < 5000) { 
        // Position actuelle (x, y) de l'effecteur
        Mat4 T = bras.computeFK();
        Eigen::Vector2d pos_actuelle(T(0,3), T(1,3)); 
        
        // Calcul de l'erreur e = cible - position_actuelle
        Eigen::Vector2d e = cible - pos_actuelle;
        
        // Condition d'arrêt : norme de l'erreur < 0.01 m
        if (e.norm() < epsilon) {
            cout << "\n>>> Succès ! Cible atteinte en " << iteration << " itérations !" << endl;
            cout << "Erreur finale : " << e.norm() << " m" << endl;
            break;
        }
        
        // 2. Calcul du Jacobien numérique
        Eigen::MatrixXd J(2, bras.getNbJoints());
        double delta_q = 1e-5; 
        
        for (size_t i = 0; i < bras.getNbJoints(); ++i) {
            Eigen::VectorXd q_temp = q_actuel;
            q_temp[i] += delta_q; 
            bras.setQ(q_temp);
            Mat4 T_temp = bras.computeFK();
            Eigen::Vector2d pos_temp(T_temp(0,3), T_temp(1,3));
            J.col(i) = (pos_temp - pos_actuelle) / delta_q;
        }
        
        // On remet le bras dans l'état actuel propre avant la mise à jour
        bras.setQ(q_actuel);
        
        // 3. Mise à jour des angles
        Eigen::VectorXd delta_theta = Kp * J.transpose() * e;
        Eigen::VectorXd q_next = q_actuel + delta_theta;
        
        // 4. Application au bras avec protection
        try {
            bras.setQ(q_next);
            q_actuel = q_next; // Validé ! On met à jour notre état
        } catch(const std::out_of_range&) {
            // On a tapé une butée ! On réduit Kp pour faire un pas plus petit
            // et on ne met PAS à jour q_actuel (on reste synchronisé)
            Kp *= 0.8; 
        }
        
        iteration++;
    }
    
    // Affichage du résultat final
    Mat4 T_final = bras.computeFK();
    Eigen::Vector2d pos_finale(T_final(0,3), T_final(1,3));
    cout << "Cible demandée   : [" << cible.transpose() << "]" << endl;
    cout << "Position finale  : [" << pos_finale.transpose() << "]" << endl;    

    return 0;
}