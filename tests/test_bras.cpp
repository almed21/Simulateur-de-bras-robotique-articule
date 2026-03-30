#include <gtest/gtest.h>
#include "CBras.h"
#include "CJointRevolute.h"
#include "CJointPrismatic.h"
#include <Eigen/Dense>

// ========================================================================
// Test 3.a : computeFK() sur un bras vide retourne I4
// ========================================================================
TEST(BrasTest, EmptyArmFKReturnsIdentity) {
    CBras bras; // Bras sans aucun joint
    Mat4 T = bras.computeFK();
    
    // On vérifie que la matrice retournée est bien l'Identité
    EXPECT_TRUE(T.isApprox(Mat4::Identity(), 1e-10));
}

// ========================================================================
// Test 3.b : 1 joint rotatif, extraction de la translation
// ========================================================================
TEST(BrasTest, SingleRevoluteJointTranslation) {
    CBras bras;
    // Ajout d'1 joint : theta = 0, qMin = -pi, qMax = pi, dx = 0.5 m
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));
    
    Mat4 T = bras.computeFK();
    
    // Extraction du bloc 3x1 (la colonne de translation) à partir de la ligne 0, colonne 3
    Eigen::Vector3d translation = T.block<3, 1>(0, 3);
    
    // L'effecteur doit être en (0.5, 0, 0)
    Eigen::Vector3d expected(0.5, 0.0, 0.0);
    
    EXPECT_TRUE(translation.isApprox(expected, 1e-10));
}

// ========================================================================
// Test 3.c : getJoint(i) lève std::out_of_range pour i >= N
// ========================================================================
TEST(BrasTest, GetJointOutOfBoundsThrows) {
    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));
    // N = 1 (il y a 1 joint, donc l'indice valide est 0)
    
    // Vérification que l'indice 1 (qui est >= N) lève bien l'exception
    EXPECT_THROW(bras.getJoint(1), std::out_of_range);
    
    // L'indice 0 ne doit rien lever
    EXPECT_NO_THROW(bras.getJoint(0));
}

// ========================================================================
// Test Exercice 3 - 2.b : Idempotence de setQ(getQ())
// ========================================================================
TEST(BrasTest, SetQGetQIsIdempotent) {
    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.5, 0.3));
    bras.addJoint(std::make_unique<CJointPrismatic>(0.0, 1.0, 0.2));

    // On sauvegarde l'état initial
    Eigen::VectorXd q_initial = bras.getQ();
    
    // On réapplique cet état
    EXPECT_NO_THROW(bras.setQ(q_initial));
    
    // On vérifie que rien n'a changé
    Eigen::VectorXd q_final = bras.getQ();
    EXPECT_TRUE(q_initial.isApprox(q_final, 1e-10));
}