#include <pinocchio/spatial/se3.hpp>
#include <gtest/gtest.h>
#include "CJointRevolute.h"
#include "CJointPrismatic.h"

// ========================================================================
// Test 2.c : Validation avec pinocchio::SE3 pour theta = pi/4
// ========================================================================
TEST(JointTest, CompareWithPinocchioPiOver4) {
    double theta = M_PI / 4.0;
    double dx = 2.0; // Valeur arbitraire pour le test

    // 1. Notre implémentation
    CJointRevolute joint(theta, -M_PI, M_PI, dx);
    Mat4 my_T = joint.getTransform();

    // 2. Implémentation Pinocchio
    // Rotation de theta autour de l'axe Z
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    // Translation de dx sur l'axe X (selon la matrice Trev du sujet)
    Eigen::Vector3d t(dx, 0.0, 0.0);
    
    // Création de l'objet SE3 (Special Euclidean group en 3D)
    pinocchio::SE3 pin_se3(R, t);
    
    // Récupération de la matrice homogène 4x4
    Mat4 pin_T = pin_se3.toHomogeneousMatrix();

    // 3. Comparaison (avec tolérance 1e-10 comme demandé par le sujet)
    EXPECT_TRUE(my_T.isApprox(pin_T, 1e-10));
    
    // Alternative avec EXPECT_NEAR pour vérifier élément par élément si isApprox échoue
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            EXPECT_NEAR(my_T(i, j), pin_T(i, j), 1e-10);
        }
    }
}

// ========================================================================
// Test 3.a : Vérification de l'exception hors bornes
// ========================================================================
TEST(JointTest, SetQOutOfBoundsThrowsException) {
    // Création d'un joint avec des butées entre -1.0 et 1.0
    CJointRevolute joint(0.0, -1.0, 1.0, 0.5);

    // On vérifie qu'une valeur au-delà de qMax lève bien std::out_of_range
    EXPECT_THROW(joint.setQ(2.0), std::out_of_range);

    // On vérifie qu'une valeur en-dessous de qMin lève bien std::out_of_range
    EXPECT_THROW(joint.setQ(-2.0), std::out_of_range);

    // On vérifie qu'une valeur valide ne lève PAS d'exception
    EXPECT_NO_THROW(joint.setQ(0.5));
}

// ========================================================================
// Test 3.b : Matrice du joint Revolute (theta = 0, dx = 0)
// ========================================================================
TEST(JointTest, RevoluteTransformIdentity) {
    // theta = 0.0, qMin = -pi, qMax = pi, dx = 0.0
    CJointRevolute joint(0.0, -3.14159, 3.14159, 0.0);

    Mat4 T = joint.getTransform();
    Mat4 I4 = Mat4::Identity();

    // La méthode isApprox d'Eigen est parfaite pour comparer des matrices 
    // avec une tolérance pour les erreurs d'arrondi des flottants.
    EXPECT_TRUE(T.isApprox(I4, 1e-10));
}

// ========================================================================
// Test 3.c : Matrice du joint Prismatic (d = 1)
// ========================================================================
TEST(JointTest, PrismaticTransformTranslation) {
    // d (ou q) = 1.0, qMin = 0.0, qMax = 2.0
    CJointPrismatic joint(1.0, 0.0, 2.0);

    Mat4 T = joint.getTransform();
    
    // On crée la matrice attendue (Identité + translation de 1 sur l'axe Z)
    Mat4 expected = Mat4::Identity();
    expected(2, 3) = 1.0;

    EXPECT_TRUE(T.isApprox(expected, 1e-10));
}

// ========================================================================
// Test 2.c : Validation mathématique avec theta = pi/4 (Bonus de rigueur)
// ========================================================================
TEST(JointTest, RevoluteTransformPiOver4) {
    double pi_4 = 3.14159265358979323846 / 4.0;
    // theta = pi/4, dx = 2.0
    CJointRevolute joint(pi_4, -3.14, 3.14, 2.0);

    Mat4 T = joint.getTransform();

    // Vérification manuelle des coefficients selon la formule Trev du sujet
    double cos_pi_4 = std::cos(pi_4);
    double sin_pi_4 = std::sin(pi_4);

    EXPECT_NEAR(T(0, 0), cos_pi_4, 1e-10);
    EXPECT_NEAR(T(0, 1), -sin_pi_4, 1e-10);
    EXPECT_NEAR(T(0, 3), 2.0, 1e-10);      // dx
    EXPECT_NEAR(T(1, 0), sin_pi_4, 1e-10);
    EXPECT_NEAR(T(1, 1), cos_pi_4, 1e-10);
}