#include <gtest/gtest.h>
#include "CVecteur.h"

TEST(VecteurTest, DotAndNorm) {
    CVecteur<double, 3> u = {1.0, 0.0, 0.0};
    CVecteur<double, 3> v = {0.0, 1.0, 0.0};

    // u.v = 0
    EXPECT_DOUBLE_EQ(u.dot(v), 0.0);

    // ||u|| = 1
    EXPECT_DOUBLE_EQ(u.norm(), 1.0);
}