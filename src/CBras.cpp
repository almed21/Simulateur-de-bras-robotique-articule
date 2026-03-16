#include "CBras.h"
#include <iomanip>  // Pour std::fixed et std::setprecision

void CBras::addJoint(std::unique_ptr<CJoint> joint) {
    joints_.push_back(std::move(joint));  // Prend ownership via move
}

size_t CBras::getNbJoints() const {
    return joints_.size();
}

CJoint& CBras::getJoint(size_t i) {
    if (i >= joints_.size()) {
        throw std::out_of_range("Indice de joint invalide");
    }
    return *joints_[i];
}

const CJoint& CBras::getJoint(size_t i) const {
    if (i >= joints_.size()) {
        throw std::out_of_range("Indice de joint invalide");
    }
    return *joints_[i];
}

Mat4 CBras::computeFK() const {
    Mat4 T = Mat4::Identity();  // Pose initiale T0 = I
    for (const auto& joint : joints_) {
        T = T * joint->getTransform();  // Chaîne T = T0 * T1 * ... * Tn
    }
    return T;
}

std::ostream& operator<<(std::ostream& os, const CBras& bras) {
    os << "Bras [" << bras.getNbJoints() << " DDL]\n";
    for (size_t i = 0; i < bras.getNbJoints(); ++i) {
        const auto& j = bras.getJoint(i);
        os << "[" << i << "] " << j.getTypeName() 
           << " q = " << std::fixed << std::setprecision(3) << j.getQ();
        if (j.getTypeName() == "Revolute") {
            os << " rad";
        } else {
            os << " m";
        }
        os << " bornes=[" << std::fixed << std::setprecision(3) 
           << j.getMin() << ", " << j.getMax() << "]" << std::endl;
    }
    return os;
}
