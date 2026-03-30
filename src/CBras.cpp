#include "CBras.h"
#include <iomanip>  // Pour std::fixed et std::setprecision
#include <algorithm> // Pour std::swap

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

// --- Implémentation de la Règle des Cinq ---

CBras::CBras(const CBras& other) {
    // Copie profonde en utilisant le polymorphisme (clone)
    for (const auto& joint : other.joints_) {
        joints_.push_back(joint->clone());
    }
}

CBras& CBras::operator=(CBras other) {
    // Copy-and-swap idiom : 'other' est une copie locale.
    // On échange nos données avec cette copie.
    std::swap(joints_, other.joints_);
    return *this;
    // En sortant de la fonction, 'other' est détruit et emporte nos anciennes données.
}

// --- Implémentation de l'interface Eigen ---

Eigen::VectorXd CBras::getQ() const {
    Eigen::VectorXd q(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        q[i] = joints_[i]->getQ();
    }
    return q;
}

void CBras::setQ(const Eigen::VectorXd& q) {
    if (static_cast<size_t>(q.size()) != joints_.size()) {
        throw std::invalid_argument("Erreur : La taille du vecteur ne correspond pas aux DDL du bras.");
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i]->setQ(q[i]);
    }
}

Eigen::VectorXd CBras::random() const {
    Eigen::VectorXd q_rand(joints_.size());
    // On génère une valeur entre qMin et qMax
    for (size_t i = 0; i < joints_.size(); ++i) {
        double min = joints_[i]->getMin();
        double max = joints_[i]->getMax();
        double r = static_cast<double>(std::rand()) / RAND_MAX;
        q_rand[i] = min + r * (max - min);
    }
    return q_rand;
}