# Simulateur-de-bras-robotique-articule
Projet en binôme, 4 séances de 4h. Le sujet modélise en C++17 un bras robotique articulé, dans l’esprit de la bibliothèque Pinocchio (cinématique, corps rigides) et de son back-end algébrique Eigen. 

# Réponses au questions
## Exercice 1 
### 1. Classe abstraite CJoint
#### a. Pourquoi le destructeur doit-il être virtual ? Montrer le comportement indéfini sur un exemple concret (delete via pointeur de base).

> Si on détruit un objet dérivé via un pointeur de base, sans le mot-clé virtual, seul le destructeur de CJoint serait appelé. Le destructeur de la classe fille serait ignoré, provoquant une fuite mémoire.

#### b. Pourquoi setQ() ne peut-il pas être const ? Pourquoi les accesseurs getQ() etc. doivent-ils l’être ?

> setQ() n'est pas 'const' car son but est justement de modifier l'attribut q_, tandi que les accesseurs sont 'const' car consulter une valeur garantit qu'on ne modifie pas l'état du robot.

#### Misc

> On a choisi de mettre directement l'implémentation des méthodes getQ() et setQ() dans le fichier .h (on appelle ça des méthodes inlines implicites) pour suivre la norme C++ et permettre au compilateur d'optimiser le code plus facilement.

### 2. Classes dérivées
#### a. Que se passe-t-il si getTransform() n’est pas virtual ?

> Si getTransform() n'est pas virtual :
>
> - Perte du polymorphisme : Le compilateur se base uniquement sur le type du pointeur (CJoint*) et non sur le type réel de l'objet qu'il contient (CJointRevolute ou CJointPrismatic).
>
> - Mauvais appel : C'est la méthode de la classe mère (CJoint::getTransform()) qui sera toujours exécutée en aveugle.
>
> - Conséquence pour le projet : Lorsque la classe CBras bouclera sur ses joints pour calculer la cinématique directe, elle n'appliquera jamais les vraies matrices de rotation ou de translation. Le bras robotique restera immobile ou plantera (surtout si la méthode de base n'a pas de corps).

#### b. Pourquoi ne peut-on pas utiliser le constructeur de copie par défaut directement depuis CBras ?

> Parce que CBras va stocker des std::unique_ptr<CJoint>. On ne peut pas copier directement un unique_ptr, ni deviner le type dérivé exact (Revolute ou Prismatic) juste à partir d'un pointeur CJoint. La méthode clone() résout ça en créant un nouvel objet du bon type dynamique.
