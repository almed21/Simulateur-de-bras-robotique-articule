# Simulateur-de-bras-robotique-articule

Projet en binôme, 4 séances de 4h. Le sujet modélise en C++17 un bras robotique articulé, dans l’esprit de la bibliothèque Pinocchio (cinématique, corps rigides) et de son back-end algébrique Eigen.

## installation libs
### installation conda et libs eigen et pinocchio
1. telecharger le script d'installation: "wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"
2. lancer l'installation puis choisir "yes" sur chaque etape : "bash Miniconda3-latest-Linux-x86_64.sh"
3. activer conda: "source ~/.bashrc"
4. creer l'environnement avec les library de eigen et pinocchio: conda create -n robot_env pinocchio eigen pkg-config -c conda-forge
5. activer l'environnement: "conda activate robot_env"

### installation de gtest et valgrind (g++ et make en plus)
sudo apt update
sudo apt install build-essential libgtest-dev valgrind

### installation ur5
1. telecharger l'urdf: "wget https://raw.githubusercontent.com/uwgraphics/RelaxedIK/master/urdfs/ur5.urdf"
2. definir le chemin: "export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

## Réponses au questions

### Exercice 1

#### 1. Classe abstraite CJoint

##### a. Pourquoi le destructeur doit-il être virtual ? Montrer le comportement indéfini sur un exemple concret (delete via pointeur de base)

> Si on détruit un objet dérivé via un pointeur de base, sans le mot-clé virtual, seul le destructeur de CJoint serait appelé. Le destructeur de la classe fille serait ignoré, provoquant une fuite mémoire.

##### b. Pourquoi setQ() ne peut-il pas être const ? Pourquoi les accesseurs getQ() etc. doivent-ils l’être ?

> setQ() n'est pas 'const' car son but est justement de modifier l'attribut q_, tandi que les accesseurs sont 'const' car consulter une valeur garantit qu'on ne modifie pas l'état du robot.

##### Misc

> On a choisi de mettre directement l'implémentation des méthodes getQ() et setQ() dans le fichier .h (on appelle ça des méthodes inlines implicites) pour suivre la norme C++ et permettre au compilateur d'optimiser le code plus facilement.

#### 2. Classes dérivées

##### a. Que se passe-t-il si getTransform() n’est pas virtual ?

> Si getTransform() n'est pas virtual :
>
> - Perte du polymorphisme : Le compilateur se base uniquement sur le type du pointeur (CJoint*) et non sur le type réel de l'objet qu'il contient (CJointRevolute ou CJointPrismatic).
>
> - Mauvais appel : C'est la méthode de la classe mère (CJoint::getTransform()) qui sera toujours exécutée en aveugle.
>
> - Conséquence pour le projet : Lorsque la classe CBras bouclera sur ses joints pour calculer la cinématique directe, elle n'appliquera jamais les vraies matrices de rotation ou de translation. Le bras robotique restera immobile ou plantera (surtout si la méthode de base n'a pas de corps).

##### b. Pourquoi ne peut-on pas utiliser le constructeur de copie par défaut directement depuis CBras ?

> Parce que CBras va stocker des std::unique_ptr<CJoint>. On ne peut pas copier directement un unique_ptr, ni deviner le type dérivé exact (Revolute ou Prismatic) juste à partir d'un pointeur CJoint. La méthode clone() résout ça en créant un nouvel objet du bon type dynamique.

### Exercice 2

#### 1. Classe CBras

##### a. Pourquoi addJoint() prend un unique_ptr par valeur et requiert std::move() à l’appel ?

> Elle prend un unique_ptr par valeur pour forcer le transfert de propriété.
> std::move() est requis à l'appel car un unique_ptr ne peut pas être copié, il doit être "déplacé" (moved) pour garantir qu'il n'y a qu'un seul propriétaire en mémoire.

##### b. Pourquoi CBras n’est-elle pas copiable par défaut ?

> Parce qu'elle contient un std::vector de std::unique_ptr.
> Les unique_ptr interdisent la copie par essence (pour éviter de libérer deux fois la même mémoire).
> Le compilateur supprime donc automatiquement le constructeur de copie de CBras.

##### c. Quel est le type de retour de operator« et pourquoi retourner std::ostream& ?

> On retourne std::ostream& pour permettre le chaînage

#### 2. Cinématique directe

##### a. Comparer A * B (Eigen) à une boucle triple manuelle. Citer deux avantages concrets

> 1. Syntaxe beaucoup plus lisible et proche des mathématiques.
> 2. Eigen optimise automatiquement les calculs avec des instructions vectorielles.

##### b. Quelle méthode de pinocchio::SE3 retourne la Eigen::Matrix4d équivalente ? Comment comparer les deux résultats numériquement ?

> 1. La méthode de la classe pinocchio::SE3 qui permet d'extraire et de retourner la matrice homogène 4x4 équivalente au format Eigen::Matrix4d est toHomogeneousMatrix().
> 2. Pour comparer les deux résultats numériquement, on calcule l'erreur de Frobenius (la norme de la différence entre les deux matrices) qui doit être proche de zéro (inférieure à 10^(-10)). En code Eigen, cela se traduit simplement par la méthode matrice_A.isApprox(matrice_B, 1e-10).

### Exercice 3

#### 1. Règle des cinq sur CBras

##### a. Pourquoi ne pas copier le vector directement ?

> Parce que le vecteur contient des pointeurs std::unique_ptr. Par définition, un unique_ptr est le propriétaire exclusif de la mémoire qu'il pointe ; il est donc interdit de le copier (sinon deux pointeurs essaieraient de libérer la même mémoire à la fin).

##### b. Quelle garantie d'exception offre l'idiome copy-and-swap ?

> Il offre la garantie forte d'exception (strong exception guarantee). Le principe est simple : on crée d'abord une copie dans une variable temporaire. Si cette copie échoue, notre objet actuel n'est pas modifié. Si la copie réussit, on échange (swap) les données. L'ancien état est ensuite détruit proprement avec le temporaire.

##### c.Constructeur et opérateur de déplacement : default suffit-il ?

> Oui, Le mot-clé = default demande au compilateur de "déplacer" les membres de la classe. Ici, il va déplacer le std::vector, qui lui-même est tout à fait capable de déplacer ses éléments internes (les unique_ptr). Le transfert de propriété se fait de manière sûre et instantanée.

##### d. Destructeur : default suffit-il et dans quel ordre les joints sont-ils détruits ?

> Oui, default suffit. Le destructeur par défaut va appeler le destructeur du std::vector, qui va détruire proprement tous les unique_ptr contenus. En C++, les éléments d'un tableau/vecteur sont toujours détruits dans l'ordre inverse de leur construction (du dernier joint ajouté jusqu'au premier).

#### 2. Interface Eigen::VectorXd

##### a. Pourquoi Pinocchio utilise Eigen::VectorXd plutôt que std::vector<double> ?

> std::vector est un simple conteneur généraliste. À l'inverse, Eigen::VectorXd est conçu spécifiquement pour l'algèbre linéaire. Il permet de faire des calculs mathématiques optimisés (grâce aux instructions vectorielles du processeur) comme calculer la norme (.norm()), le produit scalaire (.dot()), ou additionner des vecteurs avec un simple opérateur +.

#### 3. Template CVecteur<T,N>

##### a. Pourquoi la définition entière doit-elle être dans le .h ?

> Un template n'est pas du code pré-compilé, c'est un patron. Le compilateur a impérativement besoin de voir toute l'implémentation dans le fichier .h au moment où on l'utilise pour générer le code final, sinon l'éditeur de liens échouera.

##### b. Différence entre typename T et std::size_t N

> typename T (type) : Définit la nature des données stockées (ex: double, int).
>
> std::size_t N (non-type) : Définit une valeur constante (la taille du tableau) connue à la compilation, ce qui permet une allocation statique très rapide en mémoire.

#### 4. Valgrind

##### Sortie:

> ==36707== HEAP SUMMARY:
==36707==     in use at exit: 0 bytes in 0 blocks
==36707==   total heap usage: 8 allocs, 8 frees, 74,920 bytes allocated
==36707==
==36707== All heap blocks were freed -- no leaks are possible
==36707==
==36707== For lists of detected and suppressed errors, rerun with: -s
==36707== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)

##### Commentaire

> L'analyse Valgrind confirme qu'il n'y a aucune fuite de mémoire (0 bytes in 0 blocks). Ce résultat est garanti par l'utilisation de std::vector<std::unique_ptr<CJoint>> dans la classe CBras. L'utilisation des pointeurs intelligents (unique_ptr) applique strictement le principe RAII : la mémoire de chaque joint allouée dynamiquement sur le tas (heap) est automatiquement et obligatoirement libérée à la destruction du bras robotique, rendant les fuites impossibles sans même avoir à écrire de code dans le destructeur.

### Exercice 4

#### 2. Validation avec Pinocchio

##### b. Quel intérêt de découpler Model (lecture seule) et Data (mutable) en multi-threading ?

> Le Model contient la description physique immuable du robot (longueurs, butées). En multi-threading, tous les threads peuvent lire un unique Model partagé sans aucun risque de conflit (pas besoin de mutex), ce qui économise énormément de mémoire. Chaque thread possède en revanche sa propre instance de Data pour y écrire ses résultats de calculs en parallèle de manière totalement isolée.

##### c. Différence entre modèle cinématique et modèle de collision, et rôle de computeCollisions() ?

> - Le modèle cinématique est une abstraction mathématique (des repères, des points et des axes) servant uniquement à calculer des positions.
> - Le modèle de collision habille ce squelette avec des volumes 3D réels (des maillages, des cylindres).
> - La fonction computeCollisions() utilise la bibliothèque hpp-fcl pour tester mathématiquement si ces volumes 3D s'interpénètrent (ce qui signifierait que le robot se rentre dedans ou heurte un obstacle).

#### sortie valgrind

> ==58585== Memcheck, a memory error detector
==58585== Copyright (C) 2002-2022, and GNU GPL'd, by Julian Seward et al.
==58585== Using Valgrind-3.22.0 and LibVEX; rerun with -h for copyright info
==58585== Command: ./build/simulateur
==58585== 
=== SÉANCE 4 : INTÉGRATION DU BRAS 4-DDL ===

--- Test de la Règle des 5 (Copie et Déplacement) ---
q_original de l'épaule : 0 rad (attendu: 0)
q_copie de l'épaule    : 0.1 rad (attendu: 0.1)
Nb joints dans la copie (après move) : 0 (attendu: 0)
Nb joints dans le bras_deplace     : 4 (attendu: 4)

--- Balayage de l'épaule ---
Theta = -1.5708 rad     -> Pos effecteur : [ 0.3 -0.3    0]
Theta = -1.1781 rad     -> Pos effecteur : [ 0.414805 -0.277164         0]
Theta = -0.785398 rad   -> Pos effecteur : [ 0.512132 -0.212132         0]
Theta = -0.392699 rad   -> Pos effecteur : [ 0.577164 -0.114805         0]
Theta = 0 rad   -> Pos effecteur : [0.6   0   0]
Theta = 0.392699 rad    -> Pos effecteur : [0.577164 0.114805        0]
Theta = 0.785398 rad    -> Pos effecteur : [0.512132 0.212132        0]
Theta = 1.1781 rad      -> Pos effecteur : [0.414805 0.277164        0]
Theta = 1.5708 rad      -> Pos effecteur : [0.3 0.3   0]

--- Test de dépassement de butée ---
Exception attrapée avec succès : Erreur : Valeur hors des butées articulaires [qMin, qMax].

=== VALIDATION PINOCCHIO (UR5) ===
Matrice Pinocchio pour q0=0.1 :
  0.995004 -0.0998334          0 -0.0108968
 0.0998334   0.995004          0   0.108605
         0          0          1    1.00106
         0          0          0          1

=== EXERCICE 3 (BONUS) : RÉGULATEUR PROPORTIONNEL ===

>>> Succès ! Cible atteinte en 3358 itérations !
Erreur finale : 0.00999636 m
Cible demandée   : [0.2 0.2]
Position finale  : [0.195626 0.208989]
==58585== 
==58585== HEAP SUMMARY:
==58585==     in use at exit: 2,840 bytes in 20 blocks
==58585==   total heap usage: 25,272 allocs, 25,252 frees, 3,651,982 bytes allocated
==58585== 
==58585== LEAK SUMMARY:
==58585==    definitely lost: 0 bytes in 0 blocks
==58585==    indirectly lost: 0 bytes in 0 blocks
==58585==      possibly lost: 0 bytes in 0 blocks
==58585==    still reachable: 2,840 bytes in 20 blocks
==58585==         suppressed: 0 bytes in 0 blocks
==58585== Reachable blocks (those to which a pointer was found) are not shown.
==58585== To see them, rerun with: --leak-check=full --show-leak-kinds=all
==58585== 
==58585== For lists of detected and suppressed errors, rerun with: -s
==58585== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)