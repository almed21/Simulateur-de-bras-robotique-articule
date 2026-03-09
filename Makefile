# ==========================================
# Configuration du compilateur et des flags
# ==========================================
CXX = g++
# On utilise pkg-config pour récupérer automatiquement les bons chemins pour Eigen et Pinocchio
PKG_CFLAGS = $(shell pkg-config --cflags pinocchio eigen3)
PKG_LIBS = $(shell pkg-config --libs pinocchio eigen3)

# Flags de compilation demandés (-std=c++17) + flags de warnings standards + inclusion du dossier include/
CXXFLAGS = -std=c++17 -Wall -Wextra -Iinclude $(PKG_CFLAGS)
# Flags pour l'édition de liens (librairies)
LDFLAGS = $(PKG_LIBS)

# Flags spécifiques pour GTest (tests unitaires)
TEST_LIBS = -lgtest -lgtest_main -pthread

# ==========================================
# Définition des fichiers et dossiers
# ==========================================
BUILD_DIR = build
SRC_DIR = src
TEST_DIR = tests

# On sépare le main.cpp des autres sources (nécessaire car GTest a son propre main)
SRC_MAIN = $(SRC_DIR)/main.cpp
SRC_LIB = $(filter-out $(SRC_MAIN), $(wildcard $(SRC_DIR)/*.cpp))
SRC_TESTS = $(wildcard $(TEST_DIR)/*.cpp)

# Fichiers objets correspondants générés dans build/
OBJ_MAIN = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRC_MAIN))
OBJ_LIB = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRC_LIB))
OBJ_TESTS = $(patsubst $(TEST_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRC_TESTS))

# Noms des exécutables finaux
TARGET_SIMU = $(BUILD_DIR)/simulateur
TARGET_TESTS = $(BUILD_DIR)/run_tests

# ==========================================
# Règles de compilation
# ==========================================

# Règle par défaut : compile le simulateur ET les tests
all: $(TARGET_SIMU) $(TARGET_TESTS)

# Compilation de l'exécutable principal "simulateur"
$(TARGET_SIMU): $(OBJ_LIB) $(OBJ_MAIN)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "=> Exécutable principal généré : $@"

# Compilation de l'exécutable des tests (GTest)
$(TARGET_TESTS): $(OBJ_LIB) $(OBJ_TESTS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(TEST_LIBS)
	@echo "=> Exécutable de tests généré : $@"

# Règle générique pour compiler les .cpp du dossier src/ en .o
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Règle générique pour compiler les .cpp du dossier tests/ en .o
$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ==========================================
# Règles utilitaires
# ==========================================

# Lancer Valgrind exactement comme demandé dans le sujet (Séance 3, question 4)
valgrind: $(TARGET_SIMU)
	valgrind --leak-check=full --track-origins=yes ./$(TARGET_SIMU)

# Nettoyer les fichiers compilés
clean:
	rm -rf $(BUILD_DIR)/*
	@echo "=> Dossier $(BUILD_DIR)/ nettoyé."

.PHONY: all clean valgrind