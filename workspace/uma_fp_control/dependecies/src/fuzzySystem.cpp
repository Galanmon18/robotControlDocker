#include <dependecies/fuzzySystem.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>

// Implementaciones de MembershipFunction
double TrapezoidalMF::evaluate(double x) const {
    if (x <= a || x >= d) return 0.0;
    if (x >= b && x <= c) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

double GaussianMF::evaluate(double x) const {
    return exp(-(x - mean) * (x - mean) / (2 * sigma * sigma));
}

double TriangularMF::evaluate(double x) const {
    if (x <= a || x >= c) return 0.0;
    if (x <= b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

// Implementaciones de LinguisticVariable
LinguisticVariable::LinguisticVariable(const std::string& name, double min, double max)
    : name(name), range_min(min), range_max(max) {}

LinguisticVariable::~LinguisticVariable() {
    for (auto mf : mfs) delete mf;
}

void LinguisticVariable::addMF(MembershipFunction* mf) {
    mfs.push_back(mf);
}

std::vector<double> LinguisticVariable::fuzzify(double crisp_value) const {
    std::vector<double> memberships;
    for (const auto& mf : mfs) {
        memberships.push_back(mf->evaluate(crisp_value));
    }
    return memberships;
}

// Implementaciones de FuzzySystem
FuzzySystem::~FuzzySystem() {
    for (auto input : inputs) delete input;
    delete output;
}

void FuzzySystem::addInput(LinguisticVariable* input) {
    inputs.push_back(input);
}

void FuzzySystem::setOutput(LinguisticVariable* out) {
    output = out;
}

void FuzzySystem::addRule(const FuzzyRule& rule) {
    rules.push_back(rule);
}

double FuzzySystem::evaluateCOG() {
    std::cout << "\n=== Starting COG Defuzzification ===" << std::endl;
    
    // Constantes para la discretización
    const int num_points = 121;
    const double rangeMin = output->getRangeMin();
    const double rangeMax = output->getRangeMax();
    const double step = (rangeMax - rangeMin) / (num_points - 1);
    
    std::vector<double> x(num_points);
    std::vector<double> aggregatedMF(num_points, 0.0);
    
    // Generar puntos x
    for (int i = 0; i < num_points; ++i) {
        x[i] = rangeMin + i * step;
    }
    
    // Para cada punto en el universo de discurso
    for (int i = 0; i < num_points; ++i) {
        double xi = x[i];
        
        // Para cada regla
        for (size_t ruleIdx = 0; ruleIdx < rules.size(); ++ruleIdx) {
            // Solo procesar si la regla tiene fuerza > 0
            if (rule_strengths[ruleIdx] > 0) {
                // Obtener el índice de la función de membresía de salida de la regla
                size_t outputIndex = rules[ruleIdx].getOutputIndex();
                
                // Obtener el valor de membresía para este punto
                double mfValue = output->getMembershipValue(xi, outputIndex);
                
                // Aplicar el peso de la regla
                double weightedValue = rule_strengths[ruleIdx] * mfValue;
                
                // Agregar usando max (s-norm)
                aggregatedMF[i] = std::max(aggregatedMF[i], weightedValue);
                
                // Debug opcional
                if (i == 60) { // punto medio para debug
                    std::cout << "Activated Rule " << ruleIdx << ": "
                              << "Antecedents: ";
                    const std::vector<int>& antecedents = rules[ruleIdx].getAntecedents();
                    for (const int ant : antecedents) {
                        std::cout << ant << " ";  // Imprime los índices de los antecedentes
                    }
                    std::cout << "Output Index: " << outputIndex
                              << ", Weighted rule_strengths: " << rule_strengths[ruleIdx] << std::endl;
                }
            }
        }
    }
    // Calcular centroide
    double numerator = 0.0;
    double denominator = 0.0;
    
    for (int i = 0; i < num_points; ++i) {
        numerator += x[i] * aggregatedMF[i];
        denominator += aggregatedMF[i];
    }
    
    if (denominator < 1e-10) {
        std::cout << "Warning: Denominator near zero in COG calculation" << std::endl;
        return 0.0;
    }
    
    double result = numerator / denominator;
    
    // Validar resultado
    if (result < rangeMin || result > rangeMax) {
        std::cout << "Warning: COG result outside valid range!" << std::endl;
        result = std::max(rangeMin, std::min(rangeMax, result));
    }
    
    std::cout << "Final COG result: " << result << std::endl;
    return result;
}

double FuzzySystem::evaluate(const std::vector<double>& input_values) {
    // Fuzzificación
    std::cout << "fuzzySystem X input_values: " << input_values[0] << " " << input_values[1] << " " << input_values[2] << std::endl;
    std::vector<std::vector<double>> fuzzified_inputs;
    for (size_t i = 0; i < inputs.size(); ++i) {
        fuzzified_inputs.push_back(inputs[i]->fuzzify(input_values[i]));
    }
    // Imprimir los nombres de los inputs
    std::cout << "Input Linguistic Variables:" << std::endl;
    for (const auto& input : inputs) {
        std::cout << "- " << input->getName() << std::endl;
    }

    // Imprimir los valores fuzzificados de cada input
    std::cout << "Fuzzified Inputs:" << std::endl;
    for (size_t i = 0; i < fuzzified_inputs.size(); ++i) {
        std::cout << inputs[i]->getName() << " memberships: ";
        for (const auto& value : fuzzified_inputs[i]) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    // Evaluación de reglas
    std::cout << "Rules Evaluation:" << std::endl;
    rule_strengths.clear(); // Initialize the rule_strengths vector
    for (size_t i = 0; i < rules.size(); ++i) {
        const auto& rule = rules[i];
        double strength = 1.0;
        
        std::cout << "Rule " << i + 1 << " antecedents: ";
        for (size_t j = 0; j < rule.antecedent_idx.size(); ++j) {
            std::cout << rule.antecedent_idx[j] << " ";
            strength = std::min(strength, fuzzified_inputs[j][rule.antecedent_idx[j]]);
        }
        strength *= rule.weight;
        rule_strengths.push_back(strength);
        
        std::cout << "| Weight: " << rule.weight << " | Strength: " << strength << std::endl;
    }
    double result = evaluateCOG();
    return result;
}