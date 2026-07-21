#ifndef FUZZY_SYSTEM_H
#define FUZZY_SYSTEM_H

#include <vector>
#include <string>
#include <memory>

// Clase base para funciones de membresía
class MembershipFunction {
protected:
    std::string name;
public:
    MembershipFunction(const std::string& n) : name(n) {}
    virtual double evaluate(double x) const = 0;
    virtual ~MembershipFunction() = default;
};

// Implementación de función trapezoidal
class TrapezoidalMF : public MembershipFunction {
    double a, b, c, d;
public:
    TrapezoidalMF(const std::string& name, double a, double b, double c, double d)
        : MembershipFunction(name), a(a), b(b), c(c), d(d) {}

    double evaluate(double x) const override;
};

// Implementación de función gaussiana
class GaussianMF : public MembershipFunction {
    double sigma, mean;
public:
    GaussianMF(const std::string& name, double sigma, double mean)
        : MembershipFunction(name), sigma(sigma), mean(mean) {}

    double evaluate(double x) const override;
};

// Implementación de función triangular
class TriangularMF : public MembershipFunction {
    double a, b, c;
public:
    TriangularMF(const std::string& name, double a, double b, double c)
        : MembershipFunction(name), a(a), b(b), c(c) {}

    double evaluate(double x) const override;
};

// Clase para variables lingüísticas
class LinguisticVariable {
    std::string name;
    double range_min, range_max;
    std::vector<MembershipFunction*> mfs;

public:
    LinguisticVariable(const std::string& name, double min, double max);
    ~LinguisticVariable();
    void addMF(MembershipFunction* mf);
    std::vector<double> fuzzify(double crisp_value) const;
    const std::string& getName() const { return name; }
    double getRangeMax() const { return range_max; }
    double getRangeMin() const { return range_min; }
    double getMembershipValue(double x) const {
        double max_membership = 0.0;
        for (const auto& mf : mfs) {
            max_membership = std::max(max_membership, mf->evaluate(x));
        }
        return max_membership;
    }
    double getMembershipValue(double x, size_t index) const {
        if (index < mfs.size()) {
            return mfs[index]->evaluate(x);
        }
        return 0.0;
    }
};

// Estructura para reglas difusas
struct FuzzyRule {
    std::vector<int> antecedent_idx;
    int consequent_idx;
    double weight;

    FuzzyRule(const std::vector<int>& ant, int cons, double w = 1.0)
        : antecedent_idx(ant), consequent_idx(cons), weight(w) {}
    size_t getOutputIndex() const {
        return consequent_idx;  // o como se llame tu variable que almacena el índice de salida
    }
    std::vector<int> getAntecedents() const {
        return antecedent_idx;  // o como se llame tu variable que almacena el índice de salida
    }
};

enum class DefuzzificationMethod {
    COG,
    MOM
};

// Clase principal del sistema difuso
class FuzzySystem {
    std::vector<LinguisticVariable*> inputs;
    LinguisticVariable* output;
    std::vector<FuzzyRule> rules;

public:
    FuzzySystem() : output(nullptr) {}
    ~FuzzySystem();

    std::vector<double> rule_strengths;
    
    void addInput(LinguisticVariable* input);
    void setOutput(LinguisticVariable* output);
    void addRule(const FuzzyRule& rule);
    double evaluate(const std::vector<double>& input_values);
    double evaluateCOG();
};

// Función para crear el sistema específico RHO
FuzzySystem* createFisRHO();

#endif