#include "parameter_estimator.h"
#include "assert.h"

void ParameterEstimator::init(){
}

double ParameterEstimator::get_probability(enum parameters parameter){
    if(normalized_probabilities.size() > (int)parameter)
        return normalized_probabilities[(int)parameter];
    else
        return -1;
}

void ParameterEstimator::operate() {
    cout << "pe operate" << endl;
    //normalize probability estimates
    double sum = 0;
#define P(aa, bb, cc, dd, ee) \
    const bb& probability_##cc = input_##cc.getValue(); \
    sum += probability_##cc;
#include "parameter_table.h"
#undef P
    normalized_probabilities.clear();
#define P(aa, bb, cc, dd, ee) \
    normalized_probabilities.push_back(probability_##cc/sum);
#include "parameter_table.h"
#undef P
}
