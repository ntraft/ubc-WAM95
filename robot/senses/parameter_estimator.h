#ifndef PARAMETER_ESTIMATOR_H_ 
#define PARAMETER_ESTIMATOR_H_ 

#include "stdheader.h"


class ParameterEstimator : public System {
						 
public:
#define P(aa, bb, cc, dd, ee) \
        Input<bb> input_##cc; \
        //Output<bb> output_##cc;
#include "parameter_table.h"
#undef P

protected:
#define P(aa, bb, cc, dd, ee) \
    //Output<bb>::Value* output_value_##cc;
#include "parameter_table.h"
#undef P
vector<double> normalized_probabilities;

public:
	ParameterEstimator(const std::string& sysName = "ParameterEstimator") :
#define P(aa, bb, cc, dd, ee) \
        input_##cc(this), \
        //Output<bb> output_##cc;
#include "parameter_table.h"
#undef P
		System(sysName)
        //output(this, &output_value) 
        {}
	virtual ~ParameterEstimator() { mandatoryCleanUp(); }

    void init();
    double get_probability(enum parameters);

    //Output<double> output;
protected:
    //Output<double>::Value* output_value;
	virtual void operate(); 
private:
	DISALLOW_COPY_AND_ASSIGN(ParameterEstimator);

public:
	// To be safe, assume that at least one of the input types needs to be aligned.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
