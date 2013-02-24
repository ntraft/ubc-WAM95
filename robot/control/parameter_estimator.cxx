//follows example of barrett::systems::TupleGrouper

#ifndef PARAMETER_ESTIMATOR_H_ 
#define PARAMETER_ESTIMATOR_H_ 

#include "stdheader.h"

enum parameters{
#define X(aa, bb, cc, dd, ee) \
        enum_##cc,
#include "parameter_table.h"
#undef X
    NUM_PARAMETERS
};

template <
	typename T0 = boost::tuples::null_type,
	typename T1 = boost::tuples::null_type,
	typename T2 = boost::tuples::null_type,
	typename T3 = boost::tuples::null_type,
	typename T4 = boost::tuples::null_type,
	typename T5 = boost::tuples::null_type,
	typename T6 = boost::tuples::null_type,
	typename T7 = boost::tuples::null_type,
	typename T8 = boost::tuples::null_type,
	typename T9 = boost::tuples::null_type>
class TupleGrouper : public System,
					 public SingleOutput<
						 boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> > {
public:
	typedef boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> tuple_type;
	static const size_t NUM_INPUTS = boost::tuples::length<tuple_type>::value;

private:	systems::detail::InputHolder<NUM_INPUTS, T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> inputs;

public:
	TupleGrouper(const std::string& sysName = "TupleGrouper") :
		System(sysName), SingleOutput<tuple_type>(this), inputs(this) {}
	virtual ~TupleGrouper() { mandatoryCleanUp(); }

	template<size_t N>
	Input<typename boost::tuples::element<N, tuple_type>::type >& getInput() {
		return inputs.getInput<N>();
	}

protected:
	virtual void operate() {
        boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> values = inputs.getValues();
        //normalize estimation
        double sum = 0;
#define X(aa, bb, cc, dd, ee) \
        sum += boost::get<ee>(values);
#include "parameter_table.h"
#undef X
#define X(aa, bb, cc, dd, ee) \
        boost::get<ee>(values) = boost::get<ee>(values)/sum;
#include "parameter_table.h"
#undef X
		this->outputValue->setData( &values );
	}

private:
	DISALLOW_COPY_AND_ASSIGN(TupleGrouper);

public:
	// To be safe, assume that at least one of the input types needs to be aligned.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
