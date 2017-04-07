#ifndef __GridParameters_hpp__
#define __GridParameters_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <typedef.hpp>
namespace SeqPP {
	class GridParameters{
	public:
	private:
		beacls::FloatVec min;
		beacls::FloatVec max;
		beacls::IntegerVec N;
	public:
		PREFIX_VC_DLL
		GridParameters(
			const beacls::FloatVec& min,
			const beacls::FloatVec& max,
			const beacls::IntegerVec& N
		) : min(min), max(max), N(N) {};

		PREFIX_VC_DLL
			~GridParameters() {};
		beacls::FloatVec get_min() const { return min; };
		beacls::FloatVec  get_max() const { return max; };
		beacls::IntegerVec get_N() const { return N; };

	private:
		/** @overload
		Disable operator=
		*/
		GridParameters& operator=(const GridParameters& rhs);
		/** @overload
		Disable copy constructor
		*/
		GridParameters(const GridParameters& rhs);
	};
};
#endif	/* __GridParameters_hpp__ */
