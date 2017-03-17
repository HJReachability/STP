#ifndef __VehicleParameters_hpp__
#define __VehicleParameters_hpp__

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
	class VehicleParameters{
	public:
	private:
		beacls::FloatVec vRangeA;
		FLOAT_TYPE wMaxA;
		beacls::FloatVec dMaxA;
	public:
		PREFIX_VC_DLL
		VehicleParameters(
			const beacls::FloatVec& vRangeA,
			const FLOAT_TYPE wMaxA,
			const beacls::FloatVec& dMaxA
		) : vRangeA(vRangeA), wMaxA(wMaxA), dMaxA(dMaxA) {};

		PREFIX_VC_DLL
			~VehicleParameters() {};
		beacls::FloatVec get_vRangeA() const { return vRangeA; };
		FLOAT_TYPE get_wMaxA() const { return wMaxA; };
		beacls::FloatVec get_dMaxA() const { return dMaxA; };

	private:
		/** @overload
		Disable operator=
		*/
		VehicleParameters& operator=(const VehicleParameters& rhs);
		/** @overload
		Disable copy constructor
		*/
		VehicleParameters(const VehicleParameters& rhs);
	};
};
#endif	/* __VehicleParameters_hpp__ */
