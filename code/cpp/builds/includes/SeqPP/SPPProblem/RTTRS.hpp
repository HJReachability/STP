#ifndef __RTTRS_hpp__
#define __RTTRS_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <cstring>
#include <iostream>
#include <typedef.hpp>
namespace helperOC {
	class PlaneCAvoid;
}
namespace levelset {
	class HJI_Grid;
}
namespace SeqPP {

	class RTTRS
	{
	public:
	private:
		FLOAT_TYPE trackingRadius;
		levelset::HJI_Grid* g;
		beacls::FloatVec data;
		std::vector<beacls::FloatVec> Deriv;
		helperOC::PlaneCAvoid* dynSys;
	public:
		RTTRS();
		~RTTRS();
		RTTRS* clone() const;
		void set_trackingRadius(const FLOAT_TYPE v);
		void set_g(const levelset::HJI_Grid* v);
		void set_data(const beacls::FloatVec& v);
		void set_Deriv(const std::vector<beacls::FloatVec>& v);
		void set_dynSys(helperOC::PlaneCAvoid* v);
		FLOAT_TYPE get_trackingRadius() const;
		levelset::HJI_Grid* get_g() const;
		const beacls::FloatVec& get_data() const;
		const std::vector<beacls::FloatVec>& get_Deriv() const;
		helperOC::PlaneCAvoid* get_dynSys() const;

		bool load(
			beacls::MatVariable* variable_ptr
		);
		bool save(
			beacls::MatVariable* variable_ptr
		) const;
	private:
		/** @overload
		Disable operator=
		*/
		RTTRS& operator=(const RTTRS& rhs);
		/** @overload
		Disable copy constructor
		*/
		RTTRS(const RTTRS& rhs);
	};
};

#endif	/* __RTTRS_hpp__ */
