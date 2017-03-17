#ifndef __CARS_hpp__
#define __CARS_hpp__

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
class PlaneCAvoid;
class HJI_Grid;
namespace SeqPP {

	class CARS
	{
	public:
	private:
		HJI_Grid* g;
		std::vector<beacls::FloatVec> datas;
		beacls::FloatVec tau;
		PlaneCAvoid* dynSys;
	public:
		CARS();
		~CARS();
		CARS* clone() const;
		void set_g(const HJI_Grid* v);
		void set_data(const std::vector<beacls::FloatVec>& v);
		void set_tau(const beacls::FloatVec& v);
		void set_dynSys(PlaneCAvoid* v);
		HJI_Grid* get_g() const;
		const std::vector<beacls::FloatVec>& get_data() const;
		const beacls::FloatVec& get_tau() const;
		PlaneCAvoid* get_dynSys() const;

		bool load(
			beacls::MatFStream* fs,
			beacls::MatVariable* variable_ptr = NULL
		);
		bool save(
			beacls::MatFStream* fs,
			beacls::MatVariable* variable_ptr = NULL
		) const;
	private:
		/** @overload
		Disable operator=
		*/
		CARS& operator=(const CARS& rhs);
		/** @overload
		Disable copy constructor
		*/
		CARS(const CARS& rhs);
	};
};

#endif	/* __CARS_hpp__ */
