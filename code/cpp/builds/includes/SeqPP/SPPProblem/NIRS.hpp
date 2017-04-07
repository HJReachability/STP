#ifndef __NIRS_hpp__
#define __NIRS_hpp__

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
#include <limits>
#include <helperOC/helperOC_type.hpp>
namespace SeqPP {
	class SPPPlane;

	class NIRS
	{
	public:
	private:
		std::vector<SPPPlane*> Q;
		bool to_save;
	public:
		NIRS();
		NIRS(const size_t num_of_vehicles);
		~NIRS();
		bool get_to_save() const;
		void set_to_save(const bool v);
		void initializeQ(const size_t num_of_vehicles);
		void set_Q(const std::vector<SPPPlane*>& v);
		void set_Vehicle(SPPPlane* v, const size_t veh);
		void clear_Vehicle(const size_t veh);
		std::vector<SPPPlane*> get_Q() const;
		SPPPlane* get_Vehicle(const size_t veh) const;
		size_t get_num_of_vehicles() const;
		bool load(
			const helperOC::ExecParameters& execParameters,
			beacls::MatFStream* fs
		);
		bool save(
			beacls::MatFStream* fs,
			const bool clear = false,
			const size_t num_of_keeps = 0
		);
	private:
		/** @overload
		Disable operator=
		*/
		NIRS& operator=(const NIRS& rhs);
		/** @overload
		Disable copy constructor
		*/
		NIRS(const NIRS& rhs);
	};
};

#endif	/* __NIRS_hpp__ */
