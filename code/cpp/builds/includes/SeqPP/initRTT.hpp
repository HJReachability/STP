#ifndef __initRTT_hpp__
#define __initRTT_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <helperOC/helperOC_type.hpp>

namespace SeqPP {
	class SPPProblem;
	class RTTRS;
	class SPPPlane;

	/**
		@brief	Initializes helperOC::Plane objects for the RTT method SPP problem
		@param	[out]	Q			Plane dynamics
		@param	[in]	SPPProblem
		@param	[in]	RTRRS
	*/
	void initRTT(
		std::vector<SPPPlane*>& Q,
		const SPPProblem* sppp,
		const RTTRS* rttrs,
		const helperOC::ExecParameters& execParameters = helperOC::ExecParameters()
	);
	/**
	@brief	Initializes One helperOC::Plane object for the RTT method SPP problem
	@param	[in]	SPPProblem
	@param	[in]	RTRRS
	@param	[in]	vehicle_index	Index of the vehicle
	@retval							Initialized plane dynamics
	*/
	SPPPlane* initOneRTT(
		const SPPProblem* sppp,
		const RTTRS* rttrs,
		const size_t vehicle_index,
		const helperOC::ExecParameters& execParameters = helperOC::ExecParameters()
	);

};
#endif	/* __initRTT_hpp__ */
