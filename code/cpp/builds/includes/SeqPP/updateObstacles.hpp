#ifndef __updateObstacles_hpp__
#define __updateObstacles_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <typedef.hpp>
#include <SeqPP/SPPProblem/SPPProblemTypedef.hpp>

namespace SeqPP {
	/**
		@brief	Gathers obstacles by combining obstacles in the field obs_type of each
				vehicle in the vehicles list
		@param	[inout]	obstacles		Updating obstacles
		@param	[in]	newObs_tau		tau for new obstacles
		@param	[in]	newObs			new obstacles
		@param	[in]	staticObs		Static obstacles
		@retval	true					Succeeded
		@retval	false					Failed
	*/
	PREFIX_VC_DLL
		bool updateObstacles(
			Obstacles& obstacles,
			const beacls::FloatVec& newObs_tau,
			const std::vector<beacls::FloatVec>& newObs,
			const beacls::FloatVec& staticObs = beacls::FloatVec()
		);
	/**
	@brief	Gathers obstacles by combining obstacles in the field obs_type of each
	vehicle in the vehicles list
	@param	[inout]	obstacles		Updating obstacles
	@param	[in]	newObs_tau		tau for new obstacles
	@param	[in]	newObs			new obstacles
	@param	[in]	staticObs		Static obstacles
	@retval	true					Succeeded
	@retval	false					Failed
	*/
	PREFIX_VC_DLL
		bool updateObstacles(
			Obstacles& obstacles,
			const beacls::FloatVec& newObs_tau,
			const std::vector<std::vector<int8_t>>& newObs,
			const std::vector<int8_t>& staticObs = std::vector<int8_t>()
		);
};
#endif	/* __updateObstacles_hpp__ */
