#ifndef __SPPwDisturbanceRTT_hpp__
#define __SPPwDisturbanceRTT_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <cstdint>
#include <cstddef>
#include <SeqPP/SPPProblem/SPPProblemTypedef.hpp>
#include <helperOC/helperOC_type.hpp>
namespace SeqPP {
	class SPPProblem;
	/**
		@brief	Solves the entire SPP with disturbances problem using the RTT method
		@param	[in]	restart		set to true to restart and overwrite computation
		@param	[in]	checkPointType	Checkpoint type.
		@arg	CheckPointType_None	:			Don't save checkpoint files
		@arg	CheckPointType_Merged	:		Save checkpoint into a single file
		@arg	CheckPointType_Separated	:	Save checkpoint into vehicle separated files
		@param	[in]	save_brs1_file	Save BRS1 into files.
		@param	[in]	num_of_vehicles_to_computeNIRS	Number of vehicles to compute NIRS on this execution.
		@param	[in]	visualize	Visualize BRS1 and OptTraj mode.
		*/
	PREFIX_VC_DLL
		void SPPwDisturbanceRTT(
			const SeqPP::ProblemType problem_name,
			const SeqPP::SetupExtraArgs& setupExtraArgs,
			SeqPP::SPPProblem* SPPP = NULL,
			const bool lowprecision_obstacles = false,
			const bool low_memory = false,
			const helperOC::ExecParameters& execParameters = helperOC::ExecParameters(),
			const bool restart = true,
			const CheckPointType checkPointType = CheckPointType_Merged,
			const bool save_brs1_file = false,
			const size_t num_of_vehicles_to_computeNIRS = 0,
			const bool visualize = false,
			const beacls::IntegerVec& rttrs_gN = beacls::IntegerVec()
		);
};
#endif	/* __SPPwDisturbanceRTT_hpp__ */
