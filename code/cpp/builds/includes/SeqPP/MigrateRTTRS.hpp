#ifndef __MigrateRTTRS_hpp__
#define __MigrateRTTRS_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <typedef.hpp>
#include <vector>
#include <helperOC/helperOC_type.hpp>
class HJI_Grid;
namespace SeqPP {
	class MigrateRTTRS_impl;
	class RTTRS;
	/**
		@brief	
		@param	[in]	rttrs
		@param	[in]	target_g2D
		@param	[in]	R_augment
		@retval			migrated 2D RTTRS
	*/
	class MigrateRTTRS {
	private:
		MigrateRTTRS_impl* pimpl;
	public:
		PREFIX_VC_DLL
			MigrateRTTRS(
				const helperOC::ExecParameters& execParameters = helperOC::ExecParameters()
			);
		PREFIX_VC_DLL
			~MigrateRTTRS();
		PREFIX_VC_DLL
			HJI_Grid* operator()(
				beacls::FloatVec& rttrs2d,
				const RTTRS* rttrs,
				const FLOAT_TYPE R_augment
				);
	private:
		MigrateRTTRS& operator=(const MigrateRTTRS& rhs);
		MigrateRTTRS(const MigrateRTTRS& rhs);
	};
};
#endif	/* __MigrateRTTRS_hpp__ */
