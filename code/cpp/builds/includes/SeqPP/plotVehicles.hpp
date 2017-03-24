#ifndef __plotVehicles_hpp__
#define __plotVehicles_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <cstddef>
#include <helperOC/helperOC_type.hpp>

namespace levelset {
	class HJI_Grid;
}
namespace SeqPP {
	class SPPPlane;

	/**
		@brief	Updates the plot in the SPP simulation

	*/
	PREFIX_VC_DLL
	bool plotVehicles(
		beacls::FloatVec& dst_hc,
		beacls::FloatVec& dst_ho,
		beacls::FloatVec& dst_hn,
		beacls::FloatVec& dst_ht,
		const std::vector<SPPPlane*>& Q,
		const beacls::IntegerVec& tInds,
		const levelset::HJI_Grid* g2D,
		const beacls::FloatVec& src_hc,
		const beacls::FloatVec& src_ho,
		const beacls::FloatVec& src_hn,
		const beacls::FloatVec& src_ht,
		const std::vector<beacls::FloatVec>& colors,
		const FLOAT_TYPE capture_radius
		);
};
#endif	/* __plotVehicles_hpp__ */
