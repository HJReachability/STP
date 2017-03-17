#include <vector>
#include <typedef.hpp>
#include <SeqPP/plotVehicles.hpp>
#include <SeqPP/SPPPlane/SPPPlane.hpp>
#include <functional>
#include <algorithm>
#include <macro.hpp>
bool SeqPP::plotVehicles(
#if 0
	beacls::FloatVec& dst_hc,
	beacls::FloatVec& dst_ho,
	beacls::FloatVec& dst_hn,
	beacls::FloatVec& dst_ht,
	const std::vector<SPPPlane*>& Q,
	const beacls::IntegerVec& tInds,
	const HJI_Grid* g2D,
	const beacls::FloatVec& src_hc,
	const beacls::FloatVec& src_ho,
	const beacls::FloatVec& src_hn,
	const beacls::FloatVec& src_ht,
	const std::vector<beacls::FloatVec>& colors,
	const FLOAT_TYPE capture_radius
#else
	beacls::FloatVec& ,
	beacls::FloatVec& ,
	beacls::FloatVec& ,
	beacls::FloatVec& ,
	const std::vector<SPPPlane*>& ,
	const beacls::IntegerVec& ,
	const HJI_Grid* ,
	const beacls::FloatVec& ,
	const beacls::FloatVec& ,
	const beacls::FloatVec& ,
	const beacls::FloatVec& ,
	const std::vector<beacls::FloatVec>& ,
	const FLOAT_TYPE 
#endif
) {
	return true;
}
