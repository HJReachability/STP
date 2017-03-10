#include <functional>
#include <algorithm>
#include <macro.hpp>
#include <SeqPP/MigrateRTTRS.hpp>
#include <SeqPP/SPPProblem/RTTRS.hpp>
#include <helperOC/ValFuncs/proj.hpp>
#include <levelset/Grids/HJI_Grid.hpp>
#include <helperOC/Grids/createGrid.hpp>
#include <helperOC/Grids/migrateGrid.hpp>
#include <helperOC/ValFuncs/AddCRadius.hpp>
#include "MigrateRTTRS_impl.hpp"

SeqPP::MigrateRTTRS_impl::MigrateRTTRS_impl(
	const helperOC::ExecParameters& execParameters
) {
	addCRadius = new helperOC::AddCRadius(execParameters);
}
SeqPP::MigrateRTTRS_impl::~MigrateRTTRS_impl() {
	if (addCRadius) delete addCRadius;
}
SeqPP::MigrateRTTRS::MigrateRTTRS(
	const helperOC::ExecParameters& execParameters
) {
	pimpl = new MigrateRTTRS_impl(execParameters);
}
SeqPP::MigrateRTTRS::~MigrateRTTRS() {
	if (pimpl) delete pimpl;
}

HJI_Grid* SeqPP::MigrateRTTRS_impl::operator()(
	beacls::FloatVec& rttrs2d,
	const RTTRS* rttrs,
	const FLOAT_TYPE R_augment
) {
	//!< Project RTTRS into 2D
	HJI_Grid* rttrs_g2D;
	const beacls::FloatVec& rttrs_data = rttrs->get_data();
	beacls::FloatVec minus_rttrs_data(rttrs_data.size());
	std::transform(rttrs_data.cbegin(), rttrs_data.cend(), minus_rttrs_data.begin(), std::negate<FLOAT_TYPE>());
	rttrs_g2D = helperOC::proj(rttrs2d, rttrs->get_g(), minus_rttrs_data, beacls::IntegerVec{0, 0, 1});

	//!< Create slightly bigger grid to augment the RTTRS
	const beacls::FloatVec& rttrs_g2D_mins = rttrs_g2D->get_mins();
	beacls::FloatVec g2D_min(rttrs_g2D_mins.size());
	std::transform(rttrs_g2D_mins.cbegin(), rttrs_g2D_mins.cend(), g2D_min.begin(), [R_augment](const auto& rhs) { return rhs - 2 * R_augment; });

	const beacls::FloatVec& rttrs_g2D_maxs = rttrs_g2D->get_maxs();
	beacls::FloatVec g2D_max(rttrs_g2D_maxs.size());
	std::transform(rttrs_g2D_maxs.cbegin(), rttrs_g2D_maxs.cend(), g2D_max.begin(), [R_augment](const auto& rhs) { return rhs + 2 * R_augment; });

	beacls::IntegerVec g2D_N(g2D_max.size());
	const beacls::IntegerVec& Ns = rttrs_g2D->get_Ns();
	for (size_t dimension = 0; dimension < g2D_max.size(); ++dimension) {
		g2D_N[dimension] = static_cast<size_t>(std::ceil(
			(g2D_max[dimension] - g2D_min[dimension]) /
			(rttrs_g2D_maxs[dimension] - rttrs_g2D_mins[dimension]) * Ns[dimension]));
	}
	HJI_Grid* g2D = createGrid(g2D_min, g2D_max, g2D_N);

	//!< Migrate RTTRS set
	helperOC::migrateGrid(rttrs2d, rttrs_g2D, rttrs2d, g2D);
	addCRadius->operator()(rttrs2d, g2D, rttrs2d, R_augment);
	if (rttrs_g2D) delete rttrs_g2D;
	return g2D;
}
HJI_Grid* SeqPP::MigrateRTTRS::operator()(
	beacls::FloatVec& rttrs2d,
	const RTTRS* rttrs,
	const FLOAT_TYPE R_augment
	) {
	if (pimpl) return pimpl->operator()(rttrs2d, rttrs, R_augment);
	return NULL;
}