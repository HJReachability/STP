#include <vector>
#include <typedef.hpp>
#include <SeqPP/initRTT.hpp>
#include <SeqPP/SPPPlane/SPPPlane.hpp>
#include <SeqPP/SPPProblem/SPPProblem.hpp>
#include <SeqPP/SPPProblem/RTTRS.hpp>
#include <helperOC/DynSys/PlaneCAvoid/PlaneCAvoid.hpp>
#include <levelset/InitialConditions/BasicShapes/ShapeCylinder.hpp>
#include <functional>
#include <algorithm>
#include <macro.hpp>

namespace SeqPP {
	static SeqPP::SPPPlane* initOneRTT(
		const beacls::FloatVec& initStates,
		const beacls::FloatVec& targetCenters,
		const FLOAT_TYPE targetR,
		const FLOAT_TYPE targetRsmall,
		const FLOAT_TYPE wMaxA,
		const FLOAT_TYPE wMaxB,
		const beacls::FloatVec& vRangeA,
		const beacls::FloatVec& vRangeB,
		const beacls::FloatVec& dMax,
		const HJI_Grid* g,
		const helperOC::ExecParameters& execParameters = helperOC::ExecParameters()
	);
};

static SeqPP::SPPPlane* SeqPP::initOneRTT(
	const beacls::FloatVec& initStates,
	const beacls::FloatVec& targetCenters,
	const FLOAT_TYPE targetR,
	const FLOAT_TYPE targetRsmall,
	const FLOAT_TYPE wMaxA,
	const FLOAT_TYPE wMaxB,
	const beacls::FloatVec& vRangeA,
	const beacls::FloatVec& vRangeB,
	const beacls::FloatVec& dMax,
	const HJI_Grid* g,
	const helperOC::ExecParameters& execParameters
) {
	SPPPlane* sPPPlane = new SPPPlane(initStates, wMaxA, vRangeA, dMax, execParameters);
	beacls::FloatVec center;
	center.insert(center.end(), targetCenters.cbegin(), targetCenters.cend());
	center.push_back(0);
	BasicShape* targetRShape = new ShapeCylinder(beacls::IntegerVec{2}, center, targetR);

	beacls::FloatVec target;
	targetRShape->execute(g, target);
	if (targetRShape) delete targetRShape;
	sPPPlane->set_target(target);

	BasicShape* targetRsmallShape = new ShapeCylinder(beacls::IntegerVec{2}, center, targetRsmall);

	beacls::FloatVec targetsm;
	targetRsmallShape->execute(g, targetsm);
	if (targetRsmallShape) delete targetRsmallShape;

	sPPPlane->set_targetsm(targetsm);

	sPPPlane->set_targetCenter(targetCenters);
	sPPPlane->set_targetR(targetR);
	sPPPlane->set_targetRsmall(targetRsmall);

	//!< Reserved control authorities
	beacls::FloatVec vReserved(vRangeB.size());
	std::transform(vRangeB.cbegin(), vRangeB.cend(), vRangeA.cbegin(), vReserved.begin(), std::minus<FLOAT_TYPE>());
	sPPPlane->set_vReserved(vReserved);
	sPPPlane->set_wReserved(wMaxB - wMaxA);

	return sPPPlane;
}


SeqPP::SPPPlane* SeqPP::initOneRTT(
	const SPPProblem* sppp,
	const RTTRS* rttrs,
	const size_t vehicle_index,
	const helperOC::ExecParameters& execParameters
) {
	const beacls::FloatVec& initState = sppp->get_initState(vehicle_index);
	const beacls::FloatVec& targetCenter = sppp->get_targetCenter(vehicle_index);
	const FLOAT_TYPE targetR = sppp->get_targetR();
	const FLOAT_TYPE targetRsmall = targetR - rttrs->get_trackingRadius();
	const PlaneCAvoid* dynSys = rttrs->get_dynSys();
	const FLOAT_TYPE wMaxA = dynSys->get_wMaxA();
	const FLOAT_TYPE wMaxB = dynSys->get_wMaxB();
	const beacls::FloatVec& vRangeA = dynSys->get_vRangeA();
	const beacls::FloatVec& vRangeB = dynSys->get_vRangeB();
	const beacls::FloatVec& dMax = dynSys->get_dMaxA();
	const HJI_Grid* g = sppp->get_g();

	return initOneRTT(initState, targetCenter, targetR, targetRsmall, wMaxA, wMaxB, vRangeA, vRangeB, dMax, g, execParameters);
}
void SeqPP::initRTT(
	std::vector<SPPPlane*>& Q,
	const SPPProblem* sppp,
	const RTTRS* rttrs,
	const helperOC::ExecParameters& execParameters
) {
	const std::vector<beacls::FloatVec>& initStates = sppp->get_initStates();
	const std::vector<beacls::FloatVec>& targetCenters = sppp->get_targetCenters();
	const size_t numVeh = initStates.size();
	const FLOAT_TYPE targetR = sppp->get_targetR();
	const FLOAT_TYPE targetRsmall = targetR - rttrs->get_trackingRadius();
	const PlaneCAvoid* dynSys = rttrs->get_dynSys();
	const FLOAT_TYPE wMaxA = dynSys->get_wMaxA();
	const FLOAT_TYPE wMaxB = dynSys->get_wMaxB();
	const beacls::FloatVec& vRangeA = dynSys->get_vRangeA();
	const beacls::FloatVec& vRangeB = dynSys->get_vRangeB();
	const beacls::FloatVec& dMax = dynSys->get_dMaxA();
	const HJI_Grid* g = sppp->get_g();
	Q.resize(numVeh);
	std::transform(initStates.cbegin(), initStates.cend(), targetCenters.cbegin(), Q.begin(), 
		[&g, &wMaxA, &wMaxB, &vRangeA, &vRangeB, &dMax, &targetR, &targetRsmall, &execParameters](const auto& lhs, const auto& rhs) {
		return initOneRTT(lhs, rhs, targetR, targetRsmall, wMaxA, wMaxB, vRangeA, vRangeB, dMax, g, execParameters);
	});
}
