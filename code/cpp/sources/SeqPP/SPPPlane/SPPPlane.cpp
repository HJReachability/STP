#include <vector>
#include <sstream>
#include <typeinfo>
#include <functional>
#include <typedef.hpp>
#include <SeqPP/SPPPlane/SPPPlane.hpp>
#include <SeqPP/SPPProblem/SPPProblem.hpp>
#include <SeqPP/SPPProblem/CARS.hpp>
#include <SeqPP/SPPProblem/RTTRS.hpp>
#include <SeqPP/MigrateRTTRS.hpp>
#include <levelset/Grids/HJI_Grid.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/ValFuncs/HJIPDE.hpp>
#include <helperOC/helperOC_type.hpp>
#include <helperOC/ComputeOptTraj.hpp>
#include <helperOC/ValFuncs/rotateData.hpp>
#include <helperOC/ValFuncs/shiftData.hpp>
#include <helperOC/Grids/migrateGrid.hpp>
#include <helperOC/Grids/shiftGrid.hpp>
#if defined(FILESYSTEM)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif
#ifdef _OPENMP
#include <omp.h>
#endif
SeqPP::SPPPlane::SPPPlane(
	const beacls::FloatVec& x,
	const FLOAT_TYPE wMax,
	const beacls::FloatVec& vrange,
	const beacls::FloatVec& dMax,
	const helperOC::ExecParameters& execParameters
) : Plane(x, wMax, vrange, dMax), 
	vReserved(0),
	wReserved(0),
	v(0),
	BRS1(0),
	BRS1_tau(0),
	nomTraj(0),
	nomTraj_tau(0),
	nomTraj_AR(0) ,
	nomTraj_AR_tau(0),
	obsForRTT(0),
	obsForRTT_s8(0),
	obsForRTT_tau(0),
	obsForIntr(0),
	obsForIntr_tau(0),
	FRS1(0),
	FRS1_tau(0),
	FRS1_g(0),
	obs2D(0),
	obs2D_tau(0),
	target(0),
	targetsm(0),
	targetCenter(0),
	targetR(0),
	targetRsmall(0),
	replan(false),
	tauBR(0),
	tauAR(0),
	tau(0),
	execParameters(execParameters),
	to_save(false)
{}
SeqPP::SPPPlane::SPPPlane(
	const helperOC::ExecParameters& execParameters,
	beacls::MatFStream* fs,
	beacls::MatVariable* variable_ptr
) :
	Plane(fs, variable_ptr),
	vReserved(beacls::FloatVec()),
	wReserved(0),
	v(0),
	BRS1(std::vector<beacls::FloatVec>()),
	BRS1_tau(beacls::FloatVec()),
	nomTraj(std::vector<beacls::FloatVec>()),
	nomTraj_tau(beacls::FloatVec()),
	nomTraj_AR(std::vector<beacls::FloatVec>()),
	nomTraj_AR_tau(beacls::FloatVec()),
	obsForRTT(std::vector<beacls::FloatVec>()),
	obsForRTT_s8(std::vector<std::vector<int8_t>>()),
	obsForRTT_tau(beacls::FloatVec()),
	obsForIntr(std::vector<beacls::FloatVec>()),
	obsForIntr_tau(beacls::FloatVec()),
	FRS1(std::vector<beacls::FloatVec>()),
	FRS1_tau(beacls::FloatVec()),
	FRS1_g(0),
	obs2D(std::vector<beacls::FloatVec>()),
	obs2D_tau(beacls::FloatVec()),
	target(beacls::FloatVec()),
	targetsm(beacls::FloatVec()),
	targetCenter(beacls::FloatVec()),
	targetR(0),
	targetRsmall(0),
	replan(false),
	tauBR(beacls::FloatVec()),
	tauAR(beacls::FloatVec()),
	tau(beacls::FloatVec()),
	execParameters(execParameters),
	to_save(false) {
	beacls::IntegerVec dummy;
	load_vector(vReserved, std::string("vReserved"), dummy, true, fs, variable_ptr);
	load_value(wReserved, std::string("wReserved"), true, fs, variable_ptr);

	load_value(v, std::string("v"), true, fs, variable_ptr);

	load_vector_of_vectors(BRS1, std::string("BRS1"), dummy, true, fs, variable_ptr);
	load_vector(BRS1_tau, std::string("BRS1_tau"), dummy, true, fs, variable_ptr);

	load_vector_of_vectors(nomTraj, std::string("nomTraj"), dummy, true, fs, variable_ptr);
	load_vector(nomTraj_tau, std::string("nomTraj_tau"), dummy, true, fs, variable_ptr);

	load_vector_of_vectors(nomTraj_AR, std::string("nomTraj_AR"), dummy, true, fs, variable_ptr); //!< Nominal trajectory after replanning
	load_vector(nomTraj_AR_tau, std::string("nomTraj_AR_tau"), dummy, true, fs, variable_ptr);

	load_vector_of_vectors(obsForRTT, std::string("obsForRTT"), dummy, true, fs, variable_ptr);
	load_vector_of_vectors(obsForRTT_s8, std::string("obsForRTT_s8"), dummy, true, fs, variable_ptr);
	load_vector(obsForRTT_tau, std::string("obsForRTT_tau"), dummy, true, fs, variable_ptr);

	load_vector_of_vectors(obsForIntr, std::string("obsForIntr"), dummy, true, fs, variable_ptr);
	load_vector(obsForIntr_tau, std::string("obsForIntr_tau"), dummy, true, fs, variable_ptr);

	load_vector_of_vectors(FRS1, std::string("FRS1"), dummy, true, fs, variable_ptr);
	load_vector(FRS1_tau, std::string("FRS1_tau"), dummy, true, fs, variable_ptr);
	load_value(FRS1_g, std::string("FRS1_g"), true, fs, variable_ptr);

	load_vector_of_vectors(obs2D, std::string("obs2D"), dummy, true, fs, variable_ptr);
	load_vector(obs2D_tau, std::string("obs2D_tau"), dummy, true, fs, variable_ptr);

	load_vector(target, std::string("target"), dummy, true, fs, variable_ptr);
	load_vector(targetsm, std::string("targetsm"), dummy, true, fs, variable_ptr);
	load_vector(targetCenter, std::string("targetCenter"), dummy, true, fs, variable_ptr);
	load_value(targetR, std::string("targetR"), true, fs, variable_ptr);
	load_value(targetRsmall, std::string("targetRsmall"), true, fs, variable_ptr);

	//!< Replanning
	load_value(replan, std::string("replan"), true, fs, variable_ptr); //!< Whether replan is done
	load_vector(tauBR, std::string("tauBR"), dummy, true, fs, variable_ptr);
	load_vector(tauAR, std::string("tauAR"), dummy, true, fs, variable_ptr);
	load_vector(tau, std::string("tau"), dummy, true, fs, variable_ptr);
}
SeqPP::SPPPlane::~SPPPlane() {
}
bool SeqPP::SPPPlane::operator==(const SPPPlane& rhs) const {
	if (this == &rhs) return true;
	else if (!DynSys::operator==(rhs)) return false;
	//!< If tracking 3D trajectory
	else if (wReserved != rhs.wReserved) return false;

	else if (v != rhs.v) return false; //!< If tracking 2D trajectory
	else if (FRS1_g != rhs.FRS1_g) return false;
	else if (targetR != rhs.targetR) return false;
	else if (targetRsmall != rhs.targetRsmall) return false;

	else if (replan != rhs.replan) return false; //!< Whether replan is done
	else if (!execParameters.operator==(rhs.execParameters)) return false;
	else if (to_save != rhs.to_save) return false;
	else if ((target.size() != rhs.target.size()) || !std::equal(target.cbegin(), target.cend(), rhs.target.cbegin())) return false;
	else if ((targetsm.size() != rhs.targetsm.size()) || !std::equal(targetsm.cbegin(), targetsm.cend(), rhs.targetsm.cbegin())) return false;
	else if ((targetCenter.size() != rhs.targetCenter.size()) || !std::equal(targetCenter.cbegin(), targetCenter.cend(), rhs.targetCenter.cbegin())) return false;

	else if ((vReserved.size() != rhs.vReserved.size()) || !std::equal(vReserved.cbegin(), vReserved.cend(), rhs.vReserved.cbegin())) return false;
	else if ((nomTraj_tau.size() != rhs.nomTraj_tau.size()) || !std::equal(nomTraj_tau.cbegin(), nomTraj_tau.cend(), rhs.nomTraj_tau.cbegin())) return false;
	else if ((BRS1_tau.size() != rhs.BRS1_tau.size()) || !std::equal(BRS1_tau.cbegin(), BRS1_tau.cend(), rhs.BRS1_tau.cbegin())) return false;
	else if ((nomTraj_AR_tau.size() != rhs.nomTraj_AR_tau.size()) || !std::equal(nomTraj_AR_tau.cbegin(), nomTraj_AR_tau.cend(), rhs.nomTraj_AR_tau.cbegin())) return false;
	else if ((obsForRTT_tau.size() != rhs.obsForRTT_tau.size()) || !std::equal(obsForRTT_tau.cbegin(), obsForRTT_tau.cend(), rhs.obsForRTT_tau.cbegin())) return false;
	else if ((obsForIntr_tau.size() != rhs.obsForIntr_tau.size()) || !std::equal(obsForIntr_tau.cbegin(), obsForIntr_tau.cend(), rhs.obsForIntr_tau.cbegin())) return false;
	else if ((FRS1_tau.size() != rhs.FRS1_tau.size()) || !std::equal(FRS1_tau.cbegin(), FRS1_tau.cend(), rhs.FRS1_tau.cbegin())) return false;
	else if ((obs2D_tau.size() != rhs.obs2D_tau.size()) || !std::equal(obs2D_tau.cbegin(), obs2D_tau.cend(), rhs.obs2D_tau.cbegin())) return false;

	else if ((tauBR.size() != rhs.tauBR.size()) || !std::equal(tauBR.cbegin(), tauBR.cend(), rhs.tauBR.cbegin())) return false;
	else if ((tauAR.size() != rhs.tauAR.size()) || !std::equal(tauAR.cbegin(), tauAR.cend(), rhs.tauAR.cbegin())) return false;
	else if ((tau.size() != rhs.tau.size()) || !std::equal(tau.cbegin(), tau.cend(), rhs.tau.cbegin())) return false;




	else if ((BRS1.size() != rhs.BRS1.size()) || !std::equal(BRS1.cbegin(), BRS1.cend(), rhs.BRS1.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((nomTraj.size() != rhs.nomTraj.size()) || !std::equal(nomTraj.cbegin(), nomTraj.cend(), rhs.nomTraj.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((nomTraj_AR.size() != rhs.nomTraj_AR.size()) || !std::equal(nomTraj_AR.cbegin(), nomTraj_AR.cend(), rhs.nomTraj_AR.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((obsForRTT.size() != rhs.obsForRTT.size()) || !std::equal(obsForRTT.cbegin(), obsForRTT.cend(), rhs.obsForRTT.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((obsForRTT_s8.size() != rhs.obsForRTT_s8.size()) || !std::equal(obsForRTT_s8.cbegin(), obsForRTT_s8.cend(), rhs.obsForRTT_s8.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((obsForIntr.size() != rhs.obsForIntr.size()) || !std::equal(obsForIntr.cbegin(), obsForIntr.cend(), rhs.obsForIntr.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((FRS1.size() != rhs.FRS1.size()) || !std::equal(FRS1.cbegin(), FRS1.cend(), rhs.FRS1.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;
	else if ((obs2D.size() != rhs.obs2D.size()) || !std::equal(obs2D.cbegin(), obs2D.cend(), rhs.obs2D.cbegin(), [](const auto& lhs, const auto& rhs) {
		return  ((lhs.size() == rhs.size()) && std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin()));
	})) return false;



	else return true;
}
bool SeqPP::SPPPlane::operator==(const DynSys& rhs) const {
	if (this == &rhs) return true;
	else if (typeid(*this) != typeid(rhs)) return false;
	else return operator==(dynamic_cast<const SPPPlane&>(rhs));
}
bool SeqPP::SPPPlane::get_to_save() const
{
	return to_save;
}
void SeqPP::SPPPlane::set_to_save(const bool flag)
{
	to_save = flag;
}

bool SeqPP::SPPPlane::save(
	beacls::MatFStream* fs,
	beacls::MatVariable* variable_ptr
) {
	bool result = Plane::save(fs, variable_ptr);
	//!< If tracking 3D trajectory
	if (!vReserved.empty()) result &= save_vector(vReserved, std::string("vReserved"), beacls::IntegerVec(), true, fs, variable_ptr);
	result &= save_value(wReserved, std::string("wReserved"), true, fs, variable_ptr);;

	result &= save_value(v, std::string("v"), true, fs, variable_ptr); //!< If tracking 2D trajectory

	if (!BRS1.empty()) result &= save_vector_of_vectors(BRS1, std::string("BRS1"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!BRS1_tau.empty()) result &= save_vector(BRS1_tau, std::string("BRS1_tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	if (!nomTraj.empty()) result &= save_vector_of_vectors(nomTraj, std::string("nomTraj"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!nomTraj_tau.empty()) result &= save_vector(nomTraj_tau, std::string("nomTraj_tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	if (!nomTraj_AR.empty()) result &= save_vector_of_vectors(nomTraj_AR, std::string("nomTraj_AR"), beacls::IntegerVec(), true, fs, variable_ptr); //!< Nominal trajectory after replanning
	if (!nomTraj_AR_tau.empty()) result &= save_vector(nomTraj_AR_tau, std::string("nomTraj_AR_tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	if (!obsForRTT.empty()) result &= save_vector_of_vectors(obsForRTT, std::string("obsForRTT"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!obsForRTT_s8.empty()) result &= save_vector_of_vectors(obsForRTT_s8, std::string("obsForRTT_s8"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!obsForRTT_tau.empty()) result &= save_vector(obsForRTT_tau, std::string("obsForRTT_tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	if (!obsForIntr.empty()) result &= save_vector_of_vectors(obsForIntr, std::string("obsForIntr"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!obsForIntr_tau.empty()) result &= save_vector(obsForIntr_tau, std::string("obsForIntr_tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	if (!FRS1.empty()) result &= save_vector_of_vectors(FRS1, std::string("FRS1"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!FRS1_tau.empty()) result &= save_vector(FRS1_tau, std::string("FRS1_tau"), beacls::IntegerVec(), true, fs, variable_ptr);
	result &= save_value(FRS1_g, std::string("FRS1_g"), true, fs, variable_ptr);

	if (!obs2D.empty()) result &= save_vector_of_vectors(obs2D, std::string("obs2D"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!obs2D_tau.empty()) result &= save_vector(obs2D_tau, std::string("obs2D_tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	if (!target.empty()) result &= save_vector(target, std::string("target"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!targetsm.empty()) result &= save_vector(targetsm, std::string("targetsm"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!targetCenter.empty()) result &= save_vector(targetCenter, std::string("targetCenter"), beacls::IntegerVec(), true, fs, variable_ptr);
	result &= save_value(targetR, std::string("targetR"), true, fs, variable_ptr);
	result &= save_value(targetRsmall, std::string("targetRsmall"), true, fs, variable_ptr);

	//!< Replanning
	result &= save_value(replan, std::string("replan"), true, fs, variable_ptr); //!< Whether replan is done
	if (!tauBR.empty()) result &= save_vector(tauBR, std::string("tauBR"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!tauAR.empty()) result &= save_vector(tauAR, std::string("tauAR"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!tau.empty()) result &= save_vector(tau, std::string("tau"), beacls::IntegerVec(), true, fs, variable_ptr);

	return result;
}

bool SeqPP::SPPPlane::computeBRS1(
	const beacls::FloatVec& src_BRS1_tau,
	const HJI_Grid* g,
	const beacls::FloatVec& staticObs,
	const Obstacles& obstacles,
	const std::string& SPPP_folder,
	const size_t vehicle,
	const bool low_memory
) {
	//!< using same tau as FRS is causing BRS to not include target

	//!< Set schemeData
	DynSysSchemeData* schemeData = new DynSysSchemeData();
	schemeData->set_grid(g);
	schemeData->uMode = DynSys_UMode_Min;

	beacls::FloatVec nom_vrange(vrange.size());
	std::transform(vrange.cbegin(), vrange.cend(), vReserved.cbegin(), nom_vrange.begin(), std::plus<FLOAT_TYPE>());
	const FLOAT_TYPE nom_wMax = wMax + wReserved;
	const beacls::FloatVec x = get_x();
	Plane* dynSys = new Plane(x, nom_wMax, nom_vrange);
	schemeData->dynSys = dynSys;

	//!< Visualization
	helperOC::HJIPDE_extraArgs extraArgs;
	extraArgs.visualize = true;
	extraArgs.deleteLastPlot = true;
	extraArgs.plotData.plotDims = beacls::IntegerVec{ 1, 1, 0 };
	extraArgs.plotData.projpt = beacls::FloatVec{ x[2] };

	//!< Min with target
	extraArgs.targets.resize(1);
	if (low_memory) {
		extraArgs.low_memory = true;
		extraArgs.flip_output = true;
#if !defined(FLOAT_TYPE_32F)
		beacls::FloatVec tmp_targetsm;
		tmp_targetsm.resize(targetsm.size());
		std::transform(targetsm.cbegin(), targetsm.cend(), tmp_targetsm.begin(), [](const auto& rhs) {
			return static_cast<FLOAT_TYPE>(static_cast<float>(rhs));
		});
		tmp_targetsm.swap(extraArgs.targets[0]);
#else
		extraArgs.targets[0] = targetsm;
#endif
	}
	else {
		extraArgs.targets[0] = targetsm;
	}

#if defined(FILESYSTEM)	//!< T.B.D>
	std::stringstream ss;
	ss << vehicle;
	std::string folder = SPPP_folder + "/" + __func__ + "_" + ss.str();
	extraArgs.fig_filename = folder + std::string("/");
	fs::create_directories(folder);
#else
	std::stringstream ss;
	ss << vehicle;
	std::string folder = SPPP_folder + "_" + __func__ + "_" + ss.str();
	extraArgs.fig_filename = folder + std::string("_");
#endif


	//!< Set obstacles
	beacls::FloatVec modified_staticObs;
	const size_t staticObsSize = staticObs.size();
	if (!staticObs.empty() && (staticObsSize!=0)) {
		if (staticObsSize != obstacles.data[0].size()) {
			const size_t reptimes = obstacles.data[0].size() / staticObs.size();
			modified_staticObs.reserve(staticObs.size()*reptimes);
			for (size_t i = 0; i < reptimes; ++i) {
				modified_staticObs.insert(modified_staticObs.end(), staticObs.cbegin(), staticObs.cend());
			}
		}
		else {
			modified_staticObs = staticObs;
		}
	}

	//!< Obstacles for overlapping time indices
	static const FLOAT_TYPE small = (FLOAT_TYPE)1e-4;
	if (!src_BRS1_tau.empty()) {
		auto minmax_BRS1_tau = beacls::minmax_value<FLOAT_TYPE>(src_BRS1_tau.cbegin(), src_BRS1_tau.cend());
		const FLOAT_TYPE max_BRS1_tau = minmax_BRS1_tau.second;
		const FLOAT_TYPE min_BRS1_tau = minmax_BRS1_tau.first;
		extraArgs.obstacles.reserve(obstacles.tau.size());
		for (int64_t t = obstacles.tau.size() - 1; t >= 0; --t) {
			if ((obstacles.tau[t] < max_BRS1_tau + small) && (obstacles.tau[t] > min_BRS1_tau - small)) {
				extraArgs.obstacles.push_back(&obstacles.data[t]);
			}
		}
	}

	//!< Obstacles for elements of BRS1_tau that are smaller than in obstacles.tau
	const FLOAT_TYPE min_obstacles_tau = beacls::min_value<FLOAT_TYPE>(obstacles.tau.cbegin(), obstacles.tau.cend());
	for (size_t t = 0; t < src_BRS1_tau.size(); ++t) {
		//!< Concatenation is done "in reverse" to flip the obstacle order for BRS
		if (src_BRS1_tau[t] < min_obstacles_tau - small) {
			extraArgs.obstacles.push_back(&modified_staticObs);
		}
	}

	//!< No need to consider elements of BRS1_tau that are larger than in obstacles.tau

	//!< Extra solver parameters

	//!< Computation should stop once it contains the initial state
	extraArgs.stopInit = x;

	extraArgs.execParameters = execParameters;

	helperOC::HJIPDE_extraOuts extraOuts;
	HJIPDE* hjipde = new HJIPDE();
	beacls::FloatVec new_tau;
	std::vector<std::vector<FLOAT_TYPE >> new_BRS1;
	hjipde->solve(new_tau, extraOuts, extraArgs.targets[0], src_BRS1_tau, schemeData, HJIPDE::MinWithType_None, extraArgs);
	hjipde->get_datas(new_BRS1, src_BRS1_tau, schemeData);

	//!< Reverse the order of time elements
	BRS1_tau.resize(new_tau.size());
	std::copy(src_BRS1_tau.cbegin() + src_BRS1_tau.size() - new_tau.size(), src_BRS1_tau.end(), BRS1_tau.begin());
	if (!low_memory) {
		BRS1.resize(src_BRS1_tau.size());
		std::copy(new_BRS1.crbegin(), new_BRS1.crend(), BRS1.begin()+BRS1.size()-new_BRS1.size());
	}
	else {
		new_BRS1.swap(BRS1);
	}
	if (hjipde) delete hjipde;
	if (dynSys) delete dynSys;
	if (schemeData) delete schemeData;
	return false;
}
bool SeqPP::SPPPlane::computeBRS1(
	const beacls::FloatVec& src_BRS1_tau,
	const HJI_Grid* g,
	const std::vector<int8_t>& staticObs,
	const Obstacles& obstacles,
	const std::string& SPPP_folder,
	const size_t vehicle,
	const bool low_memory
) {
	//!< using same tau as FRS is causing BRS to not include target

	//!< Set schemeData
	DynSysSchemeData* schemeData = new DynSysSchemeData();
	schemeData->set_grid(g);
	schemeData->uMode = DynSys_UMode_Min;

	beacls::FloatVec nom_vrange(vrange.size());
	std::transform(vrange.cbegin(), vrange.cend(), vReserved.cbegin(), nom_vrange.begin(), std::plus<FLOAT_TYPE>());
	const FLOAT_TYPE nom_wMax = wMax + wReserved;
	const beacls::FloatVec x = get_x();
	Plane* dynSys = new Plane(x, nom_wMax, nom_vrange);
	schemeData->dynSys = dynSys;

	//!< Visualization
	helperOC::HJIPDE_extraArgs extraArgs;
	extraArgs.visualize = true;
	extraArgs.deleteLastPlot = true;
	extraArgs.plotData.plotDims = beacls::IntegerVec{ 1, 1, 0 };
	extraArgs.plotData.projpt = beacls::FloatVec{ x[2] };

	//!< Min with target
	extraArgs.targets.resize(1);
	if (low_memory) {
		extraArgs.low_memory = true;
		extraArgs.flip_output = true;
#if !defined(FLOAT_TYPE_32F)
		beacls::FloatVec tmp_targetsm;
		tmp_targetsm.resize(targetsm.size());
		std::transform(targetsm.cbegin(), targetsm.cend(), tmp_targetsm.begin(), [](const auto& rhs) {
			return static_cast<FLOAT_TYPE>(static_cast<float>(rhs));
		});
		tmp_targetsm.swap(extraArgs.targets[0]);
#else
		extraArgs.targets[0] = targetsm;
#endif
	}
	else {
		extraArgs.targets[0] = targetsm;
	}

#if defined(FILESYSTEM)	//!< T.B.D>
	std::stringstream ss;
	ss << vehicle;
	std::string folder = SPPP_folder + "/" + __func__ + "_" + ss.str();
	extraArgs.fig_filename = folder + std::string("/");
	fs::create_directories(folder);
#else
	std::stringstream ss;
	ss << vehicle;
	std::string folder = SPPP_folder + "_" + __func__ + "_" + ss.str();
	extraArgs.fig_filename = folder + std::string("_");
#endif


	//!< Set obstacles
	std::vector<int8_t> modified_staticObs;
	const size_t staticObsSize = staticObs.size();
	if (!staticObs.empty() && (staticObsSize != 0)) {
		if (staticObsSize != obstacles.data_s8[0].size()) {
			const size_t reptimes = obstacles.data_s8[0].size() / staticObs.size();
			for (size_t i = 0; i < reptimes; ++i) {
				modified_staticObs.insert(modified_staticObs.end(), staticObs.cbegin(), staticObs.cend());
			}
		}
		else {
			modified_staticObs = staticObs;
		}
	}

	//!< Obstacles for overlapping time indices
	static const FLOAT_TYPE small = (FLOAT_TYPE)1e-4;
	if (!src_BRS1_tau.empty()) {
		auto minmax_BRS1_tau = beacls::minmax_value<FLOAT_TYPE>(src_BRS1_tau.cbegin(), src_BRS1_tau.cend());
		const FLOAT_TYPE max_BRS1_tau = minmax_BRS1_tau.second;
		const FLOAT_TYPE min_BRS1_tau = minmax_BRS1_tau.first;
		extraArgs.obstacles_s8.reserve(obstacles.tau.size());
		for (int64_t t = obstacles.tau.size() - 1; t >= 0; --t) {
			if ((obstacles.tau[t] < max_BRS1_tau + small) && (obstacles.tau[t] > min_BRS1_tau - small)) {
				extraArgs.obstacles_s8.push_back(&obstacles.data_s8[t]);
			}
		}
	}

	//!< Obstacles for elements of BRS1_tau that are smaller than in obstacles.tau
	const FLOAT_TYPE min_obstacles_tau = beacls::min_value<FLOAT_TYPE>(obstacles.tau.cbegin(), obstacles.tau.cend());
	for (size_t t = 0; t < src_BRS1_tau.size(); ++t) {
		//!< Concatenation is done "in reverse" to flip the obstacle order for BRS
		if (src_BRS1_tau[t] < min_obstacles_tau - small) {
			extraArgs.obstacles_s8.push_back(&modified_staticObs);
		}
	}

	//!< No need to consider elements of BRS1_tau that are larger than in obstacles.tau

	//!< Extra solver parameters

	//!< Computation should stop once it contains the initial state
	extraArgs.stopInit = x;

	extraArgs.execParameters = execParameters;

	helperOC::HJIPDE_extraOuts extraOuts;
	HJIPDE* hjipde = new HJIPDE();
	beacls::FloatVec new_tau;
	std::vector<std::vector<FLOAT_TYPE >> new_BRS1;
	hjipde->solve(new_tau, extraOuts, extraArgs.targets[0], src_BRS1_tau, schemeData, HJIPDE::MinWithType_None, extraArgs);
	hjipde->get_datas(new_BRS1, src_BRS1_tau, schemeData);

	//!< Reverse the order of time elements
	BRS1_tau.resize(new_tau.size());
	std::copy(src_BRS1_tau.cbegin() + src_BRS1_tau.size() - new_tau.size(), src_BRS1_tau.end(), BRS1_tau.begin());
	if (!low_memory) {
		BRS1.resize(src_BRS1_tau.size());
		std::copy(new_BRS1.crbegin(), new_BRS1.crend(), BRS1.begin() + BRS1.size() - new_BRS1.size());
	}
	else {
		new_BRS1.swap(BRS1);
	}
	if (hjipde) delete hjipde;
	if (dynSys) delete dynSys;
	if (schemeData) delete schemeData;
	return false;
}
bool SeqPP::SPPPlane::computeNomTraj(
	const HJI_Grid* g,
	const std::string& SPPP_folder,
	const size_t vehicle,
	helperOC::ComputeOptTraj* computeOptTraj
) {
	static const FLOAT_TYPE small = (FLOAT_TYPE)1e-4;

	//!< Modify control bounds
	beacls::FloatVec nom_vrange(vrange.size());
	std::transform(vrange.cbegin(), vrange.cend(), vReserved.cbegin(), nom_vrange.begin(), std::plus<FLOAT_TYPE>());
	const FLOAT_TYPE nom_wMax = wMax + wReserved;
	const beacls::FloatVec x = get_x();
	Plane* dynSys = new Plane(x, nom_wMax, nom_vrange);

	//!< Set extraArgs
	helperOC::HJIPDE_extraArgs extraArgs;
	const DynSys_UMode_Type uMode = DynSys_UMode_Min;
	extraArgs.visualize = true;
	extraArgs.projDim = beacls::IntegerVec{ 1, 1, 0 };
	static const size_t subSamples = 32;

	extraArgs.execParameters = execParameters;

#if defined(FILESYSTEM)	//!< T.B.D>
	std::stringstream ss;
	ss << vehicle;
	std::string folder = SPPP_folder + "/" + __func__ + "_" + ss.str();
	extraArgs.fig_filename = folder + std::string("/");
	fs::create_directories(folder);
#else
	std::stringstream ss;
	ss << vehicle;
	std::string folder = SPPP_folder + "_" + __func__ + "_" + ss.str();
	extraArgs.fig_filename = folder + std::string("_");
#endif


	//!< Compute trajectory
	std::vector<beacls::FloatVec> new_nomTraj;
	beacls::FloatVec new_nomTraj_tau;
	computeOptTraj->operator()(new_nomTraj, new_nomTraj_tau, g, BRS1, BRS1_tau, dynSys, extraArgs, uMode, subSamples);
	//!< Pad based on FRS1_tau, if available
	if (FRS1.empty()) {
		//!< Update nominal trajectory if it doesn't exist
		new_nomTraj_tau.swap(nomTraj_tau);
		new_nomTraj.swap(nomTraj);
	} else {
		const FLOAT_TYPE min_BRS1_tau = beacls::min_value<FLOAT_TYPE>(BRS1_tau.cbegin(), BRS1_tau.cend());
		beacls::FloatVec pad_tau;
		pad_tau.reserve(FRS1_tau.size());
		for (size_t i = 0; i < FRS1_tau.size(); ++i) {
			if (FRS1_tau[i] < (min_BRS1_tau - small)) {
				pad_tau.push_back(FRS1_tau[i]);
			}
		}
		std::vector<beacls::FloatVec > pad_nomTraj(pad_tau.size());
		std::fill(pad_nomTraj.begin(), pad_nomTraj.end(), new_nomTraj[0]);
		
		pad_tau.insert(pad_tau.end(), new_nomTraj_tau.cbegin(), new_nomTraj_tau.cend());
		pad_nomTraj.insert(pad_nomTraj.end(), new_nomTraj.cbegin(), new_nomTraj.cend());

		pad_tau.swap(nomTraj_AR_tau);
		pad_nomTraj.swap(nomTraj_AR);
	}
	if (dynSys)delete dynSys;
	return false;
}
bool SeqPP::SPPPlane::computeObsForRTT(
	const SPPProblem* sppp,
	const RTTRS* rttrs,
	const std::vector< beacls::FloatVec>& src_nomTraj,
	const beacls::FloatVec& src_nomTraj_tau
) {
	const bool lowprecision_obstacles = sppp->get_s8ary_obstacles();
	const std::vector< beacls::FloatVec>& target_nomTraj = (src_nomTraj_tau.empty()) ? nomTraj : src_nomTraj;
	const beacls::FloatVec& target_nomTraj_tau = (src_nomTraj_tau.empty()) ? nomTraj_tau : src_nomTraj_tau;


	const FLOAT_TYPE R_augment = (FLOAT_TYPE)(1.1*(sppp->get_Rc() + rttrs->get_trackingRadius())); //!< Amount to augment RTTRS by
	beacls::FloatVec rttrs2d;

	MigrateRTTRS* migrateRTTRS = sppp->get_MigrateRTTRS();
	HJI_Grid* g2D = migrateRTTRS->operator()(rttrs2d, rttrs, R_augment);

	//!< Initialize 2D obstacles
	obs2D_tau = target_nomTraj_tau;
	obs2D.clear();
	obs2D.reserve(target_nomTraj_tau.size());
	//!< Initialize 3D obstacles
	if (lowprecision_obstacles)
		obsForRTT_s8.resize(target_nomTraj_tau.size());
	else
		obsForRTT.resize(target_nomTraj_tau.size());
	obsForRTT_tau = target_nomTraj_tau;

	const beacls::IntegerVec& Ns = sppp->get_g()->get_Ns();
	obs2D.resize(target_nomTraj_tau.size());
	beacls::FloatVec neg_target(target.size());
	std::transform(target.cbegin(), target.cend(), neg_target.begin(), std::negate<FLOAT_TYPE>());
	int tau_length = (int)target_nomTraj_tau.size();
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int t = 0; t < tau_length; ++t) {
		const beacls::FloatVec& x = target_nomTraj[t];
		beacls::FloatVec obsi_rot;
		beacls::FloatVec obsi;
		beacls::FloatVec p(x.cbegin(), x.cbegin() + 2);
		//!< Rotate and shift raw obstacles
		const FLOAT_TYPE angle = x[2];
		obsi_rot.clear();
		helperOC::rotateData(obsi_rot, g2D, rttrs2d, angle, beacls::IntegerVec{0, 1}, beacls::IntegerVec());
		HJI_Grid* obsi_gShift = helperOC::shiftGrid(g2D, p);
		obsi.clear();
		helperOC::migrateGrid(obsi, obsi_gShift, obsi_rot, sppp->get_g2D());
		obs2D[t] = obsi;

		if (obsi_gShift) delete obsi_gShift;
		//!< Exclude target set
		const size_t obsi_size = obsi.size();
		const size_t N2 = Ns[2];
		if (lowprecision_obstacles) {
			std::vector<int8_t>& obsForRTT_t = obsForRTT_s8[t];
			obsForRTT_t.resize(obsi_size * N2);
			for (size_t n = 0; n < N2; ++n) {
				const size_t offset_3d = obsi_size * n;
				std::transform(obsi.cbegin(), obsi.cend(), neg_target.cbegin() + offset_3d, obsForRTT_t.begin() + offset_3d, [](const auto& lhs, const auto& rhs) {
					return static_cast<int8_t>(std::floor(std::max<FLOAT_TYPE>(lhs, rhs) * obstacles_fix_ratio));
				});
			}

		}
		else {
			beacls::FloatVec& obsForRTT_t = obsForRTT[t];
			obsForRTT_t.resize(obsi_size * N2);
			for (size_t n = 0; n < N2; ++n) {
				const size_t offset_3d = obsi_size * n;
				std::transform(obsi.cbegin(), obsi.cend(), neg_target.cbegin() + offset_3d, obsForRTT_t.begin() + offset_3d, std::ptr_fun<const FLOAT_TYPE&, const FLOAT_TYPE&>(std::max<FLOAT_TYPE>));
			}
		}
	}
	if (g2D) delete g2D;
	return false;
}

class SeqPP::RawAugObs {
private:
	HJI_Grid* g2D;
	std::vector<beacls::FloatVec> FRS2D;
public:
	HJI_Grid* get_g2D() const { return g2D; }
	const std::vector<beacls::FloatVec>& get_FRS2D() const { return FRS2D; };
};

bool SeqPP::SPPPlane::addObs2D(
	const SPPProblem* sppp,
	const RTTRS* rttrs,
	const CARS* cars,
	const RawAugObs* rawAugObs
) {
	const FLOAT_TYPE small0 = (FLOAT_TYPE)1e-3;

	//!< Use newer nominal trajectory if there's replanning
	if (!nomTraj_AR.empty()) {
		//!< Determine the time steps at which there is new nominal trajectory
		const FLOAT_TYPE min_nomTraj_AR_tau = beacls::min_value<FLOAT_TYPE>(nomTraj_AR_tau.cbegin(), nomTraj_AR_tau.cend());

		std::vector<beacls::FloatVec> tmp_nomTraj;
		beacls::FloatVec tmp_nomTraj_tau;
		tmp_nomTraj.reserve(nomTraj.size());
		tmp_nomTraj_tau.reserve(nomTraj_tau.size());
		for (size_t t = 0; t < nomTraj_tau.size(); ++t) {
			//!< Replace nominal trajectory
			if (nomTraj_tau[t] < (min_nomTraj_AR_tau - small0)) {
				tmp_nomTraj.push_back(nomTraj[t]);
				tmp_nomTraj_tau.push_back(nomTraj_tau[t]);
			}
		}
		tmp_nomTraj.insert(tmp_nomTraj.end(), nomTraj_AR.cbegin(), nomTraj_AR.cend());
		tmp_nomTraj_tau.insert(tmp_nomTraj_tau.end(), nomTraj_AR_tau.cbegin(), nomTraj_AR_tau.cend());
		nomTraj = tmp_nomTraj;
		nomTraj_tau = tmp_nomTraj_tau;
	}
	HJI_Grid* g2D = sppp->get_g2D();

	//!< If there's a replan time, then migrate flattened augmented obstacles
	replan = false;
	std::vector<beacls::FloatVec > rawAugObs2D;
	if (!sppp->get_tReplan().empty() && rawAugObs) {
		replan = true;
		std::cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << " is not implemented yet!" << std::endl;
		helperOC::migrateGrid(rawAugObs2D, rawAugObs->get_g2D(), rawAugObs->get_FRS2D(), g2D);
	}

	//!< Migrate and add capture radius to RTTRS
	const FLOAT_TYPE R_augment = (FLOAT_TYPE)(1.1*(sppp->get_Rc() + rttrs->get_trackingRadius())); //!< Amount to augment RTTRS by
	beacls::FloatVec rawObs2D;
	MigrateRTTRS* migrateRTTRS = sppp->get_MigrateRTTRS();
	HJI_Grid* tmp_g = migrateRTTRS->operator()(rawObs2D, rttrs, R_augment);
	if (tmp_g) delete tmp_g;

	//!< Initialize 2D obstacles
	obs2D_tau = nomTraj_tau;
	obs2D.resize(nomTraj_tau.size());
	const size_t g2d_num_of_elements = g2D->get_sum_of_elems();
	std::for_each(obs2D.begin(), obs2D.end(), [&g2d_num_of_elements](auto& rhs) { rhs.resize(g2d_num_of_elements); });

	const FLOAT_TYPE small1 = (FLOAT_TYPE)1e-4;
	const FLOAT_TYPE min_nomTraj_tau = beacls::min_value<FLOAT_TYPE>(nomTraj_tau.cbegin(), nomTraj_tau.cend());

	const beacls::FloatVec dummy;
	const beacls::FloatVec& cars_tau = replan ? cars->get_tau() : dummy;
	const FLOAT_TYPE max_cars_tau = (replan && !cars_tau.empty())? beacls::max_value<FLOAT_TYPE>(cars_tau.cbegin(), cars_tau.cend())	: 0;
	beacls::FloatVec rawObsDatai;
	std::transform(nomTraj_tau.cbegin(), nomTraj_tau.cend(), nomTraj.cbegin(), obs2D.begin(), [&sppp, &min_nomTraj_tau, this, &max_cars_tau, &cars_tau, &small1, &rawAugObs2D, &rawObs2D, &g2D, &rawObsDatai](const auto& lhs, const auto& rhs) {
		beacls::FloatVec rawObsToRotate;
		//!< Shift and rotation amounts
		beacls::FloatVec p(rhs.cbegin(), rhs.cbegin() + 2);
		const FLOAT_TYPE t = rhs[2];
		if (replan && lhs < sppp->get_tReplan()[0]) {
			//!< Before replanning:
			//!<     for the first tauIAT time steps, use the i-step FRS projection
			const FLOAT_TYPE tauElapsed = lhs - min_nomTraj_tau;
			if (tauElapsed < max_cars_tau) {
				for (size_t tau_index = 0; tau_index < cars_tau.size(); ++tau_index) {
					if ((cars_tau[tau_index] > (tauElapsed - small1)) && (cars_tau[tau_index] < (tauElapsed + small1))) {
						rawAugObs2D[tau_index].swap(rawObsToRotate);
					}
				}
			}
			else {
				rawAugObs2D[rawAugObs2D.size() - 1].swap(rawObsToRotate);
			}
		}
		else {
			//!< After replanning: always use the same 2D obstacle
			rawObs2D.swap(rawObsToRotate);
		}
		//!< Rotate and shift 2D obstacle
		helperOC::rotateData(rawObsDatai, g2D, rawObsToRotate, t, beacls::IntegerVec{0, 1}, beacls::IntegerVec());
		helperOC::shiftData(rawObsDatai, g2D, rawObsDatai, p, beacls::IntegerVec{0, 1});
		return rawObsDatai;
	});
	return false;
}
