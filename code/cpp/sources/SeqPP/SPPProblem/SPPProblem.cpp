#define _USE_MATH_DEFINES
#include <cmath>
#include <random>
#include <helperOC/Grids/createGrid.hpp>
#include <SeqPP/SPPProblem/SPPProblem.hpp>
#include <SeqPP/SPPProblem/NIRS.hpp>
#include <SeqPP/SPPProblem/RTTRS.hpp>
#include <SeqPP/SPPProblem/CARS.hpp>
#include <SeqPP/SPPProblem/GridParameters.hpp>
#include <SeqPP/SPPProblem/VehicleParameters.hpp>
#include <SeqPP/SPPPlane/SPPPlane.hpp>
#include <SeqPP/updateObstacles.hpp>
#include <SeqPP/initRTT.hpp>
#include <SeqPP/MigrateRTTRS.hpp>
#include <helperOC/helperOC_type.hpp>
#include <helperOC/DynSys/PlaneCAvoid/PlaneCAvoid.hpp>
#include <helperOC/DynSys/Plane/Plane.hpp>
#include <helperOC/DynSys/DynSys/DynSysSchemeData.hpp>
#include <helperOC/ValFuncs/rotateData.hpp>
#include <helperOC/ValFuncs/shiftData.hpp>
#include <helperOC/Grids/shiftGrid.hpp>
#include <helperOC/Grids/migrateGrid.hpp>
#include <helperOC/ValFuncs/AddCRadius.hpp>
#include <helperOC/ValFuncs/HJIPDE.hpp>
#include <helperOC/ComputeGradients.hpp>
#include <helperOC/ComputeOptTraj.hpp>
#include <helperOC/ValFuncs/eval_u.hpp>
#include <helperOC/rotate2D.hpp>
#include <levelset/InitialConditions/BasicShapes/ShapeRectangleByCorner.hpp>
#include <levelset/InitialConditions/BasicShapes/ShapeHyperplaneByPoint.hpp>
#include <levelset/levelset.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <functional>
#include <thread>
#include <mutex>
#if defined(FILESYSTEM)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace SeqPP {
	class MatWriter {
	public:
		std::mutex mtx;
	};
	void saveNI_RS_chkpt_and_clear(
		MatWriter* matWriter, 
		const std::string filename,
		SeqPP::NIRS* nirs,
		SeqPP::Obstacles* obstacles_ptr, 
		const size_t vehicle_index,
		const CheckPointType checkPointType,
		const size_t stride,
		const bool saveBRS1,
		const bool saveObstacles);
}

SeqPP::SPPProblem::SPPProblem(
	const ProblemType problem_name,
	const std::string& RTTRS_filename,
	const std::string& NI_RS_filename,
	const std::string& NI_RS_chkpt_filename,
	const bool lowprecision_obstacles,
	const helperOC::ExecParameters& execParameters,
	const SetupExtraArgs setupExtraArgs,
	const beacls::IntegerVec& rttrs_gN
) : 
	initStates(std::vector<beacls::FloatVec >()),
	targetCenters(std::vector<beacls::FloatVec >()),
	targetR(0),
	targetRsmall(std::vector<beacls::FloatVec >()),
	tIAT(0),
	max_num_affected_vehicles(0),
	buffer_duration(0),
	buffer_duration_ind(0),
	remaining_duration_ind(0),
	Rc((FLOAT_TYPE)0.1),
	mapFile(std::string()),
	staticObs(beacls::FloatVec()),
	augStaticObs(beacls::FloatVec()),
	vRangeA(beacls::FloatVec()),
	wMaxA(0),
	dMaxA(beacls::FloatVec()),
	vReserved(beacls::FloatVec()),
	wReserved(0),
	RTT_tR(0),
	tMin(-5),
	dt((FLOAT_TYPE)0.01),
	tTarget(beacls::FloatVec{0}),
	tIntr(0),
	tReplan(beacls::FloatVec()),
	max_BRS_time(0),
	rttrs_tMax(30),
	rttrs_gN(rttrs_gN.empty() ? beacls::IntegerVec{ 51, 51, 101} : rttrs_gN),
	gMin(beacls::FloatVec()),
	gMax(beacls::FloatVec()),
	gN(beacls::IntegerVec()),
	g(NULL),
	g2D(NULL),
	tauBR(beacls::FloatVec()),
	tauAR(beacls::FloatVec()),
	tauSim(beacls::FloatVec()),
	tau(beacls::FloatVec()),
	rttrs(NULL),
	nirs(NULL),
	cars(NULL),
	folder(std::string()),
	RTTRS_filename(RTTRS_filename),
	CARS_filename(std::string()),
	rawAugObs_filename(std::string()),
	NI_RS_chkpt_filename(NI_RS_chkpt_filename),
	NI_RS_filename(NI_RS_filename),
	NI_sim_filename(std::string()),
	BR_RS_filename(std::string()),
	BR_RS_chkpt_filename(std::string()),
	AR_RS_filename(std::string()),
	AR_RS_chkpt_filename(std::string()),
	BR_sim_filename(std::string()),
	full_sim_filename(std::string()),
	keepLast(true),
	lowprecision_obstacles(lowprecision_obstacles),
	execParameters(execParameters),
	migrateRTTRS(new MigrateRTTRS()),
	matWriter(new MatWriter)
{
	const auto now = std::chrono::system_clock::now();
	const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
	std::stringstream now_ss;
	now_ss << std::fixed << std::setprecision(6) << ((long double)now_ms.count() / 1000 / 60 / 60 / 24) << std::resetiosflags(std::ios_base::floatfield);
	folder = __func__ + std::string("_") + now_ss.str();
	SetupExtraArgs modified_setupExtraArgs = setupExtraArgs;
#if defined(WIN32)	// Windows
	const std::string input_folder(".\\\\inputs\\\\");
#else	//!< Unix
	const std::string input_folder("./inputs/");
#endif
	switch (problem_name) {
	case ProblemType_default:
	case ProblemType_SF_dstb:
		modified_setupExtraArgs.number_of_vehicles = 50;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_dstb_5veh:
		modified_setupExtraArgs.number_of_vehicles = 5;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC_5veh.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_dstb_10veh:
		modified_setupExtraArgs.number_of_vehicles = 10;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC_10veh.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_dstb_20veh:
		modified_setupExtraArgs.number_of_vehicles = 20;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC_20veh.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_dstb_30veh:
		modified_setupExtraArgs.number_of_vehicles = 30;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC_30veh.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_dstb_40veh:
		modified_setupExtraArgs.number_of_vehicles = 40;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC_40veh.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_dstb_hell:
		modified_setupExtraArgs.number_of_vehicles = 50;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Dstb;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC_hell.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		break;
	case ProblemType_SF_intr_2:
		modified_setupExtraArgs.number_of_vehicles = 50;
		modified_setupExtraArgs.wind_speed = 6;
		if (modified_setupExtraArgs.separation_time<0) modified_setupExtraArgs.separation_time = 45;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Intr;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		//!< Intruder-related
		max_num_affected_vehicles = 2;

#if 0
		obj.add_data_file('RTTRS', 'RTTRS6.mat');
		obj.add_data_file('CARS', 'CARS6.mat');
		obj.add_data_file('bufferRegion', 'bufferRegion2_6.mat')
		obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
		obj.augment_staticObs_intr2;
#endif
		
		break;
	case ProblemType_SF_intr_3:
		modified_setupExtraArgs.number_of_vehicles = 50;
		modified_setupExtraArgs.wind_speed = 6;
		if (modified_setupExtraArgs.separation_time<0) modified_setupExtraArgs.separation_time = 45;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Intr;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		//!< Intruder-related
		max_num_affected_vehicles = 3;
#if 0
		obj.add_data_file('RTTRS', 'RTTRS6.mat');
		obj.add_data_file('CARS', 'CARS6.mat');
		obj.add_data_file('bufferRegion', 'bufferRegion3_6.mat')
			obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
			obj.augment_staticObs_intr2;
#endif
		break;
	case ProblemType_SF_intr_4:
		modified_setupExtraArgs.number_of_vehicles = 50;
		modified_setupExtraArgs.wind_speed = 6;
		if (modified_setupExtraArgs.separation_time<0) modified_setupExtraArgs.separation_time = 45;
		modified_setupExtraArgs.dstbOrIntr = DstbIntr_Type_Intr;
		modified_setupExtraArgs.ISTC_filename = input_folder + "SF_ISTC.mat";
		loadSetup(Setup_Type_SF, modified_setupExtraArgs);
		//!< Intruder-related
		max_num_affected_vehicles = 4;
		break;
	case ProblemType_Bay_Area:
		modified_setupExtraArgs.number_of_vehicles = 200;
		modified_setupExtraArgs.wind_speed = 11;
		if (modified_setupExtraArgs.separation_time<0) modified_setupExtraArgs.separation_time = 10;
		modified_setupExtraArgs.ISTC_filename = input_folder + "BA_ISTC.mat";
		loadSetup(Setup_Type_Bay_Area, modified_setupExtraArgs);
		break;
	case ProblemType_TCST_dstb:
		std::cerr << "Not implemented yet..." << std::endl;
		break;
	case ProblemType_TCST_intr:
		std::cerr << "Not implemented yet..." << std::endl;
		break;
	default:
	case ProblemType_Invalid:
		std::cerr << "Unknown simulation name!" << std::endl;
	break;
	}

#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	fs::create_directories(folder);
	const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
	const std::string SPPP_filename = folder + std::string("_SPPP.mat");
#endif
	beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("SPPP"));
	save(variable_ptr);
	beacls::MatFStream* fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
	beacls::writeMatVariable(fs, variable_ptr);
	beacls::closeMatVariable(variable_ptr);
	beacls::closeMatFStream(fs);
}

SeqPP::SPPProblem::SPPProblem(
	const std::string& src_SPPP_filename,
	const std::string& RTTRS_filename,
	const std::string& NI_RS_filename,
	const std::string& NI_RS_chkpt_filename,
	const bool lowprecision_obstacles,
	const helperOC::ExecParameters& execParameters
) :
	initStates(std::vector<beacls::FloatVec >()),
	targetCenters(std::vector<beacls::FloatVec >()),
	targetR(0),
	targetRsmall(std::vector<beacls::FloatVec >()),
	tIAT(0),
	max_num_affected_vehicles(0),
	buffer_duration(0),
	buffer_duration_ind(0),
	remaining_duration_ind(0),
	Rc((FLOAT_TYPE)0.1),
	mapFile(std::string()),
	staticObs(beacls::FloatVec()),
	augStaticObs(beacls::FloatVec()),
	vRangeA(beacls::FloatVec()),
	wMaxA(0),
	dMaxA(beacls::FloatVec()),
	vReserved(beacls::FloatVec()),
	wReserved(0),
	RTT_tR(0),
	tMin(-5),
	dt((FLOAT_TYPE)0.01),
	tTarget(beacls::FloatVec{0}),
	tIntr(0),
	tReplan(beacls::FloatVec()),
	max_BRS_time(0),
	rttrs_tMax(30),
	rttrs_gN(beacls::IntegerVec{ 51, 51, 101}),
	gMin(beacls::FloatVec()),
	gMax(beacls::FloatVec()),
	gN(beacls::IntegerVec()),
	g(NULL),
	g2D(NULL),
	tauBR(beacls::FloatVec()),
	tauAR(beacls::FloatVec()),
	tauSim(beacls::FloatVec()),
	tau(beacls::FloatVec()),
	rttrs(NULL),
	nirs(NULL),
	cars(NULL),
	folder(std::string()),
	RTTRS_filename(RTTRS_filename),
	CARS_filename(std::string()),
	rawAugObs_filename(std::string()),
	NI_RS_chkpt_filename(NI_RS_chkpt_filename),
	NI_RS_filename(NI_RS_filename),
	NI_sim_filename(std::string()),
	BR_RS_filename(std::string()),
	BR_RS_chkpt_filename(std::string()),
	AR_RS_filename(std::string()),
	AR_RS_chkpt_filename(std::string()),
	BR_sim_filename(std::string()),
	full_sim_filename(std::string()),
	keepLast(true),
	lowprecision_obstacles(lowprecision_obstacles),
	execParameters(execParameters),
	migrateRTTRS(new MigrateRTTRS(execParameters)),
	matWriter(new MatWriter)
	{
	bool result = true;
	beacls::MatFStream* fs = beacls::openMatFStream(src_SPPP_filename, beacls::MatOpenMode_Read);
	if (!fs) {
		std::cerr << "Error: source SPPProblem file not found: " << src_SPPP_filename.c_str() << std::endl;
		return;
	}
	beacls::MatVariable* variable_ptr = beacls::openMatVariable(fs, std::string("SPPP"));
	if (!variable_ptr){
		std::cerr << "Error: source SPPProblem object not found in " << src_SPPP_filename.c_str() << std::endl;
		return;
	}

	beacls::IntegerVec dummy;
	result &= load_vector_of_vectors(initStates, std::string("initStates"), dummy, true, fs, variable_ptr);
	result &= load_vector_of_vectors(targetCenters, std::string("targetCenters"), dummy, true, fs, variable_ptr);

	result &= load_value(targetR, std::string("targetR"), true, fs, variable_ptr);
	result &= load_vector_of_vectors(targetRsmall, std::string("targetRsmall"), dummy, true, fs, variable_ptr);

	result &= load_value(tIAT, std::string("tIAT"), true, fs, variable_ptr);

	result &= load_value(max_num_affected_vehicles, std::string("max_num_affected_vehicles"), true, fs, variable_ptr);
	result &= load_value(buffer_duration, std::string("buffer_duration"), true, fs, variable_ptr);
	result &= load_value(buffer_duration_ind, std::string("buffer_duration_ind"), true, fs, variable_ptr);
	result &= load_value(remaining_duration_ind, std::string("remaining_duration_ind"), true, fs, variable_ptr);

	result &= load_value(Rc, std::string("Rc"), true, fs, variable_ptr);

	result &= load_vector(staticObs, std::string("staticObs"), dummy, true, fs, variable_ptr);
	result &= load_vector(augStaticObs, std::string("augStaticObs"), dummy, true, fs, variable_ptr);

	result &= load_vector(vRangeA, std::string("vRangeA"), dummy, true, fs, variable_ptr);
	result &= load_value(wMaxA, std::string("wMaxA"), true, fs, variable_ptr);
	result &= load_vector(dMaxA, std::string("dMaxA"), dummy, true, fs, variable_ptr);

	result &= load_vector(vReserved, std::string("vReserved"), dummy, true, fs, variable_ptr);
	result &= load_value(wReserved, std::string("wReserved"), true, fs, variable_ptr);
	result &= load_value(RTT_tR, std::string("RTT_tR"), true, fs, variable_ptr);

	result &= load_value(tMin, std::string("tMin"), true, fs, variable_ptr);
	result &= load_value(dt, std::string("dt"), true, fs, variable_ptr);
	result &= load_vector(tTarget, std::string("tTarget"), dummy, true, fs, variable_ptr);
	result &= load_value(tIntr, std::string("tIntr"), true, fs, variable_ptr);
	result &= load_vector(tReplan, std::string("tReplan"), dummy, true, fs, variable_ptr);
	result &= load_value(max_BRS_time, std::string("max_BRS_time"), true, fs, variable_ptr);

	result &= load_value(rttrs_tMax, std::string("rttrs_tMax"), true, fs, variable_ptr);
	result &= load_vector(rttrs_gN, std::string("rttrs_gN"), dummy, true, fs, variable_ptr);

	result &= load_vector(gMin, std::string("gMin"), dummy, true, fs, variable_ptr);
	result &= load_vector(gMax, std::string("gMax"), dummy, true, fs, variable_ptr);
	result &= load_vector(gN, std::string("gN"), dummy, true, fs, variable_ptr);
	if (!g) g = new HJI_Grid();
	if (!g->load_grid(std::string("g"), fs, variable_ptr)) {
		result = false;
	}
	else {
		g->processGrid();
	}
	if (!g2D) g2D = new HJI_Grid();
	if (!g2D->load_grid(std::string("g2D"), fs, variable_ptr)) {
		result = false;
	}
	else {
		g2D->processGrid();
	}

	result &= load_vector(tauBR, std::string("tauBR"), dummy, true, fs, variable_ptr);
	result &= load_vector(tauAR, std::string("tauAR"), dummy, true, fs, variable_ptr);
	result &= load_vector(tauSim, std::string("tauSim"), dummy, true, fs, variable_ptr);
	result &= load_vector(tau, std::string("tau"), dummy, true, fs, variable_ptr);

	beacls::closeMatVariable(variable_ptr);
	beacls::closeMatFStream(fs);

	const auto now = std::chrono::system_clock::now();
	const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
	std::stringstream now_ss;
	now_ss << std::fixed << std::setprecision(6) << ((long double)now_ms.count() / 1000 / 60 / 60 / 24) << std::resetiosflags(std::ios_base::floatfield);
	folder = __func__ + std::string("_") + now_ss.str();
#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	fs::create_directories(folder);
	const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
	const std::string SPPP_filename = folder + std::string("_SPPP.mat");
#endif
	beacls::MatVariable* dst_variable_ptr = beacls::createMatStruct(std::string("SPPP"));
	save(dst_variable_ptr);
	beacls::MatFStream* dst_fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
	beacls::writeMatVariable(dst_fs, dst_variable_ptr);
	beacls::closeMatVariable(dst_variable_ptr);
	beacls::closeMatFStream(dst_fs);
	}

bool SeqPP::SPPProblem::gen_targets_initStates(
	std::vector<beacls::FloatVec>& init_States,
	std::vector<beacls::FloatVec>& tarCenters,
	beacls::IntegerVec& tarCount,
	const Setup_Type setup_name,
	const size_t numVeh,
	const bool randomSeedForTargets

) {
	std::random_device randomDevice;
	std::vector<uint32_t> randomSeedVector(10);
	std::generate(randomSeedVector.begin(), randomSeedVector.end(), std::ref(randomDevice));
	std::seed_seq randomSeed(randomSeedVector.begin(), randomSeedVector.end());


	std::mt19937_64 mt = randomSeedForTargets ? std::mt19937_64(randomSeed) : std::mt19937_64(0);

	switch (setup_name) {
	case Setup_Type_SF:
	{
		std::vector<beacls::FloatVec> targetCentersSet{
			{ 300, 400 },
			{ 50, 175 },
			{ 75, 25 },
			{ 450, 25 }
		};

		const beacls::FloatVec initPos{ (FLOAT_TYPE)475, (FLOAT_TYPE)200 };


		init_States.resize(numVeh);
		tarCenters.resize(numVeh);
		tarCount.resize(targetCentersSet.size(), 0);

		std::uniform_int_distribution<size_t> randomIntDistribution(0, targetCentersSet.size() - 1);
		for (size_t vehicle = 0; vehicle < numVeh; ++vehicle) {
			const size_t target_ind = randomIntDistribution(mt);
			tarCount[target_ind]++;
			tarCenters[vehicle] = targetCentersSet[target_ind];
			beacls::FloatVec targetDirection(initPos.size());
			std::transform(tarCenters[vehicle].cbegin(), tarCenters[vehicle].cend(), initPos.cbegin(), targetDirection.begin(), [](const auto& lhs, const auto& rhs) { return lhs - rhs; });

			const FLOAT_TYPE angle = std::atan2(targetDirection[1], targetDirection[0]);
			const FLOAT_TYPE lambda = (FLOAT_TYPE)(angle - std::floor(angle / (2 * M_PI)) * (2 * M_PI));
			const FLOAT_TYPE initHeading = (FLOAT_TYPE)((lambda == 0 && (angle > 0)) ? 2 * M_PI : lambda);
			beacls::FloatVec initState = initPos;
			initState.push_back(initHeading);
			init_States[vehicle] = initState;
		}

	}
	break;
	case Setup_Type_Bay_Area:
		break;
	default:
		break;
	}
	return true;
}
#if defined(DEBUG_GRID)
static void plot(const HJI_Grid* g, beacls::FloatVec& v) {
	for (size_t j = 0; j < g->get_N(1); ++j) {
		const size_t width = g->get_N(0);
		for (size_t i = 0; i < width; ++i) {
			if (v[j*width + i] < 0)
				std::cout << ".";
			else
				std::cout << " ";
		}
		std::cout << std::endl;
	}
}
#endif
bool SeqPP::SPPProblem::loadSetup(const Setup_Type setup_name, const SetupExtraArgs& extraArgs) {
	switch (setup_name) {
	case Setup_Type_SF:
	{
		//!< Vehicle
		vRangeA = beacls::FloatVec{ (FLOAT_TYPE)0, (FLOAT_TYPE)2.5 };
		wMaxA = 2;

		//!< RTT
		vReserved = beacls::FloatVec{ (FLOAT_TYPE)1, (FLOAT_TYPE)-1.2 };
		wReserved = (FLOAT_TYPE)-0.8;

		//!< Initial states
		const size_t numVeh = extraArgs.number_of_vehicles;

		//!< Sampling and collision radius
		max_BRS_time = 500;
		dt = (FLOAT_TYPE)0.5;
		Rc = 1;

		//!< Initial targets
		targetR = 10;

		if (!extraArgs.ISTC_filename.empty()) {
			std::cout << "Loading initial states and target centers..." << std::endl;
			beacls::MatFStream* fs = beacls::openMatFStream(extraArgs.ISTC_filename, beacls::MatOpenMode_Read);
			if (fs) {
				bool result = true;
				beacls::IntegerVec dummy;
				result &= load_vector_of_vectors(initStates, std::string("IS"), dummy, true, fs);
				result &= load_vector_of_vectors(targetCenters, std::string("TC"), dummy, true, fs);
				beacls::closeMatFStream(fs);
			}
		}
		if (initStates.empty() || targetCenters.empty() || (initStates.size() != targetCenters.size())) {
			beacls::IntegerVec targetCounts;
			gen_targets_initStates(initStates, targetCenters, targetCounts, setup_name, numVeh, extraArgs.randomSeedForTargets);
#if 1
			beacls::MatFStream* fs = beacls::openMatFStream(extraArgs.ISTC_filename, beacls::MatOpenMode_Write);
			if (fs) {
				bool result = true;
				beacls::IntegerVec dummy(1);
				dummy[0] = initStates[0].size();
				result &= save_vector_of_vectors(initStates, std::string("IS"), dummy, true, fs);
				result &= save_vector_of_vectors(targetCenters, std::string("TC"), dummy, true, fs);
				result &= save_vector(targetCounts, std::string("tarCount"), dummy, true, fs);
				beacls::closeMatFStream(fs);
			}
#endif
		}
		//!< Grid
		//!< Defaults
		gMin = beacls::FloatVec{ (FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)0 };
		gMax = beacls::FloatVec{ (FLOAT_TYPE)500, (FLOAT_TYPE)500, (FLOAT_TYPE)(2 * M_PI) };
		gN = beacls::IntegerVec{ 101, 101, 15 };

		//!< Custom modifications
		if (extraArgs.dstbOrIntr == DstbIntr_Type_Dstb && extraArgs.wind_speed == 6) {
			gN = beacls::IntegerVec{ 201, 201, 15 };
		}
		else if (extraArgs.dstbOrIntr == DstbIntr_Type_Intr) {
			gMin = beacls::FloatVec{ (FLOAT_TYPE)-50, (FLOAT_TYPE)-50, (FLOAT_TYPE)0 };
			gMax = beacls::FloatVec{ (FLOAT_TYPE)550, (FLOAT_TYPE)550, (FLOAT_TYPE)(2 * M_PI) };
			gN = beacls::IntegerVec{ 125, 125, 15 };
		}


		g = createGrid(gMin, gMax, gN, beacls::IntegerVec{2});
		g2D = createGrid(
			beacls::FloatVec(gMin.cbegin(), gMin.cbegin() + 2),
			beacls::FloatVec(gMax.cbegin(), gMax.cbegin() + 2),
			beacls::IntegerVec(gN.cbegin(), gN.cbegin() + 2));

		//!< Obstacles
		HJI_Grid* temp_g2D = createGrid(
			beacls::FloatVec{-35, -35},
			beacls::FloatVec{35, 35},
			beacls::IntegerVec{101, 101});


		//!< Financial District
		BasicShape* shapeObs1 = new ShapeRectangleByCorner(beacls::FloatVec{300, 250}, beacls::FloatVec{350, 300});
		beacls::FloatVec obs1;
		shapeObs1->execute(g2D, obs1);

		//!< Union Square
		BasicShape* shapeObs2 = new ShapeRectangleByCorner(beacls::FloatVec{-25, -30}, beacls::FloatVec{25, 30});
		beacls::FloatVec obs2;
		shapeObs2->execute(temp_g2D, obs2);
		beacls::FloatVec obs2_rot;
		helperOC::rotateData(obs2_rot, temp_g2D, obs2, (FLOAT_TYPE)(7.5*M_PI / 180), beacls::IntegerVec{0, 1}, beacls::IntegerVec());
		HJI_Grid* obs2_gShift = helperOC::shiftGrid(temp_g2D, beacls::FloatVec{325, 185});
		helperOC::migrateGrid(obs2, obs2_gShift, obs2_rot, g2D);
		BasicShape* shapeObs2b = new ShapeHyperplaneByPoint(
			std::vector<beacls::FloatVec>{ {170, 0}, { 400, 230 }},
			beacls::FloatVec{0, 500});
		beacls::FloatVec obs2b;
		shapeObs2b->execute(g2D, obs2b);
		std::transform(obs2.cbegin(), obs2.cend(), obs2b.cbegin(), obs2.begin(), [](const auto& lhs, const auto& rhs) {
			return std::max<FLOAT_TYPE>(lhs, -rhs);
		});
		//!< City Hall
		BasicShape* shapeObs3 = new ShapeRectangleByCorner(beacls::FloatVec{-25, -5}, beacls::FloatVec{25, 5});
		beacls::FloatVec obs3;
		shapeObs3->execute(temp_g2D, obs3);
		beacls::FloatVec obs3_rot;
		helperOC::rotateData(obs3_rot, temp_g2D, obs3, (FLOAT_TYPE)(7.5*M_PI / 180), beacls::IntegerVec{0, 1}, beacls::IntegerVec());
		HJI_Grid* obs3_gShift = helperOC::shiftGrid(temp_g2D, beacls::FloatVec{170, 65});
		helperOC::migrateGrid(obs3, obs3_gShift, obs3_rot, g2D);

		staticObs.resize(obs1.size());

		std::transform(obs1.cbegin(), obs1.cend(), obs2.cbegin(), staticObs.begin(), std::ptr_fun<const FLOAT_TYPE&, const FLOAT_TYPE&>(std::min<FLOAT_TYPE>));
		std::transform(staticObs.cbegin(), staticObs.cend(), obs3.cbegin(), staticObs.begin(), std::ptr_fun<const FLOAT_TYPE&, const FLOAT_TYPE&>(std::min<FLOAT_TYPE>));

		mapFile = std::string("map_streets.png");

		//!< Wind speed
		if (extraArgs.wind_speed == 6) {
			dMaxA = beacls::FloatVec{ (FLOAT_TYPE)0.6, (FLOAT_TYPE)0 };
			RTT_tR = (FLOAT_TYPE)0.5;
		}
		else if (extraArgs.wind_speed == 11) {
			dMaxA = beacls::FloatVec{ (FLOAT_TYPE)1.1, (FLOAT_TYPE)0 };
			RTT_tR = (FLOAT_TYPE)3.5;
		}
		else {
			std::cerr << "Only 6 m/s and 11 m/s winds have been properly implemented!" << std::endl;
		}

		//!< Target separation time
		tTarget.resize(numVeh);
		for (size_t i = 0; i<numVeh; ++i) {
			tTarget[i] = -extraArgs.separation_time * static_cast<FLOAT_TYPE>(i);
		}

		//!< Adjust global time horizon
		tMin = static_cast<FLOAT_TYPE>(-max_BRS_time - numVeh * extraArgs.separation_time);
		if (!tTarget.empty()) {
			tau = generateArithmeticSequence(tMin, dt, beacls::max_value<FLOAT_TYPE>(tTarget.cbegin(), tTarget.cend()));
		}

		//!< Augment static obstacles
		switch (extraArgs.dstbOrIntr) {
		case DstbIntr_Type_Dstb:
		{
			helperOC::AddCRadius()(augStaticObs, g2D, staticObs, RTT_tR);		

			//!< Boundary obstacle
			const beacls::FloatVec& mins = g->get_mins();
			const beacls::FloatVec& maxs = g->get_maxs();
			beacls::FloatVec margin{ 5, 5, -std::numeric_limits<FLOAT_TYPE>::infinity() };
			beacls::FloatVec mins_plus_5(mins.size());
			beacls::FloatVec maxs_minus_5(maxs.size());
			std::transform(mins.cbegin(), mins.cend(), margin.cbegin(), mins_plus_5.begin(), [](const auto& lhs, const auto& rhs) { return lhs + rhs; });
			std::transform(maxs.cbegin(), maxs.cend(), margin.cbegin(), maxs_minus_5.begin(), [](const auto& lhs, const auto& rhs) { return lhs - rhs; });
			BasicShape* shapeObs4 = new ShapeRectangleByCorner(mins_plus_5, maxs_minus_5);
			beacls::FloatVec obs_bdry;
			shapeObs4->execute(g, obs_bdry);
			std::transform(obs_bdry.cbegin(), obs_bdry.cend(), obs_bdry.begin(), std::negate<FLOAT_TYPE>());
			std::transform(augStaticObs.cbegin(), augStaticObs.cend(),obs_bdry.cbegin(), augStaticObs.begin(), std::ptr_fun<const FLOAT_TYPE&, const FLOAT_TYPE&>(std::min<FLOAT_TYPE>));

			if (shapeObs4) delete shapeObs4;
		}
			break;
		case DstbIntr_Type_Intr:
			tIAT = 10;
			std::cerr << "Static obstacles will be augmented after computing bufferRegion and FRSBRS" << std::endl;
			break;
		default:
			std::cerr << "The property ''dstb_or_intr'' must be ''dstb'' or ''intr''!" << std::endl;
			return false;
		}


		if (temp_g2D) delete temp_g2D;
		if (obs2_gShift) delete obs2_gShift;
		if (shapeObs1) delete shapeObs1;
		if (shapeObs2) delete shapeObs2;
		if (shapeObs2b) delete shapeObs2b;
		if (shapeObs3) delete shapeObs3;

	}
		break;
	case Setup_Type_Bay_Area:
		{
			//!< Vehicle
			vRangeA = beacls::FloatVec{ (FLOAT_TYPE)0, (FLOAT_TYPE)2.5 };
			wMaxA = 2;

			//!< RTT
			vReserved = beacls::FloatVec{ (FLOAT_TYPE)1, (FLOAT_TYPE)-1.2 };
			wReserved = (FLOAT_TYPE)-0.8;

			//!< Initial states
			size_t numVeh = 0;

			const std::vector< beacls::FloatVec> IC_and_target_centers = { 
				{ 350, 180},
				{1190, 380},
				{ 1150, 1070 },
				{ 650, 1620} };

			//!< Initial targets
			targetR = 10;
			const size_t numTar = IC_and_target_centers.size();
			std::vector<beacls::IntegerVec> tarCounts(numTar);
			std::for_each(tarCounts.begin(), tarCounts.end(), [&numTar](auto& rhs) {
				rhs.resize(numTar, 0);
			});

			std::uniform_int_distribution<size_t> randomIntDistribution(0, numTar - 1);
			if (!extraArgs.ISTC_filename.empty()) {
				std::cout << "Loading initial states and target centers..." << std::endl;
				beacls::MatFStream* fs = beacls::openMatFStream(extraArgs.ISTC_filename, beacls::MatOpenMode_Read);
				if (fs) {
					bool result = true;
					beacls::IntegerVec dummy;
					result &= load_vector_of_vectors(initStates, std::string("IS"), dummy, true, fs);
					result &= load_vector_of_vectors(targetCenters, std::string("TC"), dummy, true, fs);
					std::vector<beacls::FloatVec> tarCounts_f(tarCounts.size());
					beacls::IntegerVec dummy2;
					result &= load_vector_of_vectors(tarCounts_f, std::string("tarCount"), dummy2, true, fs);
					std::transform(tarCounts_f.cbegin(), tarCounts_f.cend(), tarCounts.begin(), [](const auto&rhs) {
						beacls::IntegerVec tmp(rhs.size());
						std::transform(rhs.cbegin(), rhs.cend(), tmp.begin(), [](const auto&rhs) {
							return (size_t)rhs;
						});
						return tmp;
					});
					beacls::closeMatFStream(fs);
					numVeh = initStates.size();
				}
			}
			if (initStates.empty() || targetCenters.empty() || (initStates.size() != targetCenters.size())) {
				std::random_device randomDevice;
				std::vector<uint32_t> randomSeedVector(10);
				std::generate(randomSeedVector.begin(), randomSeedVector.end(), std::ref(randomDevice));
				std::seed_seq randomSeed(randomSeedVector.begin(), randomSeedVector.end());

				std::mt19937_64 mt = extraArgs.randomSeedForTargets ? std::mt19937_64(randomSeed) : std::mt19937_64(0);

				numVeh = extraArgs.number_of_vehicles;
				initStates.resize(numVeh);
				targetCenters.resize(numVeh);
				for (size_t i = 0; i < numVeh; ++i) {
					//!< Randomize target and IC
					const size_t IC_index = randomIntDistribution(mt);
					size_t target_index = randomIntDistribution(mt);
					while (target_index == IC_index) {
						target_index = randomIntDistribution(mt);
					}
					tarCounts[IC_index][target_index]++;

					//!< Assign IC and target
					beacls::FloatVec IC_pos = IC_and_target_centers[IC_index];
					beacls::FloatVec target_pos = IC_and_target_centers[target_index];

					beacls::FloatVec delta(target_pos.size());
					std::transform(target_pos.cbegin(), target_pos.cend(), IC_pos.cbegin(), delta.begin(), std::minus<FLOAT_TYPE>());
					FLOAT_TYPE IC_angle = std::atan2(delta[1], delta[0]);
					IC_angle = (FLOAT_TYPE)(IC_angle - std::floor(IC_angle / (2 * M_PI)) * (2 * M_PI));

					initStates[i] = IC_pos;
					initStates[i].push_back(IC_angle);

					targetCenters[i] = target_pos;
					targetCenters[i].push_back(0);
				}
#if 1
				beacls::MatFStream* fs = beacls::openMatFStream(extraArgs.ISTC_filename, beacls::MatOpenMode_Write);
				if (fs) {
					bool result = true;
					beacls::IntegerVec dummy(1);
					dummy[0] = initStates[0].size();
					result &= save_vector_of_vectors(initStates, std::string("IS"), dummy, true, fs);
					result &= save_vector_of_vectors(targetCenters, std::string("TC"), dummy, true, fs);
					beacls::IntegerVec dummy2(1);
					dummy2[0] = tarCounts.size();
					std::vector<beacls::FloatVec> tarCounts_f(tarCounts.size());
					std::transform(tarCounts.cbegin(), tarCounts.cend(), tarCounts_f.begin(), [](const auto&rhs) {
						beacls::FloatVec tmp(rhs.size());
						std::transform(rhs.cbegin(), rhs.cend(), tmp.begin(), [](const auto&rhs) {
							return (FLOAT_TYPE)rhs;
						});
						return tmp;
					});
					result &= save_vector_of_vectors(tarCounts_f, std::string("tarCount"), dummy2, true, fs);
					beacls::closeMatFStream(fs);
				}
#endif
			}
			std::cout << "tarCount = " << std::endl;
			std::for_each(tarCounts.cbegin(), tarCounts.cend(), [](const auto& rhs) {
				std::for_each(rhs.cbegin(), rhs.cend(), [](const auto& rhs) {
					std::cout << std::setw(6) << std::left << rhs;
				});
				std::cout << std::resetiosflags(std::ios_base::floatfield) << std::endl;
				});

			//!< Sampling and collision radius
			max_BRS_time = 1500;
			dt = (FLOAT_TYPE)0.5;
			Rc = 1;

			//!< Grid
			gMin = beacls::FloatVec{ (FLOAT_TYPE)250, (FLOAT_TYPE)80, (FLOAT_TYPE)0 };
			gMax = beacls::FloatVec{ (FLOAT_TYPE)1290, (FLOAT_TYPE)1720, (FLOAT_TYPE)(2 * M_PI) };
			gN = beacls::IntegerVec{ 209, 329, 15 };

			g = createGrid(gMin, gMax, gN, beacls::IntegerVec{2});
			g2D = createGrid(
				beacls::FloatVec(gMin.cbegin(), gMin.cbegin() + 2),
				beacls::FloatVec(gMax.cbegin(), gMax.cbegin() + 2),
				beacls::IntegerVec(gN.cbegin(), gN.cbegin() + 2));

			mapFile = std::string("bay_area_streets.png");

			//!< Wind speed
			if (extraArgs.wind_speed == 11) {
				dMaxA = beacls::FloatVec{ (FLOAT_TYPE)1.1, (FLOAT_TYPE)0 };
				RTT_tR = (FLOAT_TYPE)3.5;
			}
			else {
				std::cerr << "Only 11 m/s wind have been properly implemented!" << std::endl;
			}

			//!< Target separation time
			//!< Scheduled times of arrival
			tTarget.resize(numVeh);
			for (size_t i = 0; i<numVeh; ++i) {
				tTarget[i] = -extraArgs.separation_time * static_cast<FLOAT_TYPE>(i);
			}

			//!< Adjust global time horizon
			tMin = static_cast<FLOAT_TYPE>(-max_BRS_time - numVeh * extraArgs.separation_time);
			if (!tTarget.empty()) {
				tau = generateArithmeticSequence(tMin, dt, beacls::max_value<FLOAT_TYPE>(tTarget.cbegin(), tTarget.cend()));
			}
	}
		break;
	default:
		std::cerr << "Unknown setup_name! : " << setup_name << std::endl;
		return false;
	}
	return true;
}

bool SeqPP::SPPProblem::save(
	beacls::MatVariable* variable_ptr
) {
	bool result = false;
	if (!initStates.empty()) result &= save_vector_of_vectors(initStates, std::string("initStates"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (!targetCenters.empty()) result &= save_vector_of_vectors(targetCenters, std::string("targetCenters"), beacls::IntegerVec(), true, NULL, variable_ptr);

	result &= save_value(targetR, std::string("targetR"), true, NULL, variable_ptr);
	if (!targetRsmall.empty()) result &= save_vector_of_vectors(targetRsmall, std::string("targetRsmall"), beacls::IntegerVec(), true, NULL, variable_ptr);

	result &= save_value(tIAT, std::string("tIAT"), true, NULL, variable_ptr);

	result &= save_value(max_num_affected_vehicles, std::string("max_num_affected_vehicles"), true, NULL, variable_ptr);
	result &= save_value(buffer_duration, std::string("buffer_duration"), true, NULL, variable_ptr);
	result &= save_value(buffer_duration_ind, std::string("buffer_duration_ind"), true, NULL, variable_ptr);
	result &= save_value(remaining_duration_ind, std::string("remaining_duration_ind"), true, NULL, variable_ptr);

	result &= save_value(Rc, std::string("Rc"), true, NULL, variable_ptr);
	beacls::IntegerVec g2DNs = g2D->get_Ns();

	if (!staticObs.empty()) result &= save_vector(staticObs, std::string("staticObs"), g2DNs, true, NULL, variable_ptr);
	if (!augStaticObs.empty()) result &= save_vector(augStaticObs, std::string("augStaticObs"), g2DNs, true, NULL, variable_ptr);

	if (!vRangeA.empty()) result &= save_vector(vRangeA, std::string("vRangeA"), beacls::IntegerVec(), true, NULL, variable_ptr);
	result &= save_value(wMaxA, std::string("wMaxA"), true, NULL, variable_ptr);
	if (!dMaxA.empty()) result &= save_vector(dMaxA, std::string("dMaxA"), beacls::IntegerVec(), true, NULL, variable_ptr);

	if (!vReserved.empty()) result &= save_vector(vReserved, std::string("vReserved"), beacls::IntegerVec(), true, NULL, variable_ptr);
	result &= save_value(wReserved, std::string("wReserved"), true, NULL, variable_ptr);
	result &= save_value(RTT_tR, std::string("RTT_tR"), true, NULL, variable_ptr);

	result &= save_value(tMin, std::string("tMin"), true, NULL, variable_ptr);
	result &= save_value(dt, std::string("dt"), true, NULL, variable_ptr);
	if (!tTarget.empty()) result &= save_vector(tTarget, std::string("tTarget"), beacls::IntegerVec(), true, NULL, variable_ptr);
	result &= save_value(tIntr, std::string("tIntr"), true, NULL, variable_ptr);
	if (!tReplan.empty()) result &= save_vector(tReplan, std::string("tReplan"), beacls::IntegerVec(), true, NULL, variable_ptr);
	result &= save_value(max_BRS_time, std::string("max_BRS_time"), true, NULL, variable_ptr);

	result &= save_value(rttrs_tMax, std::string("rttrs_tMax"), true, NULL, variable_ptr);
	if (!rttrs_gN.empty()) result &= save_vector(rttrs_gN, std::string("rttrs_gN"), beacls::IntegerVec(), true, NULL, variable_ptr);

	if (!gMin.empty()) result &= save_vector(gMin, std::string("gMin"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (!gMax.empty()) result &= save_vector(gMax, std::string("gMax"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (!gN.empty()) result &= save_vector(gN, std::string("gN"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (g) result &= g->save_grid(std::string("g"), NULL, variable_ptr);
	if (g2D) result &= g2D->save_grid(std::string("g2D"), NULL, variable_ptr);

	if (!tauBR.empty()) result &= save_vector(tauBR, std::string("tauBR"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (!tauAR.empty()) result &= save_vector(tauAR, std::string("tauAR"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (!tauSim.empty()) result &= save_vector(tauSim, std::string("tauSim"), beacls::IntegerVec(), true, NULL, variable_ptr);
	if (!tau.empty()) result &= save_vector(tau, std::string("tau"), beacls::IntegerVec(), true, NULL, variable_ptr);
	return result;
}
SeqPP::SPPProblem::~SPPProblem() {
	if (matWriter) delete matWriter;
	if (migrateRTTRS) delete migrateRTTRS;
	if (nirs) delete nirs;
	if (rttrs) delete rttrs;
	if (g) delete g;
	if (g2D) delete g2D;
}

bool SeqPP::SPPProblem::computeRTTRS(
	const bool save_png
) {
	if (rttrs) delete rttrs;
		rttrs = new RTTRS();
	if (!RTTRS_filename.empty()) {
		std::ifstream ifs(RTTRS_filename.c_str());
		if (!ifs.fail()) {
			ifs.close();
			beacls::MatFStream* src_rttrs_fs = beacls::openMatFStream(RTTRS_filename, beacls::MatOpenMode_Read);
			beacls::MatVariable* rttrs_var = NULL;
			bool rttrs_load = false;
			if (src_rttrs_fs) {
				rttrs_var = beacls::openMatVariable(src_rttrs_fs, std::string("RTTRS"));
				if (rttrs_var) {
					rttrs_load = rttrs->load(rttrs_var);
					beacls::closeMatVariable(rttrs_var);
				}
				beacls::closeMatFStream(src_rttrs_fs);
			}
			if (rttrs_load) {
				std::cout << "The RTTRS file " << RTTRS_filename.c_str() << " already exists. Skipping RTTRS computation." << std::endl;
				return true;
			}
		}
		RTTRS_filename.clear();
	}

	const FLOAT_TYPE tR = RTT_tR;

	//!< Grid
	const beacls::FloatVec grid_min{ -static_cast<FLOAT_TYPE>(1.25*tR), -static_cast<FLOAT_TYPE>(1.25*tR), (FLOAT_TYPE)(-M_PI) };	//!< Lower corner of computation domain
	const beacls::FloatVec grid_max{ static_cast<FLOAT_TYPE>(1.25*tR), static_cast<FLOAT_TYPE>(1.25*tR), (FLOAT_TYPE)M_PI };		//!< Upper corner of computation domain
	//!< Number of grid points per dimension
	//const beacls::IntegerVec N{ 101, 101, 101 };	//!< for SPPwIntruderRTT method 1
	//const beacls::IntegerVec N{ 51, 51, 101 };	//!< for SPPwIntruderRTT method 2
	const beacls::IntegerVec N = rttrs_gN;	//!< for SPPwIntruderRTT method 2
	HJI_Grid* grid = createGrid(grid_min, grid_max, N, beacls::IntegerVec{2});
	DynSysSchemeData* schemeData = new DynSysSchemeData;
	schemeData->set_grid(grid);

	//!< Track trajectory for up to this time
	// const FLOAT_TYPE tMax = 2; //!< for SPPwIntruderRTT method 1
	//	const FLOAT_TYPE tMax = 30; //!< for SPPwIntruderRTT method 2
	const FLOAT_TYPE tMax = rttrs_tMax;
//	std::cout << __FILE__ << ":" << __LINE__ << ": Temporal setting for debug. tMax = " << tMax << std::endl;
	const FLOAT_TYPE rttrs_dt = (FLOAT_TYPE)0.1;
	std::vector <FLOAT_TYPE> rttrs_tau = generateArithmeticSequence<FLOAT_TYPE>(0., rttrs_dt, tMax);

	//!< Virtual vehicle to be tracked
	beacls::FloatVec vRangeB(vRangeA.size());
	std::transform(vRangeA.cbegin(), vRangeA.cend(), vReserved.cbegin(), vRangeB.begin(), [](const auto& lhs, const auto& rhs) {return lhs + rhs; });
	const FLOAT_TYPE wMaxB = wMaxA + wReserved;
	const beacls::FloatVec dMaxB{ 0,0 };
	PlaneCAvoid* dynSys = new PlaneCAvoid(beacls::FloatVec{(FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)0}, wMaxA, vRangeA, wMaxB, vRangeB, dMaxA, dMaxB);
	schemeData->dynSys = dynSys;

	//!< Initial conditions
	rttrs->set_trackingRadius(tR);
	beacls::FloatVec data0;
	BasicShape* shape = new ShapeCylinder(
		beacls::IntegerVec{2}, 
		beacls::FloatVec{0., 0., 0},
		rttrs->get_trackingRadius());
	shape->execute(schemeData->get_grid(), data0);
	std::transform(data0.cbegin(), data0.cend(), data0.begin(), std::negate<FLOAT_TYPE>());
	
	schemeData->uMode = DynSys_UMode_Max;
	schemeData->dMode = DynSys_DMode_Min;
	helperOC::HJIPDE_extraArgs extraArgs;
	helperOC::HJIPDE_extraOuts extraOuts;
	extraArgs.visualize = false;
	extraArgs.deleteLastPlot = true;
	extraArgs.stopInit = beacls::FloatVec{ 0,0,0 };

	extraArgs.keepLast = keepLast;
	extraArgs.execParameters = execParameters;

	if (save_png) {
#if defined(FILESYSTEM)	//!< T.B.D>
		//!< I want to implement making folder after std::filesystem is standardlized in C++17.
		extraArgs.fig_filename = folder + std::string("/") + __func__;
		fs::create_directories(extraArgs.fig_filename);
#else
		extraArgs.fig_filename = folder + std::string("_") + __func__;
#endif
	}
	HJIPDE* hjipde = new HJIPDE();
	beacls::FloatVec stoptau;
	hjipde->solve(stoptau, extraOuts, data0, rttrs_tau, schemeData, HJIPDE::MinWithType_Zero, extraArgs);

	rttrs->set_g(schemeData->get_grid());
	beacls::FloatVec last_data;
	hjipde->get_last_data(last_data);
	rttrs->set_data(last_data);
	rttrs->set_dynSys(dynSys);

	//!< Save results;
#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	RTTRS_filename = folder + std::string("/RTTRS.mat");
#else
	RTTRS_filename = folder + std::string("_RTTRS.mat");
#endif
	beacls::MatVariable* rttrs_ptr = beacls::createMatStruct(std::string("RTTRS"));
	rttrs->save(rttrs_ptr);
	beacls::MatFStream* dst_rttrs_fs = beacls::openMatFStream(RTTRS_filename, beacls::MatOpenMode_Write);
	beacls::writeMatVariable(dst_rttrs_fs, rttrs_ptr);
	beacls::closeMatVariable(rttrs_ptr);
	beacls::closeMatFStream(dst_rttrs_fs);

#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
	const std::string SPPP_filename = folder + std::string("_SPPP.mat");
#endif
	beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("SPPP"));
	save(variable_ptr);
	beacls::MatFStream* fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
	beacls::writeMatVariable(fs, variable_ptr);
	beacls::closeMatVariable(variable_ptr);
	beacls::closeMatFStream(fs);

	if (hjipde) delete hjipde;
	if (shape) delete shape;
	if (dynSys) delete dynSys;
	if (schemeData) delete schemeData;
	if (grid) delete grid;
	return true;
}

bool SeqPP::SPPProblem::computeCARS(
	const Plane* qintr,
	const bool save_png,
	const bool restart
) {
	if (!restart) {
		if (!CARS_filename.empty()) {
			std::ifstream ifs(CARS_filename.c_str());
			if (!ifs.fail()) {
				ifs.close();
				std::cout << "The CARS file " << CARS_filename.c_str() << " already exists. Skipping CARS computation." << std::endl;
				return true;
			}
		}
	}
	const Plane* modified_qintr = (qintr) ? qintr : new Plane(beacls::FloatVec{0, 0, 0}, wMaxA, vRangeA, dMaxA);
	DynSysSchemeData* schemeData = new DynSysSchemeData;
	PlaneCAvoid * dynSys = new PlaneCAvoid(
		beacls::FloatVec{0, 0, 0},
		wMaxA, vRangeA,
		modified_qintr->get_wMax(), 
		modified_qintr->get_vrange(), 
		dMaxA, modified_qintr->get_dMax());
	schemeData->dynSys = dynSys;
	schemeData->uMode = DynSys_UMode_Max;
	schemeData->dMode = DynSys_DMode_Min;

	//!< Grid and target set
	//!<for SPPwIntruderRTT method 1
	// beacls::FloatVec grid_min{ (FLOAT_TYPE)-3 * Rc, (FLOAT_TYPE)-4 * Rc, (FLOAT_TYPE)0 };
	// beacls::FloatVec grid_max{ (FLOAT_TYPE)5 * Rc, (FLOAT_TYPE)4 * Rc, (FLOAT_TYPE)2 * M_PI };
	// beacls::IntegerVec Ns{ 101,101,101 };

	//!<for SPPwIntruderRTT method 2 with 11 m / s wind
	// beacls::FloatVec grid_min{ (FLOAT_TYPE)-27 * Rc, (FLOAT_TYPE)-27 * Rc, (FLOAT_TYPE)0 };
	// beacls::FloatVec grid_max{ (FLOAT_TYPE)27 * Rc, (FLOAT_TYPE)27 * Rc, (FLOAT_TYPE)2 * M_PI };

	//!<for SPPwIntruderRTT method 2 with 6 m / s wind
	beacls::FloatVec grid_min{ (FLOAT_TYPE)-20, (FLOAT_TYPE)-20, (FLOAT_TYPE)0 };
	beacls::FloatVec grid_max{ (FLOAT_TYPE)20, (FLOAT_TYPE)20, (FLOAT_TYPE)(2 * M_PI) };

	beacls::IntegerVec Ns{ 41,41,41 };
	
	//!< 3rd dimension is periodic
	HJI_Grid* grid = createGrid(grid_min, grid_max, Ns, beacls::IntegerVec{2});
	schemeData->set_grid(grid);

	ShapeCylinder* shape = new ShapeCylinder(beacls::IntegerVec{2}, beacls::FloatVec{0, 0, 0}, Rc);
	beacls::FloatVec data0;
	shape->execute(grid, data0);

	//!< Time stamps
	const beacls::FloatVec src_tau = generateArithmeticSequence((FLOAT_TYPE)0, dt, tIAT);

	//!< Compute set
	helperOC::HJIPDE_extraArgs extraArgs;
	extraArgs.visualize = true;
	extraArgs.deleteLastPlot = true;

	if (save_png) {
#if defined(FILESYSTEM)	//!< T.B.D>
		//!< I want to implement making folder after std::filesystem is standardlized in C++17.
		std::string data_folder = folder + std::string("/") + __func__;
		extraArgs.fig_filename = data_folder + std::string("/");
#else
		std::string data_folder = folder + std::string("_") + __func__;
		extraArgs.fig_filename = data_folder + std::string("_");
#endif
	}
	extraArgs.execParameters = execParameters;

	HJIPDE* hjipde = new HJIPDE;
	beacls::FloatVec dst_tau;
	helperOC::HJIPDE_extraOuts extraOuts;
	hjipde->solve(dst_tau, extraOuts, data0, src_tau, schemeData, HJIPDE::MinWithType_Zero, extraArgs);

	std::vector<beacls::FloatVec> datas;
	hjipde->get_datas(datas, src_tau, schemeData);
	cars->set_dynSys(dynSys);
	cars->set_g(grid);
	cars->set_data(datas);
	cars->set_tau(src_tau);

	//!< Update SPPP and save
#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	CARS_filename = folder + std::string("/CARS.mat");
#else
	CARS_filename = folder + std::string("_CARS.mat");
#endif
	beacls::MatVariable* cars_ptr = beacls::createMatStruct(std::string("CARS"));
	beacls::MatFStream* cars_fs = beacls::openMatFStream(CARS_filename, beacls::MatOpenMode_Write);
	cars->save(cars_fs, cars_ptr);
	beacls::writeMatVariable(cars_fs, cars_ptr);
	beacls::closeMatVariable(cars_ptr);
	beacls::closeMatFStream(cars_fs);

	if (!src_tau.empty()) {
		buffer_duration = beacls::max_value<FLOAT_TYPE>(src_tau.cbegin(), src_tau.cend()) / max_num_affected_vehicles;
	}
	
	auto first_ite = std::find_if(src_tau.cbegin(), src_tau.cend(), [this](const auto& rhs) { return ( rhs >= buffer_duration); });
	buffer_duration_ind = std::distance(src_tau.cbegin(), first_ite);

#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
	const std::string SPPP_filename = folder + std::string("_SPPP.mat");
#endif
	beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("SPPP"));
	save(variable_ptr);
	beacls::MatFStream* fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
	beacls::writeMatVariable(fs, variable_ptr);
	beacls::closeMatVariable(variable_ptr);
	beacls::closeMatFStream(fs);


	if (hjipde) delete hjipde;

	if (dynSys) delete dynSys;
	if (shape) delete shape;
	if (grid) delete grid;

	if (!qintr && modified_qintr) delete modified_qintr;
	if (schemeData) delete schemeData;
	return true;
}

void SeqPP::saveNI_RS_chkpt_and_clear(
	MatWriter* matWriter,
	const std::string filename,
	SeqPP::NIRS* nirs,
	SeqPP::Obstacles* obstacles_ptr,
	const size_t vehicle_index,
	const CheckPointType checkPointType,
	const size_t stride,
	const bool saveBRS1,
	const bool saveObstacles) {
	SPPPlane* vehicle = nirs->get_Vehicle(vehicle_index);
	if (!vehicle) {
		nirs->set_to_save(false);
		return;
	}
	if (!saveBRS1) {
		vehicle->clear_BRS1();
	}

	if (checkPointType == CheckPointType_Merged) {
		bool result = true;
		std::cout << "Saving NIRS check point to " << filename.c_str() << std::endl;
		matWriter->mtx.lock();
		beacls::MatFStream* fs = beacls::openMatFStream(filename, beacls::MatOpenMode_Write);
		result &= nirs->save(fs);
		if (saveObstacles) {
			beacls::MatVariable* obstacles_var = beacls::createMatStruct(std::string("obstacles"));
			result &= save_vector_of_vectors(obstacles_ptr->data, std::string("data"), beacls::IntegerVec(), true, fs, obstacles_var, 0, false);
			obstacles_ptr->data.clear();
			std::vector<beacls::FloatVec>().swap(obstacles_ptr->data);
			result &= save_vector_of_vectors(obstacles_ptr->data_s8, std::string("data_s8"), beacls::IntegerVec(), true, fs, obstacles_var, 0, false);
			obstacles_ptr->data_s8.clear();
			std::vector<std::vector<int8_t>>().swap(obstacles_ptr->data_s8);
			result &= save_vector(obstacles_ptr->tau, std::string("tau"), beacls::IntegerVec(), true, fs, obstacles_var, 0, false);
			result &= save_value(vehicle_index, std::string("veh"), true, fs);
			obstacles_ptr->tau.clear();
			delete obstacles_ptr;
			beacls::writeMatVariable(fs, obstacles_var, false);
			beacls::closeMatVariable(obstacles_var);
		}
		else {
			if (obstacles_ptr)
				delete obstacles_ptr;
			result &= save_value(vehicle_index, std::string("veh"), true, fs);
		}
		if (saveBRS1) {
			vehicle->clear_BRS1();
		}
		vehicle->clear_obsForRTT();
		vehicle->clear_obsForRTT_s8();
		nirs->set_to_save(false);
		beacls::closeMatFStream(fs);
		matWriter->mtx.unlock();
		if (!result) {
			std::cerr << "Failed NIRS check point to " << filename.c_str() << std::endl;
		}
		else {
			std::cout << "Saved NIRS check point to " << filename.c_str() << std::endl;
		}
	}
	else if (checkPointType == CheckPointType_Separated) {
		bool result = true;
		size_t num_of_vehicles_to_write = (vehicle_index >= stride) && (stride >= 1) ? 2 : 1;
		for (size_t i = 0; i <num_of_vehicles_to_write; i++) {
			size_t vehicle_index_to_write;
			std::stringstream vehicle_name_ss;
			if (i==1) {
				vehicle_index_to_write = vehicle_index - stride;
				vehicle_name_ss << "_Plane" << vehicle_index_to_write;
			}
			else {
				vehicle_index_to_write = vehicle_index;
				vehicle_name_ss << "_Plane" << vehicle_index_to_write;
			}
			std::string nth_NI_RS_chkpt_filename = filename + vehicle_name_ss.str() + std::string(".mat");
			std::cout << "Saving NIRS check point to " << nth_NI_RS_chkpt_filename.c_str() << std::endl;
			SPPPlane* vehicle_to_write = nirs->get_Vehicle(vehicle_index_to_write);
			if (vehicle_to_write) {
				if (i != 0) {
					while (vehicle_to_write->get_to_save()) {
						std::this_thread::yield();
					}
				}
				matWriter->mtx.lock();
				beacls::MatFStream* fs = beacls::openMatFStream(nth_NI_RS_chkpt_filename, beacls::MatOpenMode_Write);
				result &= save_value(vehicle_index, std::string("veh"), true, fs);
				beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("Qthis"));
				result &= vehicle_to_write->save(fs, variable_ptr);
				if (i == num_of_vehicles_to_write - 1) {
					//!< Compress for final save of vehicle.
					beacls::writeMatVariable(fs, variable_ptr, true);
				} else {
					//!< Don't compress for temporal save of vehicle with BRS.
					beacls::writeMatVariable(fs, variable_ptr, false);
				}
				beacls::closeMatVariable(variable_ptr);
				if (i == 0) {
					if (saveObstacles) {
						beacls::MatVariable* obstacles_var = beacls::createMatStruct(std::string("obstacles"));
						result &= save_vector_of_vectors(obstacles_ptr->data, std::string("data"), beacls::IntegerVec(), true, fs, obstacles_var, 0, false);
						obstacles_ptr->data.clear();
						std::vector<beacls::FloatVec>().swap(obstacles_ptr->data);
						result &= save_vector_of_vectors(obstacles_ptr->data_s8, std::string("data_s8"), beacls::IntegerVec(), true, fs, obstacles_var, 0, false);
						obstacles_ptr->data_s8.clear();
						std::vector<std::vector<int8_t>>().swap(obstacles_ptr->data_s8);
						result &= save_vector(obstacles_ptr->tau, std::string("tau"), beacls::IntegerVec(), true, fs, obstacles_var, 0, false);
						delete obstacles_ptr;
						nirs->set_to_save(false);
						if (saveBRS1) {
							vehicle_to_write->clear_BRS1();
						}
						vehicle_to_write->clear_obsForRTT();
						vehicle_to_write->clear_obsForRTT_s8();
						vehicle_to_write->set_to_save(false);
						beacls::writeMatVariable(fs, obstacles_var, false);
						beacls::closeMatVariable(obstacles_var);
					}
					else {
						if (obstacles_ptr)
							delete obstacles_ptr;
						nirs->set_to_save(false);
						if (saveBRS1) {
							vehicle_to_write->clear_BRS1();
						}
						vehicle_to_write->clear_obsForRTT();
						vehicle_to_write->clear_obsForRTT_s8();
						vehicle_to_write->set_to_save(false);
					}
					beacls::closeMatFStream(fs);
					matWriter->mtx.unlock();
				}
				else {
					nirs->clear_Vehicle(vehicle_index_to_write);
					nirs->set_to_save(false);
					beacls::closeMatFStream(fs);
					matWriter->mtx.unlock();
				}
			}

			if (!result) {
				std::cerr << "Failed NIRS check point to " << nth_NI_RS_chkpt_filename.c_str() << std::endl;
			}
			else {
				std::cout << "Saved NIRS check point to " << nth_NI_RS_chkpt_filename.c_str() << std::endl;
			}
		}
	}
	return;
}

bool SeqPP::SPPProblem::computeNIRS(
	const bool restart,
	const bool low_memory,
	const CheckPointType checkPointType,
	const bool save_brs1_file,
	const size_t num_of_vehicles_to_computeNIRS
) {
	//!< Check to see if 
	if (!NI_RS_filename.empty()) {
		std::ifstream ifs(NI_RS_filename.c_str());
		if (!restart && !ifs.fail()) {
			ifs.close();
			std::cout << "The NI RS file " << NI_RS_filename.c_str() << " already exists. Skipping NI RS computation." << std::endl;
			return true;
		}
		NI_RS_filename.empty();
	}

	//!< Load files
	//!< RTTRS
	if (rttrs) delete rttrs;
	rttrs = new RTTRS();
	std::cout << "Loading RTTRS..." << std::endl;
	beacls::MatFStream* rttrs_fs = beacls::openMatFStream(RTTRS_filename, beacls::MatOpenMode_Read);
	beacls::MatVariable* rttrs_var = NULL;
	bool rttrs_load = false;
	if (rttrs_fs) {
		rttrs_var = beacls::openMatVariable(rttrs_fs, std::string("RTTRS"));
		if (rttrs_var) {
			rttrs_load = rttrs->load(rttrs_var);
			beacls::closeMatVariable(rttrs_var);
		}
		beacls::closeMatFStream(rttrs_fs);
	}
	if (!rttrs_load) {
		std::cerr << "RTTRS file " << RTTRS_filename.c_str() << " not found!" << std::endl;
		return false;
	}


	//!< Checkpoint
	if (!nirs) nirs = new NIRS();
	size_t vehStart = 0;
	Obstacles obstacles;

	if (!restart) {
		if (checkPointType == CheckPointType_Merged) {
			beacls::MatFStream* nirs_chkpt_fs = beacls::openMatFStream(NI_RS_chkpt_filename, beacls::MatOpenMode_Read);
			bool nirs_load = false;
			if (nirs_chkpt_fs) {
				std::cout << "Loading NI RS checkpoint..." << std::endl;
				nirs_load = nirs->load(execParameters, nirs_chkpt_fs);
				load_value(vehStart, std::string("veh"), true, nirs_chkpt_fs);
				beacls::MatVariable* obstacles_var = beacls::openMatVariable(nirs_chkpt_fs, std::string("obstacles"));
				beacls::IntegerVec dummy;
				load_vector_of_vectors(obstacles.data, std::string("data"), dummy, true, nirs_chkpt_fs, obstacles_var);
				load_vector_of_vectors(obstacles.data_s8, std::string("data_s8"), dummy, true, nirs_chkpt_fs, obstacles_var);
				load_vector(obstacles.tau, std::string("tau"), dummy, true, nirs_chkpt_fs, obstacles_var);
				beacls::closeMatVariable(obstacles_var);
				beacls::closeMatFStream(nirs_chkpt_fs);
			}
			if (!nirs_load) {
				std::cerr << "Error: Cannot read NI_RS checkpoint file. The RTTRS file " << NI_RS_chkpt_filename.c_str() << std::endl;
				return false;
			}
		}
		else if (checkPointType == CheckPointType_Separated) {
			nirs->initializeQ(initStates.size());
			size_t num_of_checkPointFiles = 0;
			for (size_t plane_index = 0; plane_index < tTarget.size(); ++plane_index) {
				std::stringstream vehicle_name_ss;
				vehicle_name_ss << "_Plane" << plane_index;
				std::string nth_NI_RS_chkpt_filename = NI_RS_chkpt_filename + vehicle_name_ss.str() + std::string(".mat");
				std::fstream nth_NI_RS_chkpt_fs(nth_NI_RS_chkpt_filename);
				if (!nth_NI_RS_chkpt_fs.fail()) {
					num_of_checkPointFiles = plane_index + 1;
				}
			}
			std::cout << "Loading NI RS checkpoint..." << std::endl;
			if (num_of_checkPointFiles >= 2) {
				//!< Don't use last check point file. It may not completed.
				size_t veh = num_of_checkPointFiles - 2;
				std::stringstream vehicle_name_ss;
				vehicle_name_ss << "_Plane" << veh;
				std::string nth_NI_RS_chkpt_filename = NI_RS_chkpt_filename + vehicle_name_ss.str() + std::string(".mat");
				std::fstream nth_NI_RS_chkpt_fs(nth_NI_RS_chkpt_filename);
				if (!nth_NI_RS_chkpt_fs.fail()) {
					nth_NI_RS_chkpt_fs.close();
					std::cout << "Loading NIRS check point from " << nth_NI_RS_chkpt_filename.c_str() << "..." << std::endl;
					beacls::MatFStream* nirs_chkpt_fs = beacls::openMatFStream(nth_NI_RS_chkpt_filename, beacls::MatOpenMode_Read);
					if (nirs_chkpt_fs) {
						beacls::MatVariable* vehicle_var = beacls::openMatVariable(nirs_chkpt_fs, std::string("Qthis"));
						SPPPlane* vehicle = new SPPPlane(execParameters, nirs_chkpt_fs, vehicle_var);
						beacls::closeMatVariable(vehicle_var);
						nirs->set_Vehicle(vehicle, veh);
						if (veh == num_of_checkPointFiles - 2) {
							beacls::MatVariable* obstacles_var = beacls::openMatVariable(nirs_chkpt_fs, std::string("obstacles"));
							beacls::IntegerVec dummy;
							load_vector_of_vectors(obstacles.data, std::string("data"), dummy, true, nirs_chkpt_fs, obstacles_var);
							load_vector_of_vectors(obstacles.data_s8, std::string("data_s8"), dummy, true, nirs_chkpt_fs, obstacles_var);
							load_vector(obstacles.tau, std::string("tau"), dummy, true, nirs_chkpt_fs, obstacles_var);
							beacls::closeMatVariable(obstacles_var);
							vehStart = veh + 1;
						}
						beacls::closeMatFStream(nirs_chkpt_fs);
						std::cout << "Loaded NIRS check point from " << nth_NI_RS_chkpt_filename.c_str() << ", resume from " << vehStart << "th vehicle." << std::endl;

					}
					else {
						std::cerr << "Error: Cannot read NI_RS checkpoint file. The RTTRS file " << nth_NI_RS_chkpt_filename.c_str() << std::endl;
						return false;
					}
				}
			}
		}
	}
	if (vehStart==0) {
		std::cout << "Initializing vehicles and restarting BR RS computation..." << std::endl;
		nirs->initializeQ(initStates.size());
		//!< File name to save RS data
#if defined(FILESYSTEM)	//!< T.B.D>
		//!< I want to implement making folder after std::filesystem is standardlized in C++17.
		if (checkPointType == CheckPointType_Merged) {
			NI_RS_chkpt_filename = folder + std::string("/") + __func__ + std::string("_chkpt.mat");
		}
		else if (checkPointType == CheckPointType_Separated) {
			NI_RS_chkpt_filename = folder + std::string("/") + __func__ + std::string("_chkpt");
		}
		const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
		if (checkPointType == CheckPointType_Merged) {
			NI_RS_chkpt_filename = folder + std::string("_") + __func__ + std::string("_chkpt.mat");
		}
		else if (checkPointType == CheckPointType_Separated) {
			NI_RS_chkpt_filename = folder + std::string("_") + __func__ + std::string("_chkpt");
		}
		const std::string SPPP_filename = folder + std::string("_SPPP.mat");

#endif
		vehStart = 0;
		beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("SPPP"));
		save(variable_ptr);
		beacls::MatFStream* fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
		beacls::writeMatVariable(fs, variable_ptr);
		beacls::closeMatVariable(variable_ptr);
		beacls::closeMatFStream(fs);
	}

#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	std::string data_folder = folder + std::string("/Plane_data");
#else
	std::string data_folder = folder + std::string("_Plane_data");
#endif

	static const FLOAT_TYPE small = (FLOAT_TYPE)1e-3;

	std::vector<int8_t> augStaticObs_s8;
	if (lowprecision_obstacles) {
		augStaticObs_s8.resize(augStaticObs.size());
		std::transform(augStaticObs.cbegin(), augStaticObs.cend(), augStaticObs_s8.begin(), [](const auto& rhs) {
			return static_cast<int8_t>(std::floor(rhs * obstacles_fix_ratio));
//			return static_cast<int8_t>(std::round(rhs * obstacles_fix_ratio));
		});
	}

	helperOC::ComputeOptTraj* computeOptTraj = new helperOC::ComputeOptTraj();
	//!< Start the computation of reachable sets
	std::vector<std::thread> nirsChkptSave_ths;
	const size_t num_of_vehicles = nirs->get_num_of_vehicles();
	for (size_t vehicle = vehStart; vehicle < num_of_vehicles; ++vehicle) {
		//!< Potential time stamps for current vehicle
		const beacls::FloatVec thisTau = (tTarget.size() == 1) ? tau : generateArithmeticSequence(tTarget[vehicle] - max_BRS_time, dt, tTarget[vehicle]);
		SPPPlane* thisVehicle = initOneRTT(this, rttrs, vehicle, execParameters);
		SPPPlane* lastVehicle = NULL;
		nirs->set_Vehicle(thisVehicle, vehicle);
		//!< Update obstacle
		if (vehicle == 0) {
			obstacles.tau = thisTau;
			if (lowprecision_obstacles) {
				std::vector<int8_t> augStaticObs_s8_3D;
				const size_t reptimes = g->get_N(2);
				for (size_t i = 0; i < reptimes; ++i) {
					augStaticObs_s8_3D.insert(augStaticObs_s8_3D.end(), augStaticObs_s8.cbegin(), augStaticObs_s8.cend());
				}
				obstacles.data_s8.resize(thisTau.size());
				std::fill(obstacles.data_s8.begin(), obstacles.data_s8.end(), augStaticObs_s8_3D);
			}
			else {
				beacls::FloatVec augStaticObs_3D;
				const size_t reptimes = g->get_N(2);
				augStaticObs_3D.reserve(augStaticObs.size()*reptimes);
				for (size_t i = 0; i < reptimes; ++i) {
					augStaticObs_3D.insert(augStaticObs_3D.end(), augStaticObs.cbegin(), augStaticObs.cend());
				}
				obstacles.data.resize(thisTau.size());
				std::fill(obstacles.data.begin(), obstacles.data.end(), augStaticObs_3D);
			}
		}
		else {
			lastVehicle = nirs->get_Vehicle(vehicle - 1);
			std::cout << "Updating obstacles for vehicle " << vehicle << "..." << std::endl;
			size_t erase_begin = obstacles.tau.size();
			for (size_t t = 0; t < obstacles.tau.size(); ++t) {
				if (obstacles.tau[t] > tTarget[vehicle] + small) {
					erase_begin = t;
					break;
				}
			}
			obstacles.tau.erase(obstacles.tau.begin() + erase_begin, obstacles.tau.end());
			if (lowprecision_obstacles) {
				const std::vector<std::vector<int8_t>>& lastVehicleObsForRTT_s8 = lastVehicle->get_obsForRTT_s8();
				if (!lastVehicleObsForRTT_s8.empty()) {
					obstacles.data_s8.erase(obstacles.data_s8.begin() + erase_begin, obstacles.data_s8.end());
					updateObstacles(obstacles, lastVehicle->get_obsForRTT_tau(), lastVehicleObsForRTT_s8, augStaticObs_s8);
				}
			}
			else {
				const std::vector<beacls::FloatVec>& lastVehicleObsForRTT = lastVehicle->get_obsForRTT();
				if (!lastVehicleObsForRTT.empty()) {
					obstacles.data.erase(obstacles.data.begin() + erase_begin, obstacles.data.end());
					updateObstacles(obstacles, lastVehicle->get_obsForRTT_tau(), lastVehicleObsForRTT, augStaticObs);
				}
			}
			std::cout << "Updated obstacles for vehicle " << vehicle << "..." << std::endl;
			std::cout << "Trimming obstacle data and saving checkpoint..." << std::endl;
		}
		if (thisVehicle->get_nomTraj().empty()) {
			//!< Compute the BRS(BRS1) of the vehicle with the above obstacles
			std::cout << "Computing BRS1 for vehicle " << vehicle << std::endl;
			if (lowprecision_obstacles)
				thisVehicle->computeBRS1(thisTau, g, augStaticObs_s8, obstacles, folder, vehicle, low_memory);
			else
				thisVehicle->computeBRS1(thisTau, g, augStaticObs, obstacles, folder, vehicle, low_memory);
			std::cout << "Computed BRS1 for vehicle " << vehicle << std::endl;
			//!< Compute the nominal trajectories based on BRS1
			std::cout << "Computing nominal trajectory for vehicle " << vehicle << std::endl;
			thisVehicle->computeNomTraj(g, folder, vehicle, computeOptTraj);
			std::cout << "Computed nominal trajectory for vehicle " << vehicle << std::endl;
			while (nirs->get_to_save()) {
				std::this_thread::yield();
			}
			if (lastVehicle) {
				while (lastVehicle->get_to_save()) {
					std::this_thread::yield();
				}
			}
			//!< Compute induced obstacles
			std::cout << "Computing obstacles for vehicle " << vehicle << std::endl;
			thisVehicle->computeObsForRTT(this, rttrs);
			std::cout << "Computed obstacles for vehicle " << vehicle << std::endl;
		}
		else {
			while (nirs->get_to_save()) {
				std::this_thread::yield();
			}
			if (lastVehicle) {
				while (lastVehicle->get_to_save()) {
					std::this_thread::yield();
				}
			}
		}
		if ((checkPointType == CheckPointType_Merged) || (checkPointType == CheckPointType_Separated)) {
			nirs->set_to_save(true);
			thisVehicle->set_to_save(true);
			const size_t stride = (save_brs1_file || (vehicle == vehStart)) ? 0 : 2;
			SeqPP::Obstacles* obstacles_to_save_ptr = new SeqPP::Obstacles(obstacles);
			nirsChkptSave_ths.push_back(std::thread(&SeqPP::saveNI_RS_chkpt_and_clear,
				matWriter, NI_RS_chkpt_filename, nirs, obstacles_to_save_ptr, vehicle, checkPointType, stride, save_brs1_file, true));
		}
		if (num_of_vehicles_to_computeNIRS != 0 && (vehicle != (num_of_vehicles - 1))) {
			if (((vehStart == 0) && ((vehicle + 1) >= (vehStart + num_of_vehicles_to_computeNIRS))) ||
				((vehStart != 0) && ((vehicle + 1) > (vehStart + num_of_vehicles_to_computeNIRS)))
				) {
				//!< join threads
				std::for_each(nirsChkptSave_ths.begin(), nirsChkptSave_ths.end(), [](auto& rhs) { rhs.join(); });
				nirsChkptSave_ths.clear();
				std::cout << "Computing NIRS suspended at " << vehicle << std::endl;
				if (computeOptTraj) delete computeOptTraj;
				return false;
			}
		}
	}
	while (nirs->get_to_save()) {
		std::this_thread::yield();
	}
	//!< join threads
	std::for_each(nirsChkptSave_ths.begin(), nirsChkptSave_ths.end(), [](auto& rhs) { rhs.join(); });
	nirsChkptSave_ths.clear();
	//!< Remove obstacles.
	obstacles = SeqPP::Obstacles();

	for (size_t vehicle_index = 0; vehicle_index < num_of_vehicles; ++vehicle_index) {
		SPPPlane* thisVehicle = nirs->get_Vehicle(vehicle_index);
		if (thisVehicle) {
			while (thisVehicle->get_to_save()) {
				std::this_thread::yield();
			}
			thisVehicle->clear_BRS1();
			thisVehicle->clear_obsForRTT();
			thisVehicle->clear_obsForRTT_s8();
		}
		else if (checkPointType == CheckPointType_Separated) {
			std::stringstream vehicle_name_ss;
			vehicle_name_ss << "_Plane" << vehicle_index;
			std::string nth_NI_RS_chkpt_filename = NI_RS_chkpt_filename + vehicle_name_ss.str() + std::string(".mat");
			std::cout << "Loading NIRS check point from " << nth_NI_RS_chkpt_filename.c_str() << std::endl;

			beacls::MatFStream* fs = beacls::openMatFStream(nth_NI_RS_chkpt_filename, beacls::MatOpenMode_Read);
			if (fs) {
				beacls::MatVariable* variable_ptr = beacls::openMatVariable(fs, std::string("Qthis"));
				if (variable_ptr) {
					SPPPlane* restoredVehicle = new SPPPlane(execParameters, fs, variable_ptr);
					restoredVehicle->clear_BRS1();
					restoredVehicle->clear_obsForRTT();
					restoredVehicle->clear_obsForRTT_s8();
					if (restoredVehicle) {
						nirs->set_Vehicle(restoredVehicle, vehicle_index);
					}
					beacls::closeMatVariable(variable_ptr);
				}
				beacls::closeMatFStream(fs);
			}
		}
	}

	{
#if defined(FILESYSTEM)	//!< T.B.D>
		//!< I want to implement making folder after std::filesystem is standardlized in C++17.
		NI_RS_filename = folder + std::string("/") + __func__ + std::string(".mat");
		const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
		NI_RS_filename = folder + std::string("_") + __func__ + std::string(".mat");
		const std::string SPPP_filename = folder + std::string("_SPPP.mat");
#endif
		std::cout << "Saving NIRS check point to " << NI_RS_filename.c_str() << std::endl;
		beacls::MatFStream* nirs_fs = beacls::openMatFStream(NI_RS_filename, beacls::MatOpenMode_Write);
		bool result = true;
		//!< Save vehicles to NI_RS file and clear exept last 2.
		result &= nirs->save(nirs_fs, true, 2);
		beacls::closeMatFStream(nirs_fs);
		std::cout << "Saved NIRS check point to " << NI_RS_filename.c_str() << std::endl;

		beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("SPPP"));
		save(variable_ptr);
		beacls::MatFStream* fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
		beacls::writeMatVariable(fs, variable_ptr);
		beacls::closeMatVariable(variable_ptr);
		beacls::closeMatFStream(fs);
	}
	//!< Save last 2 vehicles without obsForRTT.
	if (checkPointType == CheckPointType_Separated) {
		if (!save_brs1_file) {
			Obstacles* dummy_obstacles_ptr = new SeqPP::Obstacles();
			SeqPP::saveNI_RS_chkpt_and_clear(matWriter, NI_RS_chkpt_filename, nirs, dummy_obstacles_ptr, num_of_vehicles - 2, checkPointType, 0, save_brs1_file, false);
			SeqPP::saveNI_RS_chkpt_and_clear(matWriter, NI_RS_chkpt_filename, nirs, dummy_obstacles_ptr, num_of_vehicles - 1, checkPointType, 0, save_brs1_file, false);
		}
	}
	if (computeOptTraj) delete computeOptTraj;
	return true;
}

bool SeqPP::SPPProblem::simulateNI(
	const bool save_png,
	const bool save_fig,
	const std::string& this_NI_RS_filename
) {
	//!< Load files
	//!< Load robust tracking reachable set
	if (rttrs) delete rttrs;
	rttrs = new RTTRS();
	std::cout << "Loading RTTRS..." << std::endl;
	beacls::MatFStream* rttrs_fs = beacls::openMatFStream(RTTRS_filename, beacls::MatOpenMode_Read);
	beacls::MatVariable* rttrs_var = NULL;
	bool rttrs_load = false;
	if (rttrs_fs) {
		rttrs_var = beacls::openMatVariable(rttrs_fs, std::string("RTTRS"));
		if (rttrs_var) {
			rttrs_load = rttrs->load(rttrs_var);
			std::cout << "The RTTRS file " << RTTRS_filename.c_str() << " already exists. Skipping RTTRS computation." << std::endl;
			beacls::closeMatVariable(rttrs_var);
		}
		beacls::closeMatFStream(rttrs_fs);
	}
	if (!rttrs_load) {
		std::cerr << "RTTRS file " << RTTRS_filename.c_str() << " not found!" << std::endl;
		return false;
	}

	const std::string& modified_NI_RS_filename = this_NI_RS_filename.empty() ? NI_RS_filename : this_NI_RS_filename;

	//!< Load path planning reachable set
	std::cout << "Loading RS data..." << std::endl;
	if (nirs) delete nirs;
	nirs = new NIRS();
	beacls::MatFStream* nirs_fs = beacls::openMatFStream(modified_NI_RS_filename, beacls::MatOpenMode_Read);
	bool nirs_load = false;
	if (nirs_fs) {
		nirs_load = nirs->load(execParameters, nirs_fs);
		beacls::closeMatFStream(nirs_fs);
	}
	if (!nirs_load) {
		std::cerr << "RS file not found!" << std::endl;
		return false;
	}

	std::vector<SPPPlane*> Q = nirs->get_Q();

	//!< Post process loaded data
	//!< Compute gradients used for optimal control
	std::cout << "Computing gradients..." << std::endl;
	std::vector<beacls::FloatVec> derivC, derivL, derivR;
	helperOC::ComputeGradients* computeGradients = new helperOC::ComputeGradients(rttrs->get_g());
	const beacls::FloatVec& data = rttrs->get_data();
	computeGradients->operator()(derivC, derivL, derivR, rttrs->get_g(), data, data.size(), false, execParameters);
	if (computeGradients) delete computeGradients;
	rttrs->set_Deriv(derivC);

	//!< Determine time of simulation
	FLOAT_TYPE tStart = std::numeric_limits<FLOAT_TYPE>::infinity();
	FLOAT_TYPE tEnd = -std::numeric_limits<FLOAT_TYPE>::infinity();

	std::for_each(Q.begin(), Q.end(), [&tStart, &tEnd](auto& rhs) {
		std::vector<beacls::FloatVec> xhist = rhs->get_xhist();
		if (!xhist.empty()) {
			beacls::FloatVec x = xhist[0];
			rhs->set_x(x);
			rhs->clear_xhist();
			rhs->push_back_xhist(x);
		}
		rhs->clear_u();
		rhs->clear_uhist();
		const beacls::FloatVec& nomTraj_tau = rhs->get_nomTraj_tau();
		if (!nomTraj_tau.empty()) {
			auto minmax_ites = beacls::minmax_value<FLOAT_TYPE>(nomTraj_tau.cbegin(), nomTraj_tau.cend());
			tStart = std::min<FLOAT_TYPE>(tStart, minmax_ites.first);
			tEnd =std::max<FLOAT_TYPE>(tEnd, minmax_ites.second);
		}
	});
	beacls::FloatVec this_tau = generateArithmeticSequence(tStart, dt, tEnd);

	//!< Add cylindrical obstacles for visualization
	if (save_png || save_fig) {
	}

	const FLOAT_TYPE small = static_cast<FLOAT_TYPE>(1e-4);

	beacls::IntegerVec tInds(Q.size());
	beacls::FloatVec taumin;
	taumin.resize(Q.size(), std::numeric_limits<FLOAT_TYPE>::infinity());
	beacls::FloatVec taumax;
	taumax.resize(Q.size(), -std::numeric_limits<FLOAT_TYPE>::infinity());

	const size_t subSamples = 32;

	for (size_t i = 0; i < this_tau.size(); ++i) {
		std::cout << "t = " << this_tau[i] << std::endl;

		for (size_t vehicle = 0; vehicle < Q.size(); ++vehicle) {
			SPPPlane* thisVehicle = Q[vehicle];
			const beacls::FloatVec& nomTraj_tau = thisVehicle->get_nomTraj_tau();
			const std::vector<beacls::FloatVec>& nomTraj = thisVehicle->get_nomTraj();
			std::vector<beacls::FloatVec> trimed_nomTraj;
			trimed_nomTraj.reserve(nomTraj.size());
			//!< Check if nominal trajectory has this t
			tInds[vehicle] = nomTraj_tau.size();
			for (size_t t=0;t<nomTraj_tau.size();++t){
				if ((nomTraj_tau[t] > (this_tau[i] - small)) &&
					(nomTraj_tau[t] < (this_tau[i] + small))
					) {
					tInds[vehicle] = t;
					break;
				}			
			};
			if (tInds[vehicle] < nomTraj_tau.size()) {
				taumin[vehicle] = std::min<FLOAT_TYPE>(taumin[vehicle], this_tau[i]);
				taumax[vehicle] = std::max<FLOAT_TYPE>(taumax[vehicle], this_tau[i]);
				//!< Get optimal control
				//!< Our plane is vehicle A, trying to stay out of reachable set, and the
				//!< reference virtual plane is vehicle B, trying to get into reachable set
				const beacls::FloatVec& x = thisVehicle->get_x();
				for (size_t s = 0; s < subSamples; ++s) {
					//!< Obtain intermediate nominal trajectory points
					const size_t this_tInd = tInds[vehicle];
					const size_t prev_tInd = (this_tInd == 0) ? 0 : this_tInd - 1;
					const FLOAT_TYPE w = (FLOAT_TYPE)s / subSamples;
					const beacls::FloatVec& prev_nomTraj = nomTraj[prev_tInd];
					const beacls::FloatVec& this_nomTraj = nomTraj[this_tInd];
					beacls::FloatVec nomTraj_pt(prev_nomTraj.size());
					std::transform(prev_nomTraj.cbegin(), prev_nomTraj.cend(), this_nomTraj.cbegin(), nomTraj_pt.begin(), [w](const auto& lhs, const auto& rhs) {
						return (1 - w) * lhs + w * rhs;
					});
					beacls::FloatVec rel_x(nomTraj_pt.size());
					std::transform(nomTraj_pt.cbegin(), nomTraj_pt.cend(), x.cbegin(), rel_x.begin(), std::minus<FLOAT_TYPE>());
					
					beacls::FloatVec rel_x_0_1{ rel_x[0], rel_x[1] };
					helperOC::rotate2D(rel_x_0_1, rel_x_0_1, -x[2]);
					std::copy(rel_x_0_1.cbegin(), rel_x_0_1.cend(), rel_x.begin());

					std::vector<beacls::FloatVec> deriv;
					helperOC::eval_u(deriv, rttrs->get_g(), rttrs->get_Deriv(), rel_x);

					std::vector<const FLOAT_TYPE*> deriv_ptrs(deriv.size());
					beacls::IntegerVec deriv_sizes(deriv.size());
					std::transform(deriv.cbegin(), deriv.cend(), deriv_ptrs.begin(), [](const auto& rhs) { return rhs.data(); });
					std::transform(deriv.cbegin(), deriv.cend(), deriv_sizes.begin(), [](const auto& rhs) { return rhs.size(); });


					std::vector<beacls::FloatVec> rel_xs(rel_x.size());
					std::transform(rel_x.cbegin(), rel_x.cend(), rel_xs.begin(), [](const auto& rhs) {
						return beacls::FloatVec{rhs};
					});
					std::vector<beacls::FloatVec::const_iterator> rel_x_ites(rel_x.size());
					beacls::IntegerVec rel_x_sizes(rel_x.size());
					for (size_t dimension = 0; dimension < rel_xs.size(); ++dimension) {
						rel_x_ites[dimension] = rel_xs[dimension].cbegin();
						rel_x_sizes[dimension] = rel_xs[dimension].size();
					}
					std::vector<beacls::FloatVec> us(rttrs->get_dynSys()->get_nu());
					std::for_each(us.begin(), us.end(), [](auto& rhs) { rhs.resize(1); });
					rttrs->get_dynSys()->optCtrl(us, 0, rel_x_ites, deriv_ptrs, rel_x_sizes, deriv_sizes, DynSys_UMode_Max);

					//!< Get disturbance
					std::vector<beacls::FloatVec> ds;
					thisVehicle->uniformDstb(ds);

					//!< Update state
					beacls::FloatVec x1;
					thisVehicle->updateState(x1, us, dt / subSamples, x, ds);
				}
				//!< Remove sub-samples
				std::vector<std::vector<beacls::FloatVec>> uhist = thisVehicle->get_uhist();
				std::vector<beacls::FloatVec> xhist = thisVehicle->get_xhist();
				uhist.resize(uhist.size() - (subSamples - 1));
				xhist.resize(xhist.size() - (subSamples - 1));
				thisVehicle->set_uhist(uhist);
				thisVehicle->set_xhist(xhist);
			}

			//!< Visualize
			if (save_png || save_fig) {

			}
			if (save_png) {
			}
			if (save_fig) {
			}
		}

	}
	//!< Save data
	tau = this_tau;
	for (size_t vehicle = 0; vehicle < Q.size(); ++vehicle) {
		Q[vehicle]->set_tau(generateArithmeticSequence(taumin[vehicle], dt, taumax[vehicle]));
	}
#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	NI_sim_filename = folder + std::string("/") + __func__ + std::string(".mat");
#else
	NI_sim_filename = folder + std::string("_") + __func__ + std::string(".mat");
#endif
	

	beacls::MatFStream* ni_sim_fs = beacls::openMatFStream(NI_sim_filename, beacls::MatOpenMode_Write);
	for (size_t i = 0; i < std::min<size_t>(Q.size(), 4); ++i) {
		std::stringstream ss;
		ss << "Q" << i;
		std::string vehicle_name = ss.str();
		beacls::MatVariable* variable_ptr = beacls::createMatStruct(vehicle_name);
		Q[i]->save(ni_sim_fs, variable_ptr);
		beacls::writeMatVariable(ni_sim_fs, variable_ptr);
		beacls::closeMatVariable(variable_ptr);
	}
	beacls::closeMatFStream(ni_sim_fs);
#if defined(FILESYSTEM)	//!< T.B.D>
	//!< I want to implement making folder after std::filesystem is standardlized in C++17.
	const std::string SPPP_filename = folder + std::string("/SPPP.mat");
#else
	const std::string SPPP_filename = folder + std::string("_SPPP.mat");
#endif

	beacls::MatVariable* variable_ptr = beacls::createMatStruct(std::string("SPPP"));
	save(variable_ptr);
	beacls::MatFStream* fs = beacls::openMatFStream(SPPP_filename, beacls::MatOpenMode_Write);
	beacls::writeMatVariable(fs, variable_ptr);
	beacls::closeMatVariable(variable_ptr);
	beacls::closeMatFStream(fs);
	return true;
}
