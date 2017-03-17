#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <numeric>
#include <typedef.hpp>
#include <SeqPP/SPPwDisturbanceRTT.hpp>
#include <SeqPP/SPPProblem/SPPProblem.hpp>
#include <macro.hpp>

void SeqPP::SPPwDisturbanceRTT(
	const SeqPP::ProblemType problem_name,
	const SeqPP::SetupExtraArgs& setupExtraArgs,
	SeqPP::SPPProblem* SeqPPP,
	const bool lowprecision_obstacles,
	const bool low_memory,
	const helperOC::ExecParameters& execParameters,
	const bool restart,
	const CheckPointType checkPointType,
	const bool save_brs1_file,
	const size_t num_of_vehicles_to_computeNIRS,
	const beacls::IntegerVec& rttrs_gN
) {
	SeqPP::SPPProblem* newSeqPPP = NULL;
	if (!SeqPPP) {

		newSeqPPP = new SeqPP::SPPProblem(problem_name, std::string(), std::string(), std::string(), lowprecision_obstacles, execParameters, setupExtraArgs, rttrs_gN);
		SeqPPP = newSeqPPP;
	}

	SeqPPP->computeRTTRS();
	bool finishAll = SeqPPP->computeNIRS(restart, low_memory, checkPointType, save_brs1_file, num_of_vehicles_to_computeNIRS);
	if (finishAll) {
		SeqPPP->simulateNI();
	}

	if (newSeqPPP) delete newSeqPPP;
}