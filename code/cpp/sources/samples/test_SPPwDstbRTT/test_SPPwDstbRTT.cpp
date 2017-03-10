#include <SeqPP/SPPwDisturbanceRTT.hpp>
#include <SeqPP/SPPProblem/SPPProblem.hpp>
#include <typedef.hpp>
#include <cstdlib>
#include <sstream>
/*
	@brief Solves the entire SPP with disturbances problem using the RTT method
*/
int main(int argc, char *argv[])
{
	SeqPP::ProblemType problem_type = SeqPP::ProblemType_SF_dstb;
	if (argc >= 2) {
		problem_type = static_cast<SeqPP::ProblemType>(atoi(argv[1]));
	}
	FLOAT_TYPE wind_speed = (FLOAT_TYPE)11;
	if (argc >= 3) {
		wind_speed = (FLOAT_TYPE)atof(argv[2]);
	}
	FLOAT_TYPE separation_time = (FLOAT_TYPE)4;
	if (argc >= 4) {
		separation_time = (FLOAT_TYPE)atof(argv[3]);
	}
	bool restart = false;
	if (argc >= 5) {
		restart = (atoi(argv[4]) == 0) ? false : true;
	}
	size_t num_of_vehicles_to_computeNIRS = 0;
	if (argc >= 6) {
		num_of_vehicles_to_computeNIRS = atoi(argv[5]);
	}
	SeqPP::CheckPointType checkPointType = SeqPP::CheckPointType_Merged;
	if (argc >= 7) {
		checkPointType = static_cast<SeqPP::CheckPointType>(atoi(argv[6]));
	}
	bool use_intermediate_file = false;
	if (argc >= 8) {
		use_intermediate_file = (atoi(argv[7]) == 0) ? false : true;
	}
	bool save_brs1_file = false;
	if (argc >= 9) {
		save_brs1_file = (atoi(argv[8]) == 0) ? false : true;
	}
	bool lowprecision_obstacles = false;
	if (argc >= 10) {
		lowprecision_obstacles = (atoi(argv[9]) == 0) ? false : true;
	}
	bool randomSeedForTargets = false;
	if (argc >= 11) {
		randomSeedForTargets = (atoi(argv[10]) == 0) ? false : true;
	}
	bool low_memory = false;
	if (argc >= 12) {
		low_memory = (atoi(argv[11]) == 0) ? false : true;
	}
	beacls::DelayedDerivMinMax_Type delayedDerivMinMax = beacls::DelayedDerivMinMax_Disable;
	if (argc >= 13) {
		switch (atoi(argv[12])) {
		default:
		case 0:
			delayedDerivMinMax = beacls::DelayedDerivMinMax_Disable;
			break;
		case 1:
			delayedDerivMinMax = beacls::DelayedDerivMinMax_Always;
			break;
		case 2:
			delayedDerivMinMax = beacls::DelayedDerivMinMax_Adaptive;
			break;
		}
	}
	bool useCuda = false;
	if (argc >= 14) {
		useCuda = (atoi(argv[13]) == 0) ? false : true;
	}
	int num_of_threads = 0;
	if (argc >= 15) {
		num_of_threads = atoi(argv[14]);
	}
	int num_of_gpus = 0;
	if (argc >= 16) {
		num_of_gpus = atoi(argv[15]);
	}
	size_t line_length_of_chunk = 0;
	if (argc >= 17) {
		line_length_of_chunk = atoi(argv[16]);
	}
	bool enable_user_defined_dynamics_on_gpu = true;
	if (argc >= 18) {
		enable_user_defined_dynamics_on_gpu = (atoi(argv[17]) == 0) ? false : true;
	}
	std::string filenames_prefix;
	if (argc >= 19) {
		filenames_prefix = std::string(argv[18]);
	}

	SeqPP::SetupExtraArgs setupExtraArgs;
	setupExtraArgs.wind_speed = wind_speed;
	setupExtraArgs.separation_time = separation_time;
	setupExtraArgs.randomSeedForTargets = randomSeedForTargets;

	helperOC::ExecParameters execParameters;
	execParameters.line_length_of_chunk = line_length_of_chunk;
	execParameters.useCuda = useCuda;
	execParameters.num_of_gpus = num_of_gpus;
	execParameters.num_of_threads = num_of_threads;
	execParameters.delayedDerivMinMax = delayedDerivMinMax;
	execParameters.enable_user_defined_dynamics_on_gpu = enable_user_defined_dynamics_on_gpu;

	SeqPP::SPPProblem* seqPPP = NULL;
#if defined(WIN32) && !defined(FILESYSTEM)	// Windows
	std::string input_folder(".\\\\inputs\\\\");
#else	//!< Unix
	std::string input_folder("./inputs/");
#endif
	if (use_intermediate_file) {
		const std::string low_memory_str = low_memory ? std::string("_wLowMem") : std::string("_woLowMem");
		if (filenames_prefix.empty()) {
			std::string problem_str;
			switch (problem_type) {
			case SeqPP::ProblemType_default:
			case SeqPP::ProblemType_SF_dstb:
				problem_str = std::string("SF_dstb");
				break;
			case SeqPP::ProblemType_SF_intr_2:
				problem_str = std::string("SF_intr_2");
				break;
			case SeqPP::ProblemType_SF_intr_3:
				problem_str = std::string("SF_intr_3");
				break;
			case SeqPP::ProblemType_SF_intr_4:
				problem_str = std::string("SF_intr_4");
				break;
			case SeqPP::ProblemType_TCST_dstb:
				problem_str = std::string("TCST_dstb");
				break;
			case SeqPP::ProblemType_TCST_intr:
				problem_str = std::string("TCST_intr_11");
				break;
			case SeqPP::ProblemType_Bay_Area:
				problem_str = std::string("BA");
				break;
			case SeqPP::ProblemType_SF_dstb_5veh:
				problem_str = std::string("SF_dstb_5veh");
				break;
			case SeqPP::ProblemType_SF_dstb_10veh:
				problem_str = std::string("SF_dstb_10veh");
				break;
			case SeqPP::ProblemType_SF_dstb_20veh:
				problem_str = std::string("SF_dstb_20veh");
				break;
			case SeqPP::ProblemType_SF_dstb_30veh:
				problem_str = std::string("SF_dstb_30veh");
				break;
			case SeqPP::ProblemType_SF_dstb_40veh:
				problem_str = std::string("SF_dstb_40veh");
				break;
			case SeqPP::ProblemType_SF_dstb_hell:
				problem_str = std::string("SF_dstb_hell");
				break;
			case SeqPP::ProblemType_Invalid:
				break;
			}
			std::stringstream wind_ss;
			wind_ss << "_" << wind_speed;
			std::stringstream separation_time_ss;
			separation_time_ss << "_" << separation_time << "sSep";

			filenames_prefix = input_folder + problem_str + wind_ss.str() + separation_time_ss.str() + low_memory_str;
		}
#if defined(FILESYSTEM)
		const std::string path_separation = "/";
#else
		const std::string path_separation = "_";
#endif
		const std::string SPPP_filename = filenames_prefix + path_separation + std::string("SPPP.mat");
		const std::string RTTRS_filename = filenames_prefix + path_separation + std::string("RTTRS.mat");
		const std::string NI_RS_filename = filenames_prefix + path_separation + std::string("computeNIRS.mat");
		const std::string NI_RS_chkpt_filename_postfix = path_separation + ((checkPointType == SeqPP::CheckPointType_Separated) ? std::string("computeNIRS_chkpt") : std::string("computeNIRS_chkpt.mat"));
		const std::string NI_RS_chkpt_filename = filenames_prefix + NI_RS_chkpt_filename_postfix;

		seqPPP = new SeqPP::SPPProblem(SPPP_filename, RTTRS_filename, NI_RS_filename, NI_RS_chkpt_filename, lowprecision_obstacles, execParameters);
	}
	SeqPP::SPPwDisturbanceRTT(
		problem_type,
		setupExtraArgs,
		seqPPP,
		lowprecision_obstacles,
		low_memory,
		execParameters,
		restart,
		checkPointType,
		save_brs1_file,
		num_of_vehicles_to_computeNIRS
	);
	if (seqPPP) delete seqPPP;
	return 0;
}

