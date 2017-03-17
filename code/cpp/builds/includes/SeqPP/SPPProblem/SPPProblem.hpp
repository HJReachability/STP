#ifndef __SPPProblem_hpp__
#define __SPPProblem_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <iostream>
#include <cstring>
#include <typedef.hpp>
#include <SeqPP/SPPProblem/SPPProblemTypedef.hpp>
#include <helperOC/helperOC_type.hpp>
class HJI_Grid;
class Plane;
namespace SeqPP {
	class RTTRS;
	class NIRS;
	class CARS;
	class VehicleParameters;
	class GridParameters;
	class MigrateRTTRS;
	class MatWriter;
	class SPPProblem{
	public:

	private:
		//!< Problem parameters
		std::vector<beacls::FloatVec > initStates;
		std::vector<beacls::FloatVec > targetCenters;
		
		FLOAT_TYPE targetR;
		std::vector<beacls::FloatVec > targetRsmall;

			//	intrIS
			//	intrCtrl
		FLOAT_TYPE tIAT;

		//!< Intruder method 2
		FLOAT_TYPE max_num_affected_vehicles;
		FLOAT_TYPE buffer_duration;
		size_t buffer_duration_ind;
		size_t remaining_duration_ind;

		FLOAT_TYPE Rc; //!< collision radius
		
		//!< static obstacles
		std::string mapFile;
		beacls::FloatVec staticObs;
		beacls::FloatVec augStaticObs;

		//!< SPP vehicle parameters
		beacls::FloatVec vRangeA;
		FLOAT_TYPE wMaxA;
		beacls::FloatVec dMaxA;

		//!< Robust trajectory tracking
		beacls::FloatVec vReserved;
		FLOAT_TYPE wReserved;
		FLOAT_TYPE RTT_tR;
		
		//!< Time
		FLOAT_TYPE tMin;	//!< Minimum time for the entire problem
		FLOAT_TYPE dt;	//!< Discretization
		beacls::FloatVec tTarget;  //!< initial target time
		FLOAT_TYPE tIntr;		//!< Intruder appearance time
		beacls::FloatVec tReplan;	  //!< time at which replanning is done (intruder disappearance)
		FLOAT_TYPE max_BRS_time;
		
		FLOAT_TYPE rttrs_tMax;	//!< Maximum time for the computeRTTRS
		beacls::IntegerVec rttrs_gN;	//!< Grid space for the computeRTTRS

		//!< Space
		beacls::FloatVec gMin;
		beacls::FloatVec gMax;
		beacls::IntegerVec gN;
		HJI_Grid* g;
		HJI_Grid* g2D;
		
		beacls::FloatVec tauBR;		//!< Time vector before replanning
		beacls::FloatVec tauAR;		//!< Time vector after replanning
		beacls::FloatVec tauSim;	   //!< Time vector for entire simulation
		beacls::FloatVec tau;		  //!< Global time (absolute time vector)
		
		RTTRS* rttrs;
		NIRS* nirs;
		CARS* cars;

		//!< File to store this SPPProblem instance
		std::string folder;
		
		//!< Files to load
		std::string RTTRS_filename; //!< robust trajectory tracking reachable set
		std::string CARS_filename;  //!< collision avoidance reachable set

		std::string minMinBRS_filename;	//!< minMinBRS file(for computing buffer region)
		std::string bufferRegion_filename;	//!< Buffer region
		std::string FRSBRS_filename;	//!< FRSs and BRSs of FRSs
		std::string rawAugObs_filename;	//!< raw augmented obstacles file nam

		std::string NI_RS_chkpt_filename; //!< no intruder reachable sets
		std::string NI_RS_filename;
		std::string NI_sim_filename;
		
		std::string BR_RS_filename; //!< before replanning reachable sets
		std::string BR_RS_chkpt_filename;
		std::string AR_RS_filename; //!< after replanning reachable sets
		std::string AR_RS_chkpt_filename;
		
		std::string BR_sim_filename; //!< before replanning simulation file (simulation results)
		std::string full_sim_filename; //!< full simulation file (simulation results)

		bool keepLast;	//!< retain all time dependent values
		bool lowprecision_obstacles;
		helperOC::ExecParameters execParameters;
		MigrateRTTRS* migrateRTTRS;
		MatWriter* matWriter;

	public:
		PREFIX_VC_DLL
		SPPProblem(
			const std::string& SPPP_filename,
			const std::string& RTTRS_filename = std::string(),
			const std::string& NI_RS_filename = std::string(),
			const std::string& NI_RS_chkpt_filename = std::string(),
			const bool lowprecision_obstacles = false,
			const helperOC::ExecParameters& execParameters = helperOC::ExecParameters()
		);
		PREFIX_VC_DLL
		SPPProblem(
			const ProblemType problem_name,
			const std::string& RTTRS_filename,
			const std::string& NI_RS_filename = std::string(),
			const std::string& NI_RS_chkpt_filename = std::string(),
			const bool lowprecision_obstacles = false,
			const helperOC::ExecParameters& execParameters = helperOC::ExecParameters(),
			const SetupExtraArgs = SetupExtraArgs(),
			const beacls::IntegerVec& rttrs_gN = beacls::IntegerVec()
		);
		PREFIX_VC_DLL
			~SPPProblem();
		bool loadSetup(const Setup_Type setup_name, const SetupExtraArgs& extraArgs);
		bool gen_targets_initStates(
			std::vector<beacls::FloatVec>& initStates,
			std::vector<beacls::FloatVec>& tarCenters,
			beacls::IntegerVec& tarCount,
			const Setup_Type setup_name,
			const size_t numVeh,
			const bool randomSeedForTargets
			);
		void set_Rc(const FLOAT_TYPE v) { Rc = v; }
		void set_tMin(const FLOAT_TYPE v) { tMin = v; }
		void set_dt(const FLOAT_TYPE v) { dt = v; }
		void set_tau(const beacls::FloatVec& v) { tau = v; }
		void set_staticObs(const beacls::FloatVec& v) { staticObs = v; }

		const std::vector<beacls::FloatVec >& get_initStates() const { return initStates; };
		const std::vector<beacls::FloatVec >& get_targetCenters() const { return targetCenters; };
		const beacls::FloatVec& get_initState(const size_t i) const { return initStates[i]; };
		const beacls::FloatVec& get_targetCenter(const size_t i) const { return targetCenters[i]; };

		FLOAT_TYPE get_targetR() const { return targetR; };
		FLOAT_TYPE get_Rc() const { return Rc; }
		FLOAT_TYPE get_tMin() const { return tMin; }
		FLOAT_TYPE get_dt() const { return dt; }
		const beacls::FloatVec& get_tTarget() const { return tTarget; }
		const beacls::FloatVec& get_tReplan() const { return tReplan; }
		HJI_Grid* get_g() const { return g; }
		HJI_Grid* get_g2D() const { return g2D; }
		MigrateRTTRS* get_MigrateRTTRS() const { return migrateRTTRS; }
		bool get_s8ary_obstacles() const { return lowprecision_obstacles; }
		/**
			@brief	Computes the robust trajectory tracking reachable set and updates the SPPP
					object with the RTTRS file name.
					SPP problem object updated with RTTRS file name.
			@param	[in]	save_png		set to true to save figures
		*/
		bool computeRTTRS(
			const bool save_png = true
		);
		/**
		@brief	Computes collision avoidance reachable set and updates the SPPP object
				with the CARS file name
		@param	[in]	Qintr		intruder vehicle object (just for extracting vehicle parameters)
		@param	[in]	tIAT		intruder avoidance time
		@param	[in]	save_png	set to true to save computation figures
		*/
		bool computeCARS(
			const Plane* qintr = NULL,
			const bool save_png = true,
			const bool restart = false
		);
		/**
			@brief	Computes the before-replanning reachable sets for the SPP problem
			@param	[in]	restart		set to true to restart and overwrite computation
		*/
		bool computeNIRS(
			const bool restart = false,
			const bool low_memory = false,
			const CheckPointType checkPointType = CheckPointType_Merged,
			const bool save_brs1_file = false,
			const size_t num_of_vehicles_to_computeNIRS = 0
		);
		/**
		@brief	Simulates SPP with disturbances with the RTT method
		@param	[in]	save_png	set to true to save figures
		@param	[in]	save_fig	set to true to save figures
		*/
		bool simulateNI(
			const bool save_png = true,
			const bool save_fig = false,
			const std::string& this_NI_RS_filename = std::string()
			);

		bool save(
			beacls::MatVariable* variable_ptr
		);
	private:
		/** @overload
		Disable operator=
		*/
		SPPProblem& operator=(const SPPProblem& rhs);
		/** @overload
		Disable copy constructor
		*/
		SPPProblem(const SPPProblem& rhs);
	};
};
#endif	/* __SPPProblem_hpp__ */
