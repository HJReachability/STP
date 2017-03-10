#ifndef __SPPPlane_hpp__
#define __SPPPlane_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <vector>
#include <utility>
using namespace std::rel_ops;
#include <typedef.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/DynSys/Plane/Plane.hpp>
#include <SeqPP/SPPProblem/SPPProblemTypedef.hpp>
namespace helperOC {
class ComputeOptTraj;
}
namespace SeqPP {
	class SPPProblem;
	class RTTRS;
	class CARS;
	class RawAugObs;
	class MigrateRTTRS;

	/**
		@brief Specialized Plane class with SPP-related properties for the SPP project
	*/
	class SPPPlane : public Plane{
	public:
	private:
		//!< If tracking 3D trajectory
		beacls::FloatVec vReserved;
		FLOAT_TYPE wReserved;

		FLOAT_TYPE v; //!< If tracking 2D trajectory

		std::vector<beacls::FloatVec> BRS1;
		beacls::FloatVec BRS1_tau;

		std::vector<beacls::FloatVec> nomTraj;
		beacls::FloatVec nomTraj_tau;

		std::vector<beacls::FloatVec> nomTraj_AR; //!< Nominal trajectory after replanning
		beacls::FloatVec nomTraj_AR_tau;

		std::vector<beacls::FloatVec> obsForRTT;
		std::vector<std::vector<int8_t>> obsForRTT_s8;
		beacls::FloatVec obsForRTT_tau;

		std::vector<beacls::FloatVec> obsForIntr;
		beacls::FloatVec obsForIntr_tau;

		std::vector<beacls::FloatVec> FRS1;
		beacls::FloatVec FRS1_tau;
		FLOAT_TYPE FRS1_g;

		std::vector<beacls::FloatVec> obs2D;
		beacls::FloatVec obs2D_tau;

		beacls::FloatVec target;
		beacls::FloatVec targetsm;
		beacls::FloatVec targetCenter;
		FLOAT_TYPE targetR;
		FLOAT_TYPE targetRsmall;

		//!< Replanning
		bool replan; //!< Whether replan is done
		beacls::FloatVec tauBR;
		beacls::FloatVec tauAR;
		beacls::FloatVec tau;
		helperOC::ExecParameters execParameters;
		bool to_save;
	public:
		PREFIX_VC_DLL
			SPPPlane(
			const beacls::FloatVec& x,
			const FLOAT_TYPE wMax,
			const beacls::FloatVec& vrange,
			const beacls::FloatVec& dMax,
			const helperOC::ExecParameters& execParameters = helperOC::ExecParameters()
		);
		SPPPlane(
			const helperOC::ExecParameters& execParameters = helperOC::ExecParameters(),
			beacls::MatFStream* fs = NULL,
			beacls::MatVariable* variable_ptr = NULL
		);
		PREFIX_VC_DLL
			~SPPPlane();
		PREFIX_VC_DLL
			virtual bool operator==(const SPPPlane& rhs) const;
		PREFIX_VC_DLL
			virtual bool operator==(const DynSys& rhs) const;

		SPPPlane* clone() const {
			return new SPPPlane(*this);
		}

		bool get_to_save() const;
		void set_to_save(const bool v);
		bool save(
			beacls::MatFStream* fs = NULL,
			beacls::MatVariable* variable_ptr = NULL
		);

		/**
			@brief	Computes the first BRS for a vehicle, and updates its data with
					BRS1_tau and BRS1 fields. This BRS is used for optimally getting to the
					target despite the worst-case disturbances and moving obstacles induced by
					other vehicles
					@param	[in]	tau			the times for which the BRS is computed; unused times will be cut off
					@param	[in]	g			grid for HJI PDE solver
					@param	[in]	obstacles	union of induced obstacles from higher-priority vehicles
					@param	[in]	SPPP_folder	output folder name
					@param	[in]	vehicle		index of vehicle for which the BRS is computed
					*/
		bool computeBRS1(
			const beacls::FloatVec& tau,
			const HJI_Grid* g,
			const beacls::FloatVec& staticObs,
			const Obstacles& obstacles,
			const std::string& SPPP_folder,
			const size_t vehicle,
			const bool low_memory
		);
		/**
		@brief	Computes the first BRS for a vehicle, and updates its data with
		BRS1_tau and BRS1 fields. This BRS is used for optimally getting to the
		target despite the worst-case disturbances and moving obstacles induced by
		other vehicles
		@param	[in]	tau			the times for which the BRS is computed; unused times will be cut off
		@param	[in]	g			grid for HJI PDE solver
		@param	[in]	obstacles	union of induced obstacles from higher-priority vehicles
		@param	[in]	SPPP_folder	output folder name
		@param	[in]	vehicle		index of vehicle for which the BRS is computed
		*/
		bool computeBRS1(
			const beacls::FloatVec& tau,
			const HJI_Grid* g,
			const std::vector<int8_t>& staticObs,
			const Obstacles& obstacles,
			const std::string& SPPP_folder,
			const size_t vehicle,
			const bool low_memory
		);
		/**
		@brief	Computes nominal trajectory of to be robustly tracked
		@param	[in]	g			grid for HJI PDE solver
		@param	[in]	SPPP_folder	output folder name
		@param	[in]	vehicle		index of vehicle for which the nomTraj is computed
		*/
		bool computeNomTraj(
			const HJI_Grid* g,
			const std::string& SPPP_folder,
			const size_t vehicle,
			helperOC::ComputeOptTraj* computeOptTraj
		);
		/**
		@brief	Computes cylindrical obstacles assuming the RTT method for SPP with
				disturbances or SPP with intruders
				Takes 2D RTTRS augmented with capture radius as input
				*/
		bool computeObsForRTT(
			const SPPProblem* sppp,
			const RTTRS* rttrs,
			const std::vector< beacls::FloatVec>& nomTraj = std::vector< beacls::FloatVec>(),
			const beacls::FloatVec& nomTraj_tau = beacls::FloatVec()
			);
		/**
		@brief	Adds 2D obstacles for visualization. If replanning was done, then the
				appropriately augmented obstacles are added for the times before
				replanning
		*/
		bool addObs2D(
			const SPPProblem* sppp,
			const RTTRS* rttrs,
			const CARS* cars = NULL,
			const RawAugObs* rawAugObs = NULL
		);

		void clear_vReserved() {
			vReserved.clear();
			beacls::FloatVec().swap(vReserved);
		}
		void clear_BRS1() {
			BRS1.clear();
			std::vector<beacls::FloatVec>().swap(BRS1);
		}
		void clear_BRS1_tau() {
			BRS1_tau.clear();
			beacls::FloatVec().swap(BRS1_tau);
		}
		void clear_nomTraj() {
			nomTraj.clear();
			std::vector<beacls::FloatVec>().swap(nomTraj);
		}
		void clear_nomTraj_tau() {
			nomTraj_tau.clear();
			beacls::FloatVec().swap(nomTraj_tau);
		}
		void clear_nomTraj_AR() {
			nomTraj_AR.clear();
			std::vector<beacls::FloatVec>().swap(nomTraj_AR);
		}
		void clear_nomTraj_AR_tau() {
			nomTraj_AR_tau.clear();
			beacls::FloatVec().swap(nomTraj_AR_tau);
		}
		void clear_obsForRTT() {
			obsForRTT.clear();
			std::vector<beacls::FloatVec>().swap(obsForRTT);
		}
		void clear_obsForRTT_s8() {
			obsForRTT_s8.clear();
			std::vector<std::vector<int8_t>>().swap(obsForRTT_s8);
		}
		void clear_obsForRTT_tau() {
			obsForRTT_tau.clear();
			beacls::FloatVec().swap(obsForRTT_tau);
		}
		void clear_obsForIntr() {
			obsForIntr.clear();
			std::vector<beacls::FloatVec>().swap(obsForIntr);
		}
		void clear_obsForIntr_tau() {
			obsForIntr_tau.clear();
			beacls::FloatVec().swap(obsForIntr_tau);
		}
		void clear_FRS1() {
			FRS1.clear();
			std::vector<beacls::FloatVec>().swap(FRS1);
		}
		void clear_FRS1_tau() {
			FRS1_tau.clear();
			beacls::FloatVec().swap(FRS1_tau);
		}
		void clear_obs2D() {
			obs2D.clear();
			std::vector<beacls::FloatVec>().swap(obs2D);
		}
		void clear_obs2D_tau() {
			obs2D_tau.clear();
			beacls::FloatVec().swap(obs2D_tau);
		}
		void clear_target() {
			target.clear();
			beacls::FloatVec().swap(target);
		}
		void clear_targetsm() {
			targetsm.clear();
			beacls::FloatVec().swap(targetsm);
		}
		void clear_targetCenter() {
			targetCenter.clear();
			beacls::FloatVec().swap(targetCenter);
		}


		void set_vReserved(const beacls::FloatVec& val) { vReserved = val; }
		void set_wReserved(const FLOAT_TYPE val) { wReserved = val; }

		void set_v(const FLOAT_TYPE val) { v = val; }

		void set_BRS1(const std::vector<beacls::FloatVec>& val) { BRS1 = val; }
		void set_BRS1_tau(const beacls::FloatVec& val) { BRS1_tau = val; }

		void set_nomTraj(const std::vector<beacls::FloatVec>& val) { nomTraj = val; }
		void set_nomTraj_tau(const beacls::FloatVec& val) { nomTraj_tau = val; }

		void set_nomTraj_AR(const std::vector<beacls::FloatVec>& val) { nomTraj_AR = val; }
		void set_nomTraj_AR_tau(const beacls::FloatVec& val) { nomTraj_AR_tau = val; }

		void set_obsForRTT(const std::vector<beacls::FloatVec>& val) { obsForRTT = val; }
		void set_obsForRTT_s8(const std::vector<std::vector<int8_t>>& val) { obsForRTT_s8 = val; }
		void set_obsForRTT_tau(const beacls::FloatVec& val) { obsForRTT_tau = val; }

		void set_obsForIntr(const std::vector<beacls::FloatVec>& val) { obsForIntr = val; }
		void set_obsForIntr_tau(const beacls::FloatVec& val) { obsForIntr_tau = val; }

		void set_FRS1(const std::vector<beacls::FloatVec>& val) { FRS1 = val; }
		void set_FRS1_tau(const beacls::FloatVec& val) { FRS1_tau = val; }
		void set_FRS1_g(const FLOAT_TYPE val) { FRS1_g = val; }

		void set_obs2D(const std::vector<beacls::FloatVec>& val) { obs2D = val; }
		void set_obs2D_tau(const beacls::FloatVec& val) { obs2D_tau = val; }

		void set_target(const beacls::FloatVec& val) { target = val; }
		void set_targetsm(const beacls::FloatVec& val) { targetsm = val; }
		void set_targetCenter(const beacls::FloatVec& val) { targetCenter = val; }
		void set_targetR(const FLOAT_TYPE val) { targetR = val; }
		void set_targetRsmall(const FLOAT_TYPE val) { targetRsmall = val; }

		void set_replan(const bool val) { replan = val; }
		void set_tauBR(const beacls::FloatVec& val) { tauBR = val; }
		void set_tauAR(const beacls::FloatVec& val) { tauAR = val; }
		void set_tau(const beacls::FloatVec& val) { tau = val; }

		const beacls::FloatVec& get_vReserved() const { return vReserved; }
		FLOAT_TYPE get_wReserved() const { return wReserved; }

		FLOAT_TYPE get_v()  const { return v; }

		const std::vector<beacls::FloatVec>&  get_BRS1()  const { return BRS1; }
		const beacls::FloatVec&  get_BRS1_tau()  const { return BRS1_tau; }

		const std::vector<beacls::FloatVec>&  get_nomTraj() const { return nomTraj; }
		const beacls::FloatVec&  get_nomTraj_tau() const { return nomTraj_tau; }

		const std::vector<beacls::FloatVec>&  get_nomTraj_AR() const { return nomTraj_AR; }
		const beacls::FloatVec&  get_nomTraj_AR_tau()  const { return nomTraj_AR_tau; }

		const std::vector<beacls::FloatVec>& get_obsForRTT() const { return obsForRTT; }
		const std::vector<std::vector<int8_t>>& get_obsForRTT_s8() const { return obsForRTT_s8; }
		const beacls::FloatVec& get_obsForRTT_tau()  const { return obsForRTT_tau; }

		const std::vector<beacls::FloatVec>& get_obsForIntr()  const { return obsForIntr; }
		const beacls::FloatVec& get_obsForIntr_tau() const { return obsForIntr_tau; }

		const std::vector<beacls::FloatVec>&  get_FRS1()  const { return FRS1; }
		const beacls::FloatVec&  get_FRS1_tau()  const { return FRS1_tau; }
		FLOAT_TYPE get_FRS1_g() const { return FRS1_g; }

		const std::vector<beacls::FloatVec>&  get_obs2D()  const { return obs2D; }
		const beacls::FloatVec&  get_obs2D_tau() const { return obs2D_tau; }

		const beacls::FloatVec& get_target() const { return target; }
		const beacls::FloatVec& get_targetsm() const { return targetsm; }
		const beacls::FloatVec& get_targetCenter() const { return targetCenter; }
		FLOAT_TYPE get_targetR() const { return targetR; }
		FLOAT_TYPE get_targetRsmall()  const { return targetRsmall; }

		bool get_replan()  const { return replan; }
		const beacls::FloatVec& get_tauBR() const { return tauBR; }
		const beacls::FloatVec& get_tauAR() const { return tauAR; }
		const beacls::FloatVec& get_tau()  const { return tau; }
		private:
			/** @overload
			Disable operator=
			*/
			Plane& operator=(const Plane& rhs);
			/** @overload
			Disable copy constructor
			*/
			SPPPlane(const SPPPlane& rhs) :
				Plane(rhs),
				vReserved(rhs.vReserved),
				wReserved(rhs.wReserved),

				v(rhs.v),

				BRS1(rhs.BRS1),
				BRS1_tau(rhs.BRS1_tau),

				nomTraj(rhs.nomTraj),
				nomTraj_tau(rhs.nomTraj_tau),

				nomTraj_AR(rhs.nomTraj_AR),
				nomTraj_AR_tau(rhs.nomTraj_AR_tau),

				obsForRTT(rhs.obsForRTT),
				obsForRTT_tau(rhs.obsForRTT_tau),

				obsForIntr(rhs.obsForIntr),
				obsForIntr_tau(rhs.obsForIntr_tau),

				FRS1(rhs.FRS1),
				FRS1_tau(rhs.FRS1_tau),
				FRS1_g(rhs.FRS1_g),

				obs2D(rhs.obs2D),
				obs2D_tau(rhs.obs2D_tau),

				target(rhs.target),
				targetsm(rhs.targetsm),
				targetCenter(rhs.targetCenter),
				targetR(rhs.targetR),
				targetRsmall(rhs.targetRsmall),

				replan(rhs.replan),
				tauBR(rhs.tauBR),
				tauAR(rhs.tauAR),
				tau(rhs.tau)
			{}
	};
};
#endif	/* __SPPPlane_hpp__ */
