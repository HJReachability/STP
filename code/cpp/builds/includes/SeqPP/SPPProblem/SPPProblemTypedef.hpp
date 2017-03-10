#ifndef __SPPProblemTypedef_hpp__
#define __SPPProblemTypedef_hpp__

//! Prefix to generate Visual C++ DLL
#ifdef _MAKE_VC_DLL
#define PREFIX_VC_DLL __declspec(dllexport)

//! Don't add prefix, except dll generating
#else
#define PREFIX_VC_DLL
#endif

#include <cstdint>
#include <vector>
#include <cstring>
#include <iostream>
#include <typedef.hpp>
namespace SeqPP {
	typedef enum ProblemType {
		ProblemType_Invalid = -1,
		ProblemType_default,
		ProblemType_SF_dstb,	//!< 1
		ProblemType_SF_intr_2,	//!< 2
		ProblemType_SF_intr_3,	//!< 3
		ProblemType_SF_intr_4,	//!< 4
		ProblemType_Bay_Area,	//!< 5
		ProblemType_TCST_dstb,	//!< 6
		ProblemType_TCST_intr,	//!< 7
		ProblemType_SF_dstb_5veh,	//!< 8
		ProblemType_SF_dstb_10veh,	//!< 9
		ProblemType_SF_dstb_20veh,	//!< 10
		ProblemType_SF_dstb_30veh,	//!< 11
		ProblemType_SF_dstb_40veh,	//!< 12
		ProblemType_SF_dstb_hell,	//!< 13
	} ProblemType;
	typedef enum Setup_Type {
		Setup_Type_Invalid = -1,
		Setup_Type_Default = 0,
		Setup_Type_SF,
		Setup_Type_Bay_Area,
	} Setup_Type;
	typedef enum DstbIntr_Type {
		DstbIntr_Type_Invalid = -1,
		DstbIntr_Type_Default = 0,
		DstbIntr_Type_Dstb,
		DstbIntr_Type_Intr,
	} DstbIntr_Type;
	class SetupExtraArgs {
	public:
		size_t number_of_vehicles;
		FLOAT_TYPE wind_speed;
		FLOAT_TYPE separation_time;
		DstbIntr_Type dstbOrIntr;
		std::string ISTC_filename;
		bool randomSeedForTargets;
		bool lowprecision_obstacles;
	public:
		SetupExtraArgs() :
			number_of_vehicles(1),
			wind_speed(0),
			separation_time(-1),
			dstbOrIntr(DstbIntr_Type_Default),
			ISTC_filename(std::string()),
			randomSeedForTargets(true),
			lowprecision_obstacles(false)
		{};

	};
	typedef enum CheckPointType {
		CheckPointType_Invalid = -1,
		CheckPointType_None,
		CheckPointType_Merged,
		CheckPointType_Separated,

	}CheckPointType;
	class Obstacles {
	public:
		std::vector<beacls::FloatVec> data;
		std::vector<std::vector<int8_t>> data_s8;
		beacls::FloatVec tau;
		Obstacles() :
			data(std::vector<beacls::FloatVec>()),
			data_s8(std::vector <std::vector<int8_t >>()),
			tau(beacls::FloatVec()) {}
		~Obstacles() {}
		Obstacles& operator=(const Obstacles& rhs) {
			if (this == &rhs) return *this;
			this->data = rhs.data;
			this->data_s8 = rhs.data_s8;
			this->tau = rhs.tau;
			return *this;
		}
		Obstacles(const Obstacles& rhs) : 
			data(rhs.data),
			data_s8(rhs.data_s8),
			tau(rhs.tau) {}
	private:
	};
};
#endif	/* __SPPProblemTypedef_hpp__ */
