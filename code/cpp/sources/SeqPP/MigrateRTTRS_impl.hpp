#ifndef __MigrateRTTRS_impl_hpp__
#define __MigrateRTTRS_impl_hpp__

#include <typedef.hpp>
#include <vector>
namespace levelset {
	class HJI_Grid;
}
namespace helperOC {
	class AddCRadius;
};

namespace SeqPP {
	class RTTRS;
	class MigrateRTTRS_impl {
	private:
		helperOC::AddCRadius* addCRadius;
	public:
		MigrateRTTRS_impl(
			const helperOC::ExecParameters& execParameters
		);
		~MigrateRTTRS_impl();
		levelset::HJI_Grid* operator()(
			beacls::FloatVec& rttrs2d,
			const RTTRS* rttrs,
			const FLOAT_TYPE R_augment
		);
	private:
		MigrateRTTRS_impl(const MigrateRTTRS_impl& rhs);
		MigrateRTTRS_impl& operator=(const MigrateRTTRS_impl& rhs);
	};
};
#endif	/* __MigrateRTTRS_impl_hpp__ */

