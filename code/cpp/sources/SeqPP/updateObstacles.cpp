#include <functional>
#include <algorithm>
#include <macro.hpp>
#include <iostream>
#include <SeqPP/updateObstacles.hpp>
#include <thread>
bool SeqPP::updateObstacles(
	Obstacles& obstacles,
	const beacls::FloatVec& newObs_tau,
	const std::vector<beacls::FloatVec>& newObs,
	const beacls::FloatVec& staticObs
) {
	const FLOAT_TYPE small = (FLOAT_TYPE)1e-4;

	if (obstacles.tau.empty() && newObs_tau.empty()) return false;
	if (obstacles.tau.empty()) {
		obstacles.data = newObs;
		obstacles.tau = newObs_tau;
		return true;
	}
	if (newObs_tau.empty()) {
		return true;
	}
	auto minmax_src_oblstacles_tau = beacls::minmax_value<FLOAT_TYPE>(obstacles.tau.cbegin(), obstacles.tau.cend());
	const FLOAT_TYPE min_srcObs_tau = minmax_src_oblstacles_tau.first;
	const FLOAT_TYPE max_srcObs_tau = minmax_src_oblstacles_tau.second;

	Obstacles tmp_obstacles;

	//!< Update obstacle data for smaller time indices
	tmp_obstacles.tau.reserve(newObs_tau.size() + obstacles.tau.size());
	tmp_obstacles.data.reserve(newObs.size() + obstacles.data.size());
	const size_t staticObsSize = staticObs.size();
	beacls::FloatVec obsSmaller;
	for (size_t t = 0; t < newObs_tau.size(); ++t) {
		if (newObs_tau[t] < min_srcObs_tau - small) {
			tmp_obstacles.tau.push_back(newObs_tau[t]);
			if (staticObsSize == 0) {
				tmp_obstacles.data.push_back(newObs[t]);
			}
			else {
				obsSmaller.resize(newObs[t].size());
				const size_t reptimes = newObs[t].size() / staticObsSize;
				for (size_t i = 0; i < reptimes; ++i) {
					const size_t offset3D = staticObsSize*i;
					std::transform(staticObs.cbegin(), staticObs.cend(), newObs[t].cbegin() + offset3D,
						obsSmaller.begin() + offset3D,
						[](const auto&lhs, const auto& rhs) {
						return (lhs < rhs) ? lhs : rhs;
					});
				}
				tmp_obstacles.data.push_back(obsSmaller);
			}
		}
	}

	//!< Update obstacle data for overlapping time indices
	auto minmax_newObs_tau = beacls::minmax_value<FLOAT_TYPE>(newObs_tau.cbegin(), newObs_tau.cend());
	const FLOAT_TYPE min_newObs_tau = minmax_newObs_tau.first;
	const FLOAT_TYPE max_newObs_tau = minmax_newObs_tau.second;
	auto first_newObs_tau_overlapped_ite = std::find_if(newObs_tau.cbegin(), newObs_tau.cend(), [&min_srcObs_tau, &max_srcObs_tau, &small](const auto& rhs) {
		return ((rhs > (min_srcObs_tau - small)) && (rhs < (max_srcObs_tau + small)));
	});
	auto last_newObs_tau_overlapped_ite = std::find_if(first_newObs_tau_overlapped_ite, newObs_tau.cend(), [&min_srcObs_tau, &max_srcObs_tau, &small](const auto& rhs) {
		return !((rhs >(min_srcObs_tau - small)) && (rhs < (max_srcObs_tau + small)));
	});
	auto first_srcObs_tau_overlapped_ite = std::find_if(obstacles.tau.cbegin(), obstacles.tau.cend(), [&min_newObs_tau, &max_newObs_tau, &small](const auto& rhs) {
		return ((rhs >(min_newObs_tau - small)) && (rhs < (max_newObs_tau + small)));
	});
	auto last_srcObs_tau_overlapped_ite = std::find_if(first_srcObs_tau_overlapped_ite, obstacles.tau.cend(), [&min_newObs_tau, &max_newObs_tau, &small](const auto& rhs) {
		return !((rhs >(min_newObs_tau - small)) && (rhs < (max_newObs_tau + small)));
	});
	if (std::distance(first_newObs_tau_overlapped_ite, last_newObs_tau_overlapped_ite) == 0) {
		std::cerr << "There must be overlap between the time stamps!" << std::endl;
		return false;
	}
	const size_t first_newObs_tau_overlapped_index = std::distance(newObs_tau.cbegin(), first_newObs_tau_overlapped_ite);
	const size_t last_newObs_tau_overlapped_index = std::distance(newObs_tau.cbegin(), last_newObs_tau_overlapped_ite);
	const size_t first_srcObs_tau_overlapped_index = std::distance(obstacles.tau.cbegin(), first_srcObs_tau_overlapped_ite);
	const size_t last_srcObs_tau_overlapped_index = std::distance(obstacles.tau.cbegin(), last_srcObs_tau_overlapped_ite);
	const size_t overlaped_length = std::min<size_t>(last_newObs_tau_overlapped_index - first_newObs_tau_overlapped_index, last_srcObs_tau_overlapped_index - first_srcObs_tau_overlapped_index);
	const size_t newObsSmaller_length = tmp_obstacles.data.size();
	tmp_obstacles.data.resize(newObsSmaller_length + obstacles.data.size());
	for (size_t t = 0; t < first_srcObs_tau_overlapped_index; ++t) {
		obstacles.data[t].swap(tmp_obstacles.data[t + newObsSmaller_length]);
		beacls::FloatVec().swap(obstacles.data[t]);
	}
	for (size_t t = 0; t<overlaped_length;++t) {
		const size_t src_org_index = t + first_srcObs_tau_overlapped_index;
		const size_t src_new_index = t + first_newObs_tau_overlapped_index;
		const size_t dst_index = src_org_index + newObsSmaller_length;
		if (obstacles.data[src_org_index].empty()) {
			tmp_obstacles.data[dst_index] = newObs[src_new_index];
		}
		else {
			tmp_obstacles.data[dst_index].resize(obstacles.data[src_org_index].size());
			std::transform(
				obstacles.data[src_org_index].cbegin(),
				obstacles.data[src_org_index].cend(),
				newObs[src_new_index].cbegin(),
				tmp_obstacles.data[dst_index].begin(),
				[](const auto&lhs, const auto& rhs) {
				return (lhs < rhs) ? lhs : rhs;
			});
			beacls::FloatVec().swap(obstacles.data[src_org_index]);
		}
	}
	for (size_t t = 0; t < (obstacles.data.size()- overlaped_length- first_srcObs_tau_overlapped_index); ++t) {
		const size_t src_org_index = t + first_srcObs_tau_overlapped_index + overlaped_length;
		const size_t dst_index = src_org_index + newObsSmaller_length;
		if (!obstacles.data[src_org_index].empty()) {
			obstacles.data[src_org_index].swap(tmp_obstacles.data[dst_index]);
			beacls::FloatVec().swap(obstacles.data[src_org_index]);
		}
	}
	tmp_obstacles.tau.insert(tmp_obstacles.tau.end(), obstacles.tau.cbegin(), obstacles.tau.cend());
	tmp_obstacles.tau.swap(obstacles.tau);
	tmp_obstacles.data.swap(obstacles.data);

	//!< No need to update obstacle data for larger time indices
	return true;
}
bool SeqPP::updateObstacles(
	Obstacles& obstacles,
	const beacls::FloatVec& newObs_tau,
	const std::vector<std::vector<int8_t>>& newObs,
	const std::vector<int8_t>& staticObs
) {
	const FLOAT_TYPE small = (FLOAT_TYPE)1e-4;

	if (obstacles.tau.empty() && newObs_tau.empty()) return false;
	if (obstacles.tau.empty()) {
		obstacles.data_s8 = newObs;
		obstacles.tau = newObs_tau;
		return true;
	}
	if (newObs_tau.empty()) {
		return true;
	}
	auto minmax_src_oblstacles_tau = beacls::minmax_value<FLOAT_TYPE>(obstacles.tau.cbegin(), obstacles.tau.cend());
	const FLOAT_TYPE min_srcObs_tau = minmax_src_oblstacles_tau.first;
	const FLOAT_TYPE max_srcObs_tau = minmax_src_oblstacles_tau.second;

	Obstacles tmp_obstacles;

	//!< Update obstacle data for smaller time indices
	tmp_obstacles.tau.reserve(newObs_tau.size() + obstacles.tau.size());
	tmp_obstacles.data_s8.reserve(newObs.size() + obstacles.data_s8.size());
	const size_t staticObsSize = staticObs.size();
	std::vector<int8_t> obsSmaller;
	for (size_t t = 0; t < newObs_tau.size(); ++t) {
		if (newObs_tau[t] < min_srcObs_tau - small) {
			tmp_obstacles.tau.push_back(newObs_tau[t]);
			if (staticObsSize == 0) {
				tmp_obstacles.data_s8.push_back(newObs[t]);
			}
			else {
				obsSmaller.resize(newObs[t].size());
				const size_t reptimes = newObs[t].size() / staticObsSize;
				for (size_t i = 0; i < reptimes; ++i) {
					const size_t offset3D = staticObsSize*i;
					std::transform(staticObs.cbegin(), staticObs.cend(), newObs[t].cbegin() + offset3D,
						obsSmaller.begin() + offset3D,
						[](const auto&lhs, const auto& rhs) {
						return (lhs < rhs) ? lhs : rhs;
					});
				}
				tmp_obstacles.data_s8.push_back(obsSmaller);
			}
		}
	}

	//!< Update obstacle data for overlapping time indices
	auto minmax_newObs_tau = beacls::minmax_value<FLOAT_TYPE>(newObs_tau.cbegin(), newObs_tau.cend());
	const FLOAT_TYPE min_newObs_tau = minmax_newObs_tau.first;
	const FLOAT_TYPE max_newObs_tau = minmax_newObs_tau.second;
	auto first_newObs_tau_overlapped_ite = std::find_if(newObs_tau.cbegin(), newObs_tau.cend(), [&min_srcObs_tau, &max_srcObs_tau, &small](const auto& rhs) {
		return ((rhs > (min_srcObs_tau - small)) && (rhs < (max_srcObs_tau + small)));
	});
	auto last_newObs_tau_overlapped_ite = std::find_if(first_newObs_tau_overlapped_ite, newObs_tau.cend(), [&min_srcObs_tau, &max_srcObs_tau, &small](const auto& rhs) {
		return !((rhs >(min_srcObs_tau - small)) && (rhs < (max_srcObs_tau + small)));
	});
	auto first_srcObs_tau_overlapped_ite = std::find_if(obstacles.tau.cbegin(), obstacles.tau.cend(), [&min_newObs_tau, &max_newObs_tau, &small](const auto& rhs) {
		return ((rhs >(min_newObs_tau - small)) && (rhs < (max_newObs_tau + small)));
	});
	auto last_srcObs_tau_overlapped_ite = std::find_if(first_srcObs_tau_overlapped_ite, obstacles.tau.cend(), [&min_newObs_tau, &max_newObs_tau, &small](const auto& rhs) {
		return !((rhs >(min_newObs_tau - small)) && (rhs < (max_newObs_tau + small)));
	});
	if (std::distance(first_newObs_tau_overlapped_ite, last_newObs_tau_overlapped_ite) == 0) {
		std::cerr << "There must be overlap between the time stamps!" << std::endl;
		return false;
	}
	const size_t first_newObs_tau_overlapped_index = std::distance(newObs_tau.cbegin(), first_newObs_tau_overlapped_ite);
	const size_t last_newObs_tau_overlapped_index = std::distance(newObs_tau.cbegin(), last_newObs_tau_overlapped_ite);
	const size_t first_srcObs_tau_overlapped_index = std::distance(obstacles.tau.cbegin(), first_srcObs_tau_overlapped_ite);
	const size_t last_srcObs_tau_overlapped_index = std::distance(obstacles.tau.cbegin(), last_srcObs_tau_overlapped_ite);
	const size_t overlaped_length = std::min<size_t>(last_newObs_tau_overlapped_index - first_newObs_tau_overlapped_index, last_srcObs_tau_overlapped_index - first_srcObs_tau_overlapped_index);
	const size_t newObsSmaller_length = tmp_obstacles.data_s8.size();
	tmp_obstacles.data_s8.resize(newObsSmaller_length + obstacles.data_s8.size());
	for (size_t t = 0; t < first_srcObs_tau_overlapped_index; ++t) {
		obstacles.data_s8[t].swap(tmp_obstacles.data_s8[t + newObsSmaller_length]);
		std::vector<int8_t>().swap(obstacles.data_s8[t]);
	}
	for (size_t t = 0; t<overlaped_length; ++t) {
		const size_t src_org_index = t + first_srcObs_tau_overlapped_index;
		const size_t src_new_index = t + first_newObs_tau_overlapped_index;
		const size_t dst_index = src_org_index + newObsSmaller_length;
		if (obstacles.data_s8[src_org_index].empty()) {
			tmp_obstacles.data_s8[dst_index] = newObs[src_new_index];
		}
		else {
			tmp_obstacles.data_s8[dst_index].resize(obstacles.data_s8[src_org_index].size());
			std::transform(
				obstacles.data_s8[src_org_index].cbegin(),
				obstacles.data_s8[src_org_index].cend(),
				newObs[src_new_index].cbegin(),
				tmp_obstacles.data_s8[dst_index].begin(),
				[](const auto&lhs, const auto& rhs) {
				return (lhs < rhs) ? lhs : rhs;
			});
			std::vector<int8_t>().swap(obstacles.data_s8[src_org_index]);
		}
	}
	for (size_t t = 0; t < (obstacles.data_s8.size() - overlaped_length - first_srcObs_tau_overlapped_index); ++t) {
		const size_t src_org_index = t + first_srcObs_tau_overlapped_index + overlaped_length;
		const size_t dst_index = src_org_index + newObsSmaller_length;
		if (!obstacles.data_s8[src_org_index].empty()) {
			obstacles.data_s8[src_org_index].swap(tmp_obstacles.data_s8[dst_index]);
			std::vector<int8_t>().swap(obstacles.data_s8[src_org_index]);
		}
	}
	tmp_obstacles.tau.insert(tmp_obstacles.tau.end(), obstacles.tau.cbegin(), obstacles.tau.cend());
	tmp_obstacles.tau.swap(obstacles.tau);
	tmp_obstacles.data_s8.swap(obstacles.data_s8);

	//!< No need to update obstacle data for larger time indices
	return true;
}
