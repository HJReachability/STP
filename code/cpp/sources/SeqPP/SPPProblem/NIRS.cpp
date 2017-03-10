#include <SeqPP/SPPProblem/NIRS.hpp>
#include <levelset/Grids/HJI_Grid.hpp>
#include <SeqPP/SPPPlane/SPPPlane.hpp>
#include <sstream>
#include <thread>

SeqPP::NIRS::NIRS() :
	Q(std::vector<SPPPlane*>()),
	to_save(false)
{
}
SeqPP::NIRS::NIRS(const size_t num_of_vehicles) :
	to_save(false)
{
	Q.resize(num_of_vehicles,NULL);
}
SeqPP::NIRS::~NIRS()
{
	std::for_each(Q.begin(), Q.end(), [](auto& rhs) { if (rhs) delete rhs; });
	}
bool SeqPP::NIRS::get_to_save() const
{
	return to_save;
}
void SeqPP::NIRS::set_to_save(const bool flag)
{
	to_save = flag;
}
size_t SeqPP::NIRS::get_num_of_vehicles() const {
	return Q.size();
}


bool SeqPP::NIRS::load(
	const helperOC::ExecParameters& execParameters,
	beacls::MatFStream* fs
) {
	beacls::MatVariable* Q_var = beacls::openMatVariable(fs, std::string("Q"));
	if (!Q_var) {
		std::cerr << "Error: Q is not found" << std::endl;
		return false;
	}
	const size_t num_of_vehicles = beacls::getCellSize(Q_var);
	Q.resize(num_of_vehicles);
	
	for (size_t i = 0; i < num_of_vehicles; ++i) {
		std::cout << "Loading " << i << "th vehicle..." << std::endl;
		beacls::MatVariable* vehicle_var = beacls::getVariableFromCell(Q_var, i);
		SPPPlane* plane = new SPPPlane(execParameters, fs, vehicle_var);
		beacls::closeMatVariable(vehicle_var);
		Q[i] = plane;
	}
	beacls::closeMatVariable(Q_var);
	return true;
}
bool SeqPP::NIRS::save(
	beacls::MatFStream* fs,
	const bool clear,
	const size_t num_of_keeps
) {
	const size_t num_of_vehicles = Q.size();
	beacls::MatVariable* Q_var = beacls::createMatCell(std::string("Q"), num_of_vehicles);
	bool result = true;
	for (size_t i = 0; i < num_of_vehicles; ++i) {
		if (Q[i]) {
			beacls::MatVariable* Q_i_var = beacls::createMatStruct(std::string("Q"));
			result &= Q[i]->save(fs, Q_i_var);
			if (clear) {
				if (i < (num_of_vehicles - num_of_keeps)) {
					delete Q[i];
					Q[i] = NULL;
				}
			}
			beacls::setVariableToCell(Q_var, Q_i_var, i);
			beacls::closeMatVariable(Q_i_var);
		}
	}
	beacls::writeMatVariable(fs, Q_var);
	beacls::closeMatVariable(Q_var);
	return result;
}

void SeqPP::NIRS::set_Q(const std::vector<SeqPP::SPPPlane*>& v) {
	Q = v;
}
void SeqPP::NIRS::initializeQ(const size_t num_of_vehicles) {
	Q.resize(num_of_vehicles, NULL);
}
std::vector<SeqPP::SPPPlane*> SeqPP::NIRS::get_Q() const {
	return Q;
}
void SeqPP::NIRS::set_Vehicle(SeqPP::SPPPlane* v, const size_t veh) {
	if (Q.size() <= veh) Q.resize(veh+1, NULL);
	if (Q[veh]) delete Q[veh];
	Q[veh] = v;
}
void SeqPP::NIRS::clear_Vehicle(const size_t veh) {
	if (Q.size() <= veh) return;
	if (Q[veh]) {
		delete Q[veh];
		Q[veh] = NULL;
	}
}
SeqPP::SPPPlane* SeqPP::NIRS::get_Vehicle(const size_t veh) const {
	if (veh<Q.size()) return Q[veh];
	else return NULL;
}