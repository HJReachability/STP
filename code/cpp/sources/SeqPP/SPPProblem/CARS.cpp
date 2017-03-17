#include <SeqPP/SPPProblem/CARS.hpp>
#include <levelset/Grids/HJI_Grid.hpp>
#include <helperOC/DynSys/PlaneCAvoid/PlaneCAvoid.hpp>
#include <helperOC/DynSys/PlaneCAvoid/PlaneCAvoid.hpp>
SeqPP::CARS::CARS() : g(NULL), dynSys(NULL)
{
	}
SeqPP::CARS::~CARS()
{
	if (g) delete g;
	if (dynSys) delete dynSys;
	}
SeqPP::CARS::CARS(const CARS& rhs) :
	g(rhs.g->clone()),
	dynSys(rhs.dynSys->clone())
{
}
SeqPP::CARS* SeqPP::CARS::clone() const {
	return new SeqPP::CARS(*this);
}

bool SeqPP::CARS::load(
	beacls::MatFStream* fs,
	beacls::MatVariable* variable_ptr) {
	if (g) delete g;
	g = new HJI_Grid();
	g->load_grid(std::string("g"), fs, variable_ptr);
	beacls::IntegerVec dummy;
	load_vector_of_vectors(datas, std::string("data"), dummy, true, fs, variable_ptr);
	load_vector(tau, std::string("tau"), dummy, true, fs, variable_ptr);
	if (dynSys) delete dynSys;
	beacls::MatVariable* dynSys_var = beacls::getVariableFromStruct(variable_ptr, std::string("dynSys"));
	dynSys = new PlaneCAvoid(fs, dynSys_var);
	beacls::closeMatVariable(dynSys_var);
	if (!dynSys) {
		std::cerr << "Cannot load dynSys from:"  << std::endl;
		return false;
	}
	return true;
}
bool SeqPP::CARS::save(
	beacls::MatFStream* fs,
	beacls::MatVariable* variable_ptr
) const {
	bool result = true;
	if (g) result &= g->save_grid(std::string("g"), fs, variable_ptr);
	if (!datas.empty()) result &= save_vector_of_vectors(datas, std::string("data"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (!tau.empty()) result &= save_vector(tau, std::string("tau"), beacls::IntegerVec(), true, fs, variable_ptr);
	if (dynSys) {
		beacls::MatVariable* dynSys_var = beacls::createMatStruct(std::string("dynSys"));
		result &= dynSys->save(fs, dynSys_var);
		beacls::setVariableToStruct(variable_ptr, dynSys_var, std::string("dynSys"));
		beacls::closeMatVariable(dynSys_var);
	}
	return result;
}
void SeqPP::CARS::set_g(const HJI_Grid* v) {
	g = v->clone();
}
void SeqPP::CARS::set_data(const std::vector<beacls::FloatVec>& v) {
	datas = v;
}
void SeqPP::CARS::set_tau(const beacls::FloatVec& v) {
	tau = v;
}
void SeqPP::CARS::set_dynSys(PlaneCAvoid* v) {
	dynSys = v->clone();
}
HJI_Grid* SeqPP::CARS::get_g() const {
	return g;
}
const std::vector<beacls::FloatVec>& SeqPP::CARS::get_data() const {
	return datas;
}
const beacls::FloatVec& SeqPP::CARS::get_tau() const {
	return tau;
}
PlaneCAvoid* SeqPP::CARS::get_dynSys() const {
	return dynSys;
}
