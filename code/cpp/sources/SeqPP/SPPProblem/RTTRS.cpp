#include <SeqPP/SPPProblem/RTTRS.hpp>
#include <levelset/Grids/HJI_Grid.hpp>
#include <helperOC/DynSys/PlaneCAvoid/PlaneCAvoid.hpp>
#include <helperOC/DynSys/PlaneCAvoid/PlaneCAvoid.hpp>
SeqPP::RTTRS::RTTRS() : g(NULL), dynSys(NULL)
{
	}
SeqPP::RTTRS::~RTTRS()
{
	if (g) delete g;
	if (dynSys) delete dynSys;
	}
SeqPP::RTTRS::RTTRS(const RTTRS& rhs) :
	g(rhs.g->clone()),
	dynSys(rhs.dynSys->clone())
{
}
SeqPP::RTTRS* SeqPP::RTTRS::clone() const {
	return new SeqPP::RTTRS(*this);
}

bool SeqPP::RTTRS::load(
	beacls::MatVariable* variable_ptr
	) {
	beacls::MatFStream* fs = NULL;
	if (g) delete g;
	g = new HJI_Grid();
	g->load_grid(std::string("g"), fs, variable_ptr);
	beacls::IntegerVec dummy;
	load_vector(data, std::string("data"), dummy, true, fs, variable_ptr);
	load_value(trackingRadius, std::string("trackingRadius"),true, fs, variable_ptr);
	if (dynSys) delete dynSys;
	beacls::MatVariable* dynSys_var = beacls::getVariableFromStruct(variable_ptr, std::string("dynSys"));
	dynSys = new PlaneCAvoid(fs, dynSys_var);
	beacls::closeMatVariable(dynSys_var);
	load_vector_of_vectors(Deriv, std::string("Deriv"), dummy, true, fs, variable_ptr);
	return true;
}
bool SeqPP::RTTRS::save(
	beacls::MatVariable* variable_ptr
) const {
	bool result = true;
	beacls::MatFStream* fs = NULL;

	if (g) result &= g->save_grid(std::string("g"), fs, variable_ptr);
	result &= save_value(trackingRadius, std::string("trackingRadius"), true, fs, variable_ptr);
	beacls::IntegerVec Ns = g ? g->get_Ns() :  beacls::IntegerVec();
	if (!data.empty())result &= save_vector(data, std::string("data"), Ns, true, fs, variable_ptr);
	if (dynSys) {
		beacls::MatVariable* dynSys_var = beacls::createMatStruct(std::string("dynSys"));
		result &= dynSys->save(fs, dynSys_var);
		beacls::setVariableToStruct(variable_ptr, dynSys_var, std::string("dynSys"));
		beacls::closeMatVariable(dynSys_var);
	}

	if (!Deriv.empty()) result &= save_vector_of_vectors(Deriv, std::string("Deriv"), Ns, true, fs, variable_ptr);
	return result;
}
void SeqPP::RTTRS::set_trackingRadius(const FLOAT_TYPE v) {
	trackingRadius = v;
}
void SeqPP::RTTRS::set_g(const HJI_Grid* v) {
	g = v->clone();
}
void SeqPP::RTTRS::set_data(const beacls::FloatVec& v) {
	data = v;
}
void SeqPP::RTTRS::set_Deriv(const std::vector<beacls::FloatVec>& v) {
	Deriv = v;
}
void SeqPP::RTTRS::set_dynSys(PlaneCAvoid* v) {
	dynSys = v->clone();
}
FLOAT_TYPE SeqPP::RTTRS::get_trackingRadius() const {
	return trackingRadius;
}
HJI_Grid* SeqPP::RTTRS::get_g() const {
	return g;
}
const beacls::FloatVec& SeqPP::RTTRS::get_data() const {
	return data;
}
const std::vector<beacls::FloatVec>& SeqPP::RTTRS::get_Deriv() const {
	return Deriv;
}
PlaneCAvoid* SeqPP::RTTRS::get_dynSys() const {
	return dynSys;
}
