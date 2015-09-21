
#include "msgs_utils.h"

#include <algorithm>
#include <iterator>

using namespace std;

namespace bwi_krexec {

actasp::AspFluent TranslateFluent::operator()(const bwi_kr_execution::AspFluent& bwiFluent) {
  return actasp::AspFluent(bwiFluent.name,bwiFluent.variables,bwiFluent.timeStep);
}

bwi_kr_execution::AspFluent TranslateFluent::operator()(const actasp::AspFluent& actaspFluent) {

  bwi_kr_execution::AspFluent bwiFluent;
  bwiFluent.name = actaspFluent.getName();
  bwiFluent.timeStep = actaspFluent.getTimeStep();
  bwiFluent.variables = actaspFluent.getParameters();

  return bwiFluent;
}

actasp::AspAtom TranslateAtom::operator()(const bwi_kr_execution::AspAtom& bwiAtom) {
	std::string atom_str = bwiAtom.name+"(";
	for (uint i=0; i<bwiAtom.params.size(); i++) {
		atom_str += bwiAtom.params[i];
		if (i+1 < bwiAtom.params.size()) atom_str += ","; }
	atom_str += ")";
	return actasp::AspAtom(atom_str);
}
bwi_kr_execution::AspAtom TranslateAtom::operator()(const actasp::AspAtom& actaspAtom) {
	bwi_kr_execution::AspAtom bwiAtom;
	bwiAtom.name = actaspAtom.getName();
	bwiAtom.params = actaspAtom.getParameters();
	return bwiAtom;
}

actasp::AspRule TranslateRule::operator()(const bwi_kr_execution::AspRule& bwiRule) {
  actasp::AspRule actaspRule;

  transform(bwiRule.head.begin(), bwiRule.head.end(), back_inserter(actaspRule.head), TranslateFluent());
  transform(bwiRule.body.begin(), bwiRule.body.end(), back_inserter(actaspRule.body), TranslateFluent());


  return actaspRule;
}

bwi_kr_execution::AspRule TranslateRule::operator()(const actasp::AspRule& actaspRule) {
  bwi_kr_execution::AspRule bwiRule;

  transform(actaspRule.head.begin(), actaspRule.head.end(), back_inserter(bwiRule.head), TranslateFluent());
  transform(actaspRule.body.begin(), actaspRule.body.end(), back_inserter(bwiRule.body), TranslateFluent());

  return bwiRule;

}

actasp::AnswerSet TranslateAnswerSet::operator()(const bwi_kr_execution::AnswerSet& bwiAnswerSet) {
  
  if(!bwiAnswerSet.satisfied)
    return actasp::AnswerSet();
  
  list<actasp::AspFluent> fluents;
  transform(bwiAnswerSet.fluents.begin(), bwiAnswerSet.fluents.end(), back_inserter(fluents), TranslateFluent());
  
  return actasp::AnswerSet(fluents.begin(), fluents.end());
}

bwi_kr_execution::AnswerSet TranslateAnswerSet::operator()(const actasp::AnswerSet& actaspAnswerSet) {
  bwi_kr_execution::AnswerSet bwiAnswerSet;
  
  transform(actaspAnswerSet.getFluents().begin(),actaspAnswerSet.getFluents().end(),back_inserter(bwiAnswerSet.fluents),TranslateFluent());
  bwiAnswerSet.satisfied = actaspAnswerSet.isSatisfied();
  
  return bwiAnswerSet;
}




}
