#ifndef bwi_kr_execution_msgs_utils_h__guard
#define bwi_kr_execution_msgs_utils_h__guard

#include "bwi_kr_execution/AspFluent.h"
#include "bwi_kr_execution/AspRule.h"
#include "bwi_kr_execution/AnswerSet.h"
#include "bwi_kr_execution/AspAtom.h"

#include "actasp/AspFluent.h"
#include "actasp/AspRule.h"
#include "actasp/AnswerSet.h"
#include "actasp/AspAtom.h"

namespace bwi_krexec {

struct TranslateFluent {
  
 actasp::AspFluent operator()(const bwi_kr_execution::AspFluent& bwiFluent);
 bwi_kr_execution::AspFluent operator()(const actasp::AspFluent& actaspFluent);
};

struct TranslateAtom {
	actasp::AspAtom operator()(const bwi_kr_execution::AspAtom& bwiAtom);
	bwi_kr_execution::AspAtom operator()(const actasp::AspAtom& actaspAtom);
};

struct TranslateRule {
  actasp::AspRule operator()(const bwi_kr_execution::AspRule& bwiRule);
  bwi_kr_execution::AspRule operator()(const actasp::AspRule& actaspRule);

};

struct TranslateAnswerSet {
  actasp::AnswerSet operator()(const bwi_kr_execution::AnswerSet& bwiAnswerSet);
  bwi_kr_execution::AnswerSet operator()(const actasp::AnswerSet& actaspAnswerSet);
  
};

}


#endif
