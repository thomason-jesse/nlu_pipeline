
#include "actions/ActionFactory.h"

#include "actasp/reasoners/Clingo.h"
#include "actasp/action_utils.h"

#include "msgs_utils.h"
#include "StaticFacts.h"
#include "bwi_kr_execution/UpdateFluents.h"
#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/StaticFactQuery.h"
#include "bwi_kr_execution/ComputePlan.h"
#include "bwi_kr_execution/ComputeAllPlans.h"
#include "bwi_kr_execution/IsPlanValid.h"
#include "bwi_kr_execution/StaticFactQuery.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>


#include <boost/filesystem.hpp>

using namespace actasp;
using namespace std;
using namespace ros;
using namespace bwi_krexec;

const int MAX_N = 20;
const std::string queryDirectory("/tmp/bwi_kr_execution/");

string domainDirectory;


bool updateFluents(bwi_kr_execution::UpdateFluents::Request  &req,
                   bwi_kr_execution::UpdateFluents::Response &res) throw();

bool currentStateQuery(bwi_kr_execution::CurrentStateQuery::Request  &req,
                       bwi_kr_execution::CurrentStateQuery::Response &res) throw();

bool computePlan(bwi_kr_execution::ComputePlan::Request  &req,
                 bwi_kr_execution::ComputePlan::Response &res);

bool computeAllPlans(bwi_kr_execution::ComputeAllPlans::Request  &req,
                     bwi_kr_execution::ComputeAllPlans::Response &res);

bool isPlanvalid(bwi_kr_execution::IsPlanValid::Request  &req,
                 bwi_kr_execution::IsPlanValid::Response &res);

bool resetState(std_srvs::Empty::Request &,
                std_srvs::Empty::Response &);

bool staticFactQuery(bwi_kr_execution::StaticFactQuery::Request &req, bwi_kr_execution::StaticFactQuery::Response &res) throw();

std::list<AspAtom> static_facts;

actasp::AspKR *reasoner;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bwi_kr");
  ros::NodeHandle n;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle privateNode("~");
  
  n.param<std::string>("bwi_kr_execution/domain_directory", domainDirectory, ros::package::getPath("bwi_kr_execution")+"/domain/");
  
  if(domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';
  
  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);

  ActionFactory::setSimulation(simulating);

  boost::filesystem::create_directories(queryDirectory);

  reasoner = new Clingo(MAX_N,"n",queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()));
  reasoner->reset();

  ros::ServiceServer update_fluents = n.advertiseService("update_fluents", updateFluents);
  ros::ServiceServer current_state_query = n.advertiseService("current_state_query", currentStateQuery);
  ros::ServiceServer compute_plan = n.advertiseService("compute_plan", computePlan);
  ros::ServiceServer compute_all_plans = n.advertiseService("compute_all_plans", computeAllPlans);
  ros::ServiceServer is_plan_valid = n.advertiseService("is_plan_valid", isPlanvalid);
  ros::ServiceServer reset_state = n.advertiseService("reset_state", resetState);
  ros::ServiceServer static_fact_query = n.advertiseService("static_fact_query", staticFactQuery);

  //TODO make sure clingo can be executed concurrently, or create multiple instances
// ros::MultiThreadedSpinner m(2); //we don't really want to potentially block all the available cores

  ros::spin();

  delete reasoner;

  return 0;
}

bool updateFluents(bwi_kr_execution::UpdateFluents::Request  &req,
                   bwi_kr_execution::UpdateFluents::Response &res) throw() {

  vector<AspFluent> fluents;
  transform(req.fluents.begin(),req.fluents.end(),back_inserter(fluents),TranslateFluent());

  res.consistent = reasoner->updateFluents(fluents);

  return true;
}

bool currentStateQuery(bwi_kr_execution::CurrentStateQuery::Request  &req,
                       bwi_kr_execution::CurrentStateQuery::Response &res) throw() {

  vector<AspRule> rules;
  transform(req.query.begin(),req.query.end(),back_inserter(rules),TranslateRule());

  AnswerSet answer = reasoner->currentStateQuery(rules);

  res.answer.satisfied = answer.isSatisfied();
  transform(answer.getFluents().begin(),answer.getFluents().end(),back_inserter(res.answer.fluents),TranslateFluent());

  return true;
}

bool staticFactQuery(bwi_kr_execution::StaticFactQuery::Request &req, bwi_kr_execution::StaticFactQuery::Response &res) throw() {

	vector<AspAtom> facts;
	transform(req.facts.begin(),req.facts.end(),back_inserter(facts),TranslateAtom());

	if (static_facts.size() == 0) {
		StaticFacts::retrieveStaticFacts(reasoner, domainDirectory);
		static_facts = StaticFacts::staticFacts(); }

	bool fact_holds = true;
	for (uint i=0; i<facts.size(); i++) {
		fact_holds &= (std::find(static_facts.begin(),static_facts.end(),facts[i]) != static_facts.end()); }

	res.satisfied = fact_holds;

	return true;
}

bool computePlan(bwi_kr_execution::ComputePlan::Request  &req,
                 bwi_kr_execution::ComputePlan::Response &res) {
  vector<AspRule> goal;
  transform(req.goal.begin(),req.goal.end(),back_inserter(goal),TranslateRule());

  //TODO catch exception
  AnswerSet answer = reasoner->computePlan(goal);

  res.plan.satisfied = answer.isSatisfied();
  transform(answer.getFluents().begin(),answer.getFluents().end(),back_inserter(res.plan.fluents),TranslateFluent());

  return true;
}

bool computeAllPlans(bwi_kr_execution::ComputeAllPlans::Request& req, bwi_kr_execution::ComputeAllPlans::Response& res) {

  vector<AspRule> goal;
  transform(req.goal.begin(),req.goal.end(),back_inserter(goal),TranslateRule());

  //TODO catch exception
  vector<actasp::AnswerSet> answers = reasoner->computeAllPlans(goal,req.suboptimality);

  transform(answers.begin(),answers.end(),back_inserter(res.plans),TranslateAnswerSet());

  return true;
}


bool isPlanvalid(bwi_kr_execution::IsPlanValid::Request& req, bwi_kr_execution::IsPlanValid::Response& res) {

  vector<AspRule> goal;
  transform(req.goal.begin(),req.goal.end(),back_inserter(goal),TranslateRule());

  res.valid = reasoner->isPlanValid(TranslateAnswerSet()(req.plan), goal);

  return true;
}

bool resetState(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  reasoner->reset();
}
