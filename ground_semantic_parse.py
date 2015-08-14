#!/usr/bin/env python

import sys
import rospy
from bwi_kr_execution.srv import *
from bwi_kr_execution.msg import *

def current_state_query_client(req):
	rospy.wait_for_service('current_state_query')
	try:
		current_state_query = rospy.ServiceProxy('current_state_query', CurrentStateQuery)
		res = current_state_query(req)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def compute_plan_client(req):
	rospy.wait_for_service('compute_plan')
	try:
		compute_plan = rospy.ServiceProxy('compute_plan', ComputePlan)
		res = compute_plan(req)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def static_fact_query_client(req):
	rospy.wait_for_service('static_fact_query')
	try:
		static_fact_query = rospy.ServiceProxy('static_fact_query', StaticFactQuery)
		res = static_fact_query(req)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	if (sys.argv[1] == '-q'): req = CurrentStateQueryRequest()
	elif (sys.argv[1] == '-p'): req = ComputePlanRequest()
	elif (sys.argv[1] == '-f'): req = StaticFactQueryRequest()
	else: sys.exit("-q query or -p plan or -f fact")
	if (len(sys.argv) == 3 and sys.argv[1] != '-f'):
		rules = sys.argv[2].split("|")
		for rule in rules:
			r = AspRule()
			rhs,lhs = rule.split(":-")
			for statement in rhs.strip().split(";"):
				if (len(statement) == 0): continue
				f = AspFluent()
				f.name = statement.split("(")[0].strip()
				f.variables = statement.split("(")[1].strip()[:-1].split(",")
				r.head.append(f)
			for statement in lhs.strip().split(";"):
				if (len(statement) == 0): continue
				f = AspFluent()
				f.name = statement.split("(")[0].strip()
				f.variables = statement.split("(")[1].strip()[:-1].split(",")
				r.body.append(f)
			if (sys.argv[1] == '-q'): req.query.append(r)
			elif (sys.argv[1] == '-p'): req.goal.append(r)
	elif (len(sys.argv) == 3 and sys.argv[1] == '-f'):
		for statement in sys.argv[2].strip().split(";"):
			a = AspAtom()
			a.name = statement.split("(")[0].strip()
			a.params = statement.split("(")[1].strip()[:-1].split(",")
			req.facts.append(a)
	elif (len(sys.argv) != 2): sys.exit("invalid call")
	if (sys.argv[1] == '-q'):
		print str(req)
		res = current_state_query_client(req)
	elif (sys.argv[1] == '-p'):
		#add hri fluent
		r = AspRule()
		f = AspFluent()
		f.name = "findPersonTask"
		r.head.append(f)
		req.goal.append(r)
		print str(req)
		res = compute_plan_client(req)
	elif (sys.argv[1] == '-f'):
		print str(req)
		res = static_fact_query_client(req)
	print "Response: '"+str(res)+"'"
