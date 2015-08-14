import os,sys

class SemanticNode:

	def __init__(self, parent, type, category, is_lambda, idx=None, lambda_name=None, is_lambda_instantiation=None, children=None):
		self.parent = parent
		self.type = type
		self.category = category
		self.is_lambda = is_lambda
		if ((self.is_lambda and (lambda_name == None or is_lambda_instantiation == None)) or (not self.is_lambda and idx == None)): sys.exit("Invalid SemanticNode instantiation ("+str([self.is_lambda,lambda_name,is_lambda_instantiation,idx])+")")
		self.idx = idx
		self.lambda_name = lambda_name
		self.is_lambda_instantiation = is_lambda_instantiation
		self.children = children
		self.return_type = None

	#return own type if no children; output expected return value if children match; error out otherwise
	def setReturnType(self, ontology):
		if (self.is_lambda_instantiation == True): #lambdas 'return' <type,child_return_type> if consumed by upper functions
			type_str = ontology.composeStrFromType(self.type)
			child_return_type_str = ontology.composeStrFromType(self.children[0].return_type)
			self.return_type = ontology.readTypeFromStr("<"+type_str+","+child_return_type_str+">")
		elif (self.children == None): self.return_type = self.type #leaf returns itself
		else:
			candidate_type = self.type
			for c in self.children:
				if (c.return_type == None): c.setReturnType(ontology)
				if (ontology.types[candidate_type][0] == c.return_type): candidate_type = ontology.types[candidate_type][1]
				else: raise TypeError("Non-matching child type "+str(ontology.types[c.return_type])+" ("+c.printLittle()+") for parent "+str(ontology.types[candidate_type])+" ("+self.printLittle()+")")
			self.return_type = candidate_type

	#copy attributes of the given SemanticNode into this one (essentially, clone the second into this space)
	def copyAttributes(self, A, lambda_enumeration=0, preserve_parent=False, preserve_children=False):
		self.category = A.category
		self.type = A.type
		self.is_lambda = A.is_lambda
		self.idx = A.idx
		self.lambda_name = None if A.lambda_name == None else A.lambda_name+lambda_enumeration
		self.is_lambda_instantiation = A.is_lambda_instantiation
		if (preserve_parent == False): self.parent = A.parent
		if (preserve_children == False):
			if (A.children == None): self.children = None
			else:
				self.children = [SemanticNode(self, 0, 0, False, 0) for i in range(0,len(A.children))]
				for i in range(0,len(A.children)): self.children[i].copyAttributes(A.children[i],lambda_enumeration)
		self.return_type = A.return_type

	def printLittle(self):
		return "("+",".join([str(self.is_lambda),str(self.type),str(self.lambda_name) if self.is_lambda else str(self.idx)])+")"		

	def __str__(self):
		s = "("+",".join([str(self.is_lambda),str(self.type),str(self.lambda_name) if self.is_lambda else str(self.idx)])+")"
		if (self.children != None):
			for c in self.children:
				s += "\n"
				curr = self
				while (curr != None):
					s += "\t"
					curr = curr.parent
				s += str(c)
		return s

	#this is a forward comparison, so two trees are considered equal if their roots have different parents, as long as the roots and children down are identical
	#categories need not match since this is a test for semantic, not syntactic, equality
	def __eq__(self, other):
		if (type(self) != type(other)): return False #ie no casting
		if (self.type == other.type and self.is_lambda == other.is_lambda and self.idx == other.idx and self.lambda_name == other.lambda_name):
			if (self.children == None and other.children == None): return True
			if (self.children == None and other.children != None): return False
			if (self.children != None and other.children == None): return False
			if (len(self.children) == len(other.children)):
				for i in range(0,len(self.children)):
					if (not (self.children[i] == other.children[i])): return False
				return True
			else: return False
		else: return False
