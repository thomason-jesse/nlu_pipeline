import os,sys

class Ontology:

	#initializes an ontology data structure which reads in atoms and predicates from given files
	def __init__(self, ont_fname):

		self.types = ['t','e','d','a','c'] #subsequent entries are tuples of indices into this list defining a binary hierarchy
		self.preds,self.entries = self.readSemFromFile(ont_fname)
		self.num_args = [self.calcNumPredArgs(i) for i in range(0,len(self.preds))]

	#calculate the number of arguments a predicate takes
	def calcNumPredArgs(self, idx):
		num_args = 0
		curr_type = self.types[self.entries[idx]]
		while (len(curr_type) == 2):
			num_args += 1
			curr_type = self.types[curr_type[1]]
		return num_args

	#reads semantic atom/predicate declarations from a given file of format:
	#atom_name:type
	#pred_name:<complex_type>
	def readSemFromFile(self, fname):

		preds = []
		entries = {} #map of pred_idx:type read in
		f = open(fname,'r')
		for line in f.readlines():
			
			#ignore blank lines and comments
			line = line.strip()
			if (len(line) == 0 or line[0] == '#'):
				continue

			#create semantic meaning representation from string
			[name,type_str] = line.split(':')
			if (name in preds): sys.exit("Multiply defined type for predicate '"+name+"'")
			entries[len(preds)] = self.readTypeFromStr(type_str)
			preds.append(name)
		f.close()
		return preds,entries

	#returns the index of self.types at which this type is stored; adds types to this list as necessary to compose such a type
	def readTypeFromStr(self, str):

		#a complex type
		if (str[0] == "<" and str[-1] == ">"):
			d = 0
			for split_idx in range(1,len(str)-1):
				if (str[split_idx] == '<'): d += 1
				elif (str[split_idx] == '>'): d -= 1
				elif (str[split_idx] == ',' and d == 0): break
			comp_type = [self.readTypeFromStr(str[1:split_idx]),self.readTypeFromStr(str[split_idx+1:-1])]
			try:
				return self.types.index(comp_type)
			except ValueError:
				self.types.append(comp_type)
				return len(self.types)-1
		
		#a primitive type
		else:
			try:
				return self.types.index(str)
			except ValueError: sys.exit("Unrecognized primitive type '"+str+"'")

	#returns a string representing the given ontological type
	def composeStrFromType(self, t):
		s = ''
		#a complex type
		if (type(self.types[t]) is not str): s += '<'+self.composeStrFromType(self.types[t][0])+','+self.composeStrFromType(self.types[t][1])+'>'
		#a primitive type
		else: s += self.types[t]
		return s
