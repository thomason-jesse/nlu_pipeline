import sys

class FeatureExtractor:

	def __init__(self, ont, lex):
		self.ontology = ont
		self.lexicon = lex

	def incrementDictionary(self, d, idx, inc=1):
		dr = d
		for i in range(0,len(idx)):
			if (idx[i] not in dr):
				if (i == len(idx)-1): dr[idx[i]] = 0
				else: dr[idx[i]] = {}
			if (i == len(idx)-1): dr[idx[i]] += inc
			else: dr = dr[idx[i]]

	def readParseStructureToTokenAssignments(self, t, ta, ps):
		if (len(ps) == 2 and type(ps[1]) is str): ta[t.index(ps[1])] = ps[0]
		else:
			for psc in ps: self.readParseStructureToTokenAssignments(t,ta,psc)
	def extractFeatureMap(self, tokens, partial, parse_structure, denotation):
                f_map = {}

		#count features related to number of trees in partial and presence of None assignments
		sf = {}
		if (len(tokens) != 0):
			sf['trees_per_token'] = len(parse_structure)/float(len(tokens))
                	sf['prop_trees'] = 1.0/len(parse_structure)
			token_assignments = [None for i in range(0,len(tokens))]
			self.readParseStructureToTokenAssignments(tokens, token_assignments, parse_structure)
                	sf['None_per_token'] = sum([1 if p == None else 0 for p in token_assignments])/float(len(tokens))
		f_map['sf'] = sf

		#count of (token,lexical_entry) chosen at leaf level for ambiguous tokens
		#TODO

		#count of (rule,syntax,syntax) combinations used to form surface semantics
		#TODO

		token_surface_pred_co = {}
		denotation_card = {}
		lambda_card = {}
		lambda_type = {}
		den_type = {}
		for t in tokens:
			if (t not in self.lexicon.surface_forms): #add this word if we haven't seen it
				self.lexicon.surface_forms.append(t)
				vocab_idx = len(self.lexicon.surface_forms)-1
				self.lexicon.entries[vocab_idx] = []
			else: vocab_idx = self.lexicon.surface_forms.index(t)

			#count (token,predicate) pairs between tokens and surface semantic form
			to_examine = [p]
			while (len(to_examine) > 0):
				curr = to_examine.pop()
				if (curr == None):
					self.incrementDictionary(token_surface_pred_co, [vocab_idx,'None'])
				elif (type(curr) is int):
					self.incrementDictionary(token_surface_pred_co,[vocab_idx,curr])						
				elif (type(curr) is list and type(curr[0]) is int):
					self.incrementDictionary(token_surface_pred_co,[vocab_idx,curr[0]])
				else:
					if  (curr.is_lambda == False):
						self.incrementDictionary(token_surface_pred_co,[vocab_idx,curr.idx])
					if (curr.children != None): to_examine.extend(curr.children)

			#count of (token,|lambda|),(token,type(lambda)),(token,|dens|),(token,type(dens))
			if (len(denotation) > 0):
				dl = float(len(denotation))
				dltl = sum([float(len(d[0])) for d in denotation])
				self.incrementDictionary(denotation_card,[vocab_idx],inc=1/dl)
				for d in denotation:
					self.incrementDictionary(lambda_card,[vocab_idx],inc=len(d[0])/dl)
					for i in range(0,len(d[0])):				
						self.incrementDictionary(lambda_type,[vocab_idx,self.ontology.entries[self.ontology.preds.index(d[0][i])]],inc=1/dltl)
						self.incrementDictionary(den_type,[vocab_idx,str(d[1])],inc=1/dl)

		f_map['token_surface_pred_co'] = token_surface_pred_co
		f_map['token_denotation_card'] = denotation_card
		f_map['token_lambda_card'] = lambda_card
		f_map['token_lambda_type'] = lambda_type
		f_map['token_den_type'] = den_type

                return f_map

