# Storing some earlier code that identified new lexical entries

def train_parser_from_dialogue(self, final_action) :
    print 'Lexicon - ', self.parser.lexicon.surface_forms
    
    print 'self.parser_train_data = ', self.parser_train_data
    
    answers = dict()
    answers['full'] = str(final_action)
    goal = final_action.name
    for (idx, param_name) in enumerate(self.knowledge.param_order[goal]) :
        answers[param_name] = final_action.params[idx]
    
    print 'answers = ', answers
    
    new_lexical_entries = list()
    
    for key in self.parser_train_data :
        if key != 'full' :
            # This is a param value. It may be a multi-word expression
            # present in the lexicon
            for value in self.parser_train_data[key] :
                if len(self.parser.lexicon.get_semantic_forms_for_surface_form(value)) > 0 :
                    # Known expression. No need for new lexical entry
                    continue
                else :
                    # If the first word is a, an, the, see if the rest
                    # of it is present in the lexicon
                    tokens = self.parser.tokenize(value)
                    if tokens[0] == 'a' or tokens[0] == 'an' or tokens[0] == 'the' :
                        rest_of_it = ' '.join(tokens[1:])
                        if len(self.parser.lexicon.get_semantic_forms_for_surface_form(rest_of_it)) > 0 :
                            continue
                        else :
                            del tokens[0]
                    
                    # If it is of the form x's office and x is known 
                    # again it can be ignored
                    if tokens[-1] == 'office' :
                        rest_of_it = ' '.join(tokens[:-2])
                        if len(self.parser.lexicon.get_semantic_forms_for_surface_form(rest_of_it)) > 0 :
                            continue
                        # Else you'd ideally want to use the knowledge 
                        # base to find who's office is the room you're 
                        # talking about and hence create an entry for x
                    
                    # If it has too many words it is unlikely to be 
                    # a multi-word expression to be learnt as such
                    if len(tokens) > 3 :
                        continue
                
                    surface_form = ' '.join(tokens)
                    if key in answers :
                        semantic_form = answers[key]
                        entry = surface_form + ' :- N : ' + semantic_form
                        print 'Creating entry ', entry 
                        new_lexical_entries.append(entry)
        
        else :    
            for value in self.parser_train_data[key] :            
                tokens = self.parser.tokenize(value)
                unknown_surface_forms = [(idx, token) for (idx, token) in enumerate(tokens) if len(self.parser.lexicon.get_semantic_forms_for_surface_form(token)) == 0]
                non_multi_word_expressions = list()
                possible_matchers = tokens 
                for (idx, token) in unknown_surface_forms :
                    # Bad heuristic to check for multi-word expressions
                    possible_multi_word_tokens = []
                    if idx >= 1 :
                        possible_multi_word_tokens.append(tokens[idx-1:idx+1])
                    if idx >= 2 :
                        possible_multi_word_tokens.append(tokens[idx-2:idx+1])
                    if idx >= 3 :
                        possible_multi_word_tokens.append(tokens[idx-3:idx+1])
                    if idx <= len(tokens) - 2 :
                        possible_multi_word_tokens.append(tokens[idx:idx+1])
                    if idx <= len(tokens) - 3 :
                        possible_multi_word_tokens.append(tokens[idx:idx+2])
                    if idx <= len(tokens) - 4 :
                        possible_multi_word_tokens.append(tokens[idx:idx+3])
                    match_found = False
                    for multi_word_token in possible_multi_word_tokens :
                        text = ' '.join(multi_word_token)
                        if len(self.parser.lexicon.get_semantic_forms_for_surface_form(text)) > 0 :
                            possible_matchers.append(text)
                            match_found = True
                            break
                    if not match_found :
                        non_multi_word_expressions.append(token)
                
                if len(non_multi_word_expressions) == 1 :
                    # Try to create lexical entries only if exactly one
                    # unknown surface form
                    unknown_surface_form = non_multi_word_expressions[0]
                    possible_matches = final_action.params
                    if unknown_surface_form + '\'s office' not in value :
                        # We don't really know how to do the whole office
                        # thing yet
                        for matcher in possible_matchers :
                            semantic_indices = self.parser.lexicon.get_semantic_forms_for_surface_form(matcher)
                            for idx in semantic_indices :
                                semantic_form = self.parser.lexicon.semantic_forms[idx]
                                if semantic_form.idx is not None :
                                    ont_value = self.parser.lexicon.ontology.preds[semantic_form.idx]
                                    if ont_value in possible_matches :
                                        possible_matches.remove(ont_value)
                        print 'possible_matches = ', possible_matches
                        if len(possible_matches) == 1 :
                            # Only add an extry if it appears unambiguous
                            entry = unknown_surface_form + ' :- N : ' + possible_matches[0]
                            print 'Creating entry from sentence ', entry
                            new_lexical_entries.append(entry)
                         
    print 'new_lexical_entries = ', new_lexical_entries                
    #print 'Retraining parser'
    
    # Add new entries to lexicon
    self.parser.lexicon.expand_lex_from_strs(
                new_lexical_entries, self.parser.lexicon.surface_forms, self.parser.lexicon.semantic_forms, 
                self.parser.lexicon.entries, self.parser.lexicon.pred_to_surface, 
                allow_expanding_ont=False)
                
    # Log the added lexicon entries   
    if self.lexical_addition_log is not None :
        f = open(self.lexical_addition_log, 'a')
        for entry in new_lexical_entries :
            f.write(entry + '\n')
        f.close()
                
    training_pairs = list()
    for key in self.parser_train_data :
        if key in answers :
            for item in self.parser_train_data[key] :
                training_pairs.append((item, answers[key]))

    print 'training_pairs = ', training_pairs

    self.parser.train_learner_on_denotations(training_pairs, 10, 100, 3)

    print 'Finished retraining parser'

    self.parser_train_data = dict()
    save_model(self.parser, 'static_parser')
