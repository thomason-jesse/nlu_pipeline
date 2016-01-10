__author__ = 'aishwarya'

# This is a simple generator based on templates that gives sensible 
# sentences given the system action, goal and arguments

class TemplateBasedGenerator :
    
    def __init__(self) :
        pass
    
    def get_param(self, system_action, param_name) :
        if system_action.referring_params is None :
            return None
        if param_name in system_action.referring_params :
            return system_action.referring_params[param_name]
        else :
            return None
    
    def get_action_sentence(self, action) :
        if action.name == 'searchroom' :
            return 'The robot searched for ' + str(action.params[0]) + ' in room ' + str(action.params[1]) + '.'
        elif action.name == 'bring' :
            return 'The robot brought ' + str(action.params[0]) + ' to ' + str(action.params[1]) + '.'
        elif action.name == 'at' :
            return 'The robot went to ' + str(action.params[0]) + '.'
        else :
            if len(action.params) == 0 :
                return 'The robot took action ' + action.name + '.'
            elif len(action.params) == 1 :
                return 'The robot took action ' + action.name + ' with parameter ' + str(action.params[0]) + '.'
            else :
                params = [str(param) for param in action.params]
                return 'The robot took action ' + action.name + ' with parameters ' + ', '.join(params[:-1]) + ' and ' + params[-1] + '.'
            
    
    def get_sentence(self, system_action) :
        if system_action.action_type == 'repeat_goal' :
            if system_action.extra_data is not None and 'first' in system_action.extra_data :
                return 'How can I help?'
            else :
                return 'I\'m sorry but could you clarify again what you wanted me to do?'
                
        elif system_action.action_type == 'confirm_action' :
            if system_action.referring_goal is None :
                params = [str(param) for param in system_action.referring_params.values()]
                if len(params) == 0 :
                    return 'I\'m sorry, I didn\'t quite understand that. What did you want me to do?'
                elif len(params) == 1 :
                    return 'You want me to take an action with ' + params[0] + '?'
                else :
                    return 'You want me to take an action with ' + ', '.join(params[:-1]) + ' and ' + params[-1] + '?'
            elif system_action.referring_goal == 'searchroom' :
                patient = self.get_param(system_action, 'patient')
                location = self.get_param(system_action, 'location')
                if patient is not None and location is not None :
                    return 'You want me to search for ' + patient + ' in room ' + location + '?'
                elif patient is not None :
                    return 'You want me to search for ' + patient + ' in some room?'
                elif location is not None :
                    return 'You want me to search for someone in room ' + location + '?'
                else :    
                    return 'You want me to search for someone?'
            elif system_action.referring_goal == 'bring' :
                patient = self.get_param(system_action, 'patient')
                recipient = self.get_param(system_action, 'recipient')
                if patient is not None and recipient is not None :
                    return 'You want me to bring ' + patient + ' for ' + recipient + '?'
                elif patient is not None :
                    return 'You want me to bring ' + patient + ' for someone?'
                elif recipient is not None :
                    return 'You want me to bring something for ' + recipient + '?'
                else :    
                    return 'You want me to bring something for someone?'
            elif system_action.referring_goal == 'at' :
                location = self.get_param(system_action, 'location')
                if location is not None :
                    return 'You want me to walk to room ' + location + '?'
                else :    
                    return 'You want me to walk to somewhere?'
            else :
                goal = system_action.referring_goal
                params = [str(param) for param in system_action.referring_params.values()]
                if len(params) == 0 :
                    return 'You want me to take action ' + goal + '?'
                elif len(params) == 1 :
                    return 'You want me to take action ' + goal + ' with ' + params[0]
                else :
                    return 'You want me to take action ' + goal + ' with ' + ', '.join(params[:-1]) + ' and ' + params[-1]
        
        elif system_action.action_type == 'request_missing_param' :
            param_to_confirm = system_action.extra_data[0]
            if system_action.referring_goal == 'searchroom' :
                if param_to_confirm == 'location' :
                    return 'Where would you like me to search?'
                else :
                    return 'Whom would you like me to search for?'
            elif system_action.referring_goal == 'bring' :
                if param_to_confirm == 'patient' :
                    return 'What would you like me to bring?'
                else :
                    patient = self.get_param(system_action, 'patient')
                    if patient is not None :
                        return 'Whom would you like me to bring ' + patient + ' to?'
                    else :
                        return 'Whom would you like me to bring something to?'
            elif system_action.referring_goal == 'at' :
                return 'Where would you like me to go?'    
            else :
                return 'What is the ' + param_to_confirm + ' of the action you would like me to take?'
                
        else :
            return 'Error! Wrong query to generator.'            
