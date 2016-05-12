import sys, csv, shutil, os, pickle, re
from os import listdir
from os.path import isfile, join, isdir
sys.path.append('../')

path_to_batch = '/u/aish/Documents/Research/AMT_results/after_fluency/Batch10/'

people_map = {'alice' : 'stacy', 'bob' : 'jesse', 'carol' : 'shiqi', \
    'dave' : 'jivko', 'eve' : 'aishwarya', 'frannie' : 'scott', \
    'george' : 'rodolfo', 'mallory' : 'peter', 'peggy' : 'dana', \
    'walter' : 'ray'}

def get_valid_ids_without_codes() :
    log_path = path_to_batch + 'original/log/'
    validations_path = path_to_batch + 'original/validations/'
    surveys_path = path_to_batch + 'original/surveys/'
    target_commands_path = path_to_batch + 'original/target_commands/'
    
    # Start with surveys because only people who submitted a survey could
    # have gotten a code
    files = [f for f in listdir(surveys_path) if isfile(join(surveys_path, f))]
    valid_ids = dict()
    
    for filename in files :
        user_id = filename[:-4]
        
        # Check that they passed validation
        cmd_file = target_commands_path + user_id + '_main.txt'
        query_file = target_commands_path + user_id + '_query.txt'
        query_ans_file = validations_path + user_id + '_query.txt'
        if isfile(cmd_file) and isfile(query_file) and isfile(query_ans_file) :
            query = open(query_file).read().strip()
            ans = open(query_ans_file).read().strip()
            if query == ans :
                # Passed validation. Check that conversation log exists
                log_file = log_path + user_id + '_main.pkl'
                if isfile(log_file) :
                    valid_ids[user_id] = user_id
    
    return valid_ids    

# Hallucinates codes for user ids verified by other methods
# This was written to get statistics for after_fluency/Batch4 which ran 
# into some error. 
# Note that doing this *should* not compromise the validity of results 
# but we will not use them for any comparisons just in case
def create_fake_codes() :        
    valid_ids = get_valid_ids_without_codes()
    path = path_to_batch + 'completed/codes/'
    for user_id in valid_ids :
        f = open(path + user_id + '.txt', 'w')
        output = user_id + ',1,,' + user_id + ',0,1,2' 
        f.write(output)
        f.close()

# Get IDs of users who completed the task all the way and received a code
def get_ids_with_codes() :
    path = path_to_batch + 'original/codes/'
    files = [f for f in listdir(path) if isfile(join(path, f))]
    ids_with_codes = dict()
    for filename in files :
        #print 'Processing file ', filename
        file_handle = open(path + filename, 'r')
        line = file_handle.read()
        if len(line) > 0 :
            data = line.split(',')
            code = data[3]
            user_id = data[0]
            if len(user_id) > 0 :
                ids_with_codes[user_id] = code
    return ids_with_codes

# Separate users who went till the stage of getting a code and those who 
# didn't
def move_logs_with_codes(ids_with_codes) :
    #ids_with_codes = get_ids_with_codes()
    src_path = path_to_batch + 'invalid/'
    dst_path = path_to_batch + 'completed/'
    for user_id in ids_with_codes.keys() :
        if isfile(src_path + 'codes/' + user_id + '.txt') :
            shutil.move(src_path + 'codes/' + user_id + '.txt', dst_path + 'codes/' + user_id + '.txt')
        if isfile(src_path + 'error/' + user_id + '_main.txt') :
            shutil.move(src_path + 'error/' + user_id + '_main.txt', dst_path + 'error/' + user_id + '_main.txt')
        if isfile(src_path + 'executed_actions/' + user_id + '_main.txt') :
            shutil.move(src_path + 'executed_actions/' + user_id + '_main.txt', dst_path + 'executed_actions/' + user_id + '_main.txt')
        if isfile(src_path + 'log/' + user_id + '_main.pkl') :
            shutil.move(src_path + 'log/' + user_id + '_main.pkl', dst_path + 'log/' + user_id + '_main.pkl')
        if isfile(src_path + 'log_text/' + user_id + '_main.txt') :
            shutil.move(src_path + 'log_text/' + user_id + '_main.txt', dst_path + 'log_text/' + user_id + '_main.txt')
        if isfile(src_path + 'surveys/' + user_id + '.txt') :
            shutil.move(src_path + 'surveys/' + user_id + '.txt', dst_path + 'surveys/' + user_id + '.txt')
        if isfile(src_path + 'target_commands/' + user_id + '_main.txt') :
            shutil.move(src_path + 'target_commands/' + user_id + '_main.txt', dst_path + 'target_commands/' + user_id + '_main.txt')
        if isfile(src_path + 'target_commands/' + user_id + '_query.txt') :
            shutil.move(src_path + 'target_commands/' + user_id + '_query.txt', dst_path + 'target_commands/' + user_id + '_query.txt')
        if isfile(src_path + 'validations/' + user_id + '_query.txt') :
            shutil.move(src_path + 'validations/' + user_id + '_query.txt', dst_path + 'validations/' + user_id + '_query.txt')

# Get a list of IDs of users who completed the HIT and whose pickle log 
# was created 
def get_ids_having_pickle_logs(path=path_to_batch + 'completed/log/') :
    #path = path_to_batch + 'completed/log/'
    files = [f for f in listdir(path) if isfile(join(path, f))]
    ids_having_pickle_logs = [filename[:-9] for filename in files]
    return ids_having_pickle_logs   

# Some pickle logs did not get created. Collate information required to 
# recreate these    
def collate_files_for_reconstructing_pickle_logs() :
    ids_with_codes = get_ids_with_codes()
    ids_having_pickle_logs = get_ids_having_pickle_logs()
    src_path = path_to_batch + 'completed/'
    dst_path = path_to_batch + 'without_pickle_logs/'
    #dst_path = path_to_batch + 'with_pickle_logs/'
    for user_id in ids_with_codes :
        if user_id not in ids_having_pickle_logs :
        #if user_id not in ids_having_pickle_logs :
            os.mkdir(dst_path + user_id)
            if isfile(src_path + 'executed_actions/' + user_id + '_main.txt') :
                shutil.copy(src_path + 'executed_actions/' + user_id + '_main.txt', dst_path + user_id + '/executed_action.txt')
            shutil.copy(src_path + 'target_commands/' + user_id + '_main.txt', dst_path + user_id + '/target_command.txt')
            shutil.copy(src_path + 'log_text/' + user_id + '_main.txt', dst_path + user_id + '/log_text.txt')

# Assuming you have subfolders named by user_id each of which has a 
# log_text.txt file, check those files for the word "stop" and print 
# user ids where it is present        
def check_for_stops() :
    path = path_to_batch + 'with_pickle_logs/'
    dirs = [f for f in listdir(path) if isdir(join(path, f))]    
    for user_id in dirs :
        file_handle = open(path + user_id + '/log_text.txt', 'r')
        text = file_handle.read()
        if 'stop' in text :
            print user_id

def remap_target_command(fake_people_command) :
    parts = re.split('[(,)]', fake_people_command)
    if parts[0] == 'search' :
        parts[0] = 'searchroom'
        parts[1] = people_map[parts[1]]
        return parts[0] + '(' + parts[1] + ',' + parts[2] + ')'
    elif parts[0] == 'bring' :
        parts[2] = people_map[parts[2]]
        return parts[0] + '(' + parts[1] + ',' + parts[2] + ')'
    else :
        return fake_people_command
    

# Performs 2 clean-up operations on pickle logs -
#       Use ground truth to decide whether the task succeeded or failed
#       The state was being logged in the first turn. Deleting this. 
def check_and_correct_pickle_logs() :
    ids_having_pickle_logs = get_ids_having_pickle_logs()
    log_path = path_to_batch + 'completed/log/'
    tc_path = path_to_batch + 'completed/target_commands/'
    ea_path = path_to_batch + 'completed/executed_actions/'
    dst_path = path_to_batch + 'corrected_pickle_logs/'
    
    #log_path = path_to_batch + 'invalid/log/'
    #ids_having_pickle_logs = get_ids_having_pickle_logs(log_path)
    #tc_path = path_to_batch + 'invalid/target_commands/'
    #ea_path = path_to_batch + 'invalid/executed_actions/'
    #dst_path = path_to_batch + 'all_corrected_pickle_logs/'
    
    for user_id in ids_having_pickle_logs :
        log_file = open(log_path + user_id + '_main.pkl', 'rb')
        log = pickle.load(log_file)
        
        conv = log[1]
        #print 'conv = ', conv
        if conv is not None and len(conv) > 0 and len(conv[0]) == 3 :
            conv[0] = conv[0][1:]
        
        tc_file = open(tc_path + user_id + '_main.txt', 'r')
        target_command = tc_file.read().strip()
        task_successful = True 
        action = log[2]
        if isfile(ea_path + user_id + '_main.txt') :
            ea_file = open(ea_path + user_id + '_main.txt', 'r')
            executed_action = ea_file.read().strip()
            if remap_target_command(target_command) != executed_action :
                print 'target_command = ', target_command
                print 'executed_action = ', executed_action
                print
                task_successful = False
        else :
            action = None 
            task_successful = False
            
        new_log = (log[0],  conv, action, task_successful, log[4])
        new_log_file = open(dst_path + user_id + '_main.pkl', 'wb')
        pickle.dump(new_log, new_log_file, pickle.HIGHEST_PROTOCOL)

def organize_text_logs() :
    results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    #results_file = open(path_to_batch + 'results.csv', 'r')
    results_reader = csv.reader(results_file, delimiter=',')
    results_reader.next()
    src_path = path_to_batch + 'completed/log_text/'
    for row in results_reader :
        user_id = row[0]
        agent_type = row[2]
        success = bool(int(row[5]))
        dst_path = path_to_batch + 'categorized_text_logs/'
        if success :
            dst_path = dst_path + 'success/'
        else :
            dst_path = dst_path + 'failure/'
        dst_path = dst_path + agent_type + '/'
        shutil.copy(src_path + user_id + '_main.txt', dst_path + user_id + '.txt')    
        
        

if __name__ == '__main__' :
    #valid_ids = get_valid_ids_without_codes()
    #print valid_ids
    #valid_ids = get_ids_with_codes()
    #move_logs_with_codes(valid_ids)
    #collate_files_for_reconstructing_pickle_logs()    
    #check_and_correct_pickle_logs()
    #create_fake_codes()
    organize_text_logs()
