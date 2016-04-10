import sys, shutil, pickle, re, csv 
from os import listdir
from os.path import isfile, join, isdir
sys.path.append('../')

path_to_batch = '/u/aish/Documents/Research/AMT_results/Batch3/'

def move_to_invalid(rejected_user_ids) :
    src_path = path_to_batch + 'completed/'
    dst_path = path_to_batch + 'rejected/'
    for user_id in rejected_user_ids :
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

def get_ids_with_codes() :
    path = path_to_batch + 'completed/codes/'
    files = [f for f in listdir(path) if isfile(join(path, f))]
    ids_with_codes = dict()
    for filename in files :
        file_handle = open(path + filename, 'r')
        line = file_handle.read()
        if len(line) > 0 :
            data = line.split(',')
            code = data[3]
            user_id = data[0]
            if len(user_id) > 0 :
                ids_with_codes[user_id] = code
    return ids_with_codes
    
def collate_results() :
    ids_with_codes = get_ids_with_codes()
    log_path = path_to_batch + 'completed/log/'
    log_text_path = path_to_batch + 'completed/log_text/'
    #log_path = path_to_batch + 'corrected_pickle_logs/'
    survey_path = path_to_batch + 'completed/surveys/'
    
    filename = path_to_batch + 'results.csv'
    #filename = path_to_batch + 'results_corrected.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    header = ["user_id", "code", "agent_type", "performed_action", "correct_action", "task_easy", "robot_understood", "robot_delayed", "asked_sensible_questions", "conversation_too_long", "dialog_length", "comment"]
    writer.writerow(header)
    
    for (user_id, code) in ids_with_codes.items() :
        log_file_handle = open(log_path + user_id + '_main.pkl', 'rb')
        log = pickle.load(log_file_handle)
        row = [user_id, code, log[0], int(log[2] != None), int(log[3])]
        survey_file_handle = open(survey_path + user_id + '.txt', 'r')
        lineno = 0
        comment = ''
        for line in survey_file_handle :
            line = line.strip()
            if lineno == 0 :
                row = row + line.strip().split(',')
            elif lineno == 1 :
                comment = line 
            else :
                comment = comment + line 
            lineno += 1
        
        log_text_handle = open(log_text_path + user_id + '_main.txt', 'r')
        log_text = log_text_handle.read()
        length = len(log_text.split('\n'))
        row = row + [length] 
        
        comment = re.sub('\n', ' ', comment)
        row = row + [comment] 
        writer.writerow(row)
        
    file_handle.close()

def collate_results_corrected() :
    ids_with_codes = get_ids_with_codes()
    log_text_path = path_to_batch + 'completed/log_text/'
    log_path = path_to_batch + 'corrected_pickle_logs/'
    survey_path = path_to_batch + 'completed/surveys/'
    
    filename = path_to_batch + 'results_corrected.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    header = ["user_id", "code", "agent_type", "performed_action", "correct_action", "task_easy", "robot_understood", "robot_delayed", "asked_sensible_questions", "conversation_too_long", "dialog_length", "comment"]
    writer.writerow(header)
    
    for (user_id, code) in ids_with_codes.items() :
        log_file_handle = open(log_path + user_id + '_main.pkl', 'rb')
        log = pickle.load(log_file_handle)
        row = [user_id, code, log[0], int(log[2] != None), int(log[3])]
        survey_file_handle = open(survey_path + user_id + '.txt', 'r')
        lineno = 0
        comment = ''
        for line in survey_file_handle :
            line = line.strip()
            if lineno == 0 :
                row = row + line.strip().split(',')
            elif lineno == 1 :
                comment = line 
            else :
                comment = comment + line 
            lineno += 1
        
        log_text_handle = open(log_text_path + user_id + '_main.txt', 'r')
        log_text = log_text_handle.read()
        length = len(log_text.split('\n'))
        row = row + [length] 
        
        comment = re.sub('\n', ' ', comment)
        row = row + [comment] 
        writer.writerow(row)
        
    file_handle.close()

    
def summarize_results_by_agent_type() :
    results_file = open(path_to_batch + 'results.csv', 'r')
    #results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    reader = csv.reader(results_file, delimiter=',')
    sums = dict()
    num_rows = dict()
    cols_to_ignore = header = ["user_id", "code", "agent_type", "comment"]
    header = reader.next()
    type_idx = header.index("agent_type")
    for row in reader :
        agent_type = row[type_idx]
        if agent_type not in sums :
            sums[agent_type] = dict()
        if agent_type not in num_rows :
            num_rows[agent_type] = 0
        num_rows[agent_type] = num_rows[agent_type] + 1
        for (idx, col) in enumerate(header) :
            if col not in cols_to_ignore :
                if col not in sums[agent_type] :
                    sums[agent_type][col] = 0
                sums[agent_type][col] = sums[agent_type][col] + int(row[idx])
    
    filename = path_to_batch + 'summary.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    write_header = ['agent_type'] + [col for col in header if col not in cols_to_ignore]
    writer.writerow(write_header)
    print write_header
    for agent_type in sums.keys() :
        row = [agent_type]
        for col in write_header[1:] :
            avg = float(sums[agent_type][col]) / num_rows[agent_type]
            avg = round(avg, 2)
            row.append(avg)
        print row
        writer.writerow(row)
    
    file_handle.close()

def summarize_results_by_agent_type_corrected() :
    results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    reader = csv.reader(results_file, delimiter=',')
    sums = dict()
    num_rows = dict()
    cols_to_ignore = header = ["user_id", "code", "agent_type", "comment"]
    header = reader.next()
    type_idx = header.index("agent_type")
    for row in reader :
        agent_type = row[type_idx]
        if agent_type not in sums :
            sums[agent_type] = dict()
        if agent_type not in num_rows :
            num_rows[agent_type] = 0
        num_rows[agent_type] = num_rows[agent_type] + 1
        for (idx, col) in enumerate(header) :
            if col not in cols_to_ignore :
                if col not in sums[agent_type] :
                    sums[agent_type][col] = 0
                sums[agent_type][col] = sums[agent_type][col] + int(row[idx])
    
    filename = path_to_batch + 'summary_corrected.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    write_header = ['agent_type'] + [col for col in header if col not in cols_to_ignore]
    writer.writerow(write_header)
    print write_header
    for agent_type in sums.keys() :
        row = [agent_type]
        for col in write_header[1:] :
            avg = float(sums[agent_type][col]) / num_rows[agent_type]
            avg = round(avg, 2)
            row.append(avg)
        print row
        writer.writerow(row)
    
    file_handle.close()


def compare_task_completion() :
    results = open(path_to_batch + 'results.csv', 'r')
    results_reader = csv.reader(results, delimiter=',')
    results_corrected = open(path_to_batch + 'results_corrected.csv', 'r')
    corrected_reader = csv.reader(results_corrected, delimiter=',')
    
    tc_path = path_to_batch + 'completed/target_commands/'
    ea_path = path_to_batch + 'completed/executed_actions/'
    
    header = results_reader.next()
    idx = header.index('correct_action')
    corrected_reader.next()

    user_ids_with_results = dict()
    filename = path_to_batch + 'mismatch.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    header = ['user_id', 'user_says', 'we_think', 'target_command', 'executed_action']
    writer.writerow(header)
    
    for row in results_reader :
        user_id = row[0]
        completed = row[idx]
        user_ids_with_results[user_id] = completed 
        
    for row in corrected_reader :
        user_id = row[0]
        completed = row[idx]
        if completed != user_ids_with_results[user_id] :
            tc = open(tc_path + user_id + '_main.txt', 'r').read().strip()
            ea = open(ea_path + user_id + '_main.txt', 'r').read().strip()
            row_to_write = [user_id, user_ids_with_results[user_id], completed, tc, ea]
            writer.writerow(row_to_write)
            
    file_handle.close()
    
if __name__ == '__main__' :
    rejected_user_ids = ['57053c209db61', '57053619b608', '5705fd9cb9fcb', '5705f87f0b3ab', '5706076b544b5', '5705343360d6a', '570538488b7f5']
    move_to_invalid(rejected_user_ids)
    collate_results()
    summarize_results_by_agent_type()
    collate_results_corrected()
    summarize_results_by_agent_type_corrected()
    #compare_task_completion()
