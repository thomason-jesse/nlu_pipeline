import sys, shutil, pickle, re, csv, math, itertools, copy 
from scipy import stats
from os import listdir
from os.path import isfile, join, isdir
sys.path.append('../')

path_to_batch = '/u/aish/Documents/Research/AMT_results/after_fluency/Batch3/'

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
    
    header = ["user_id", "code", "agent_type", "target_task", "performed_action", "correct_action", "task_easy", "robot_understood", "robot_delayed", "asked_sensible_questions", "conversation_too_long", "dialog_length", "comment"]
    writer.writerow(header)
    
    for (user_id, code) in ids_with_codes.items() :
        log_file_handle = open(log_path + user_id + '_main.pkl', 'rb')
        log = pickle.load(log_file_handle)
        
        tc_path = path_to_batch + 'completed/target_commands/'
        tc_file = open(tc_path + user_id + '_main.txt', 'r')
        target_command = tc_file.read().strip().split('(')[0]
        
        row = [user_id, code, log[0], target_command, int(log[2] != None), int(log[3])]
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
    
    header = ["user_id", "code", "agent_type", "target_task", "performed_action", "correct_action", "task_easy", "robot_understood", "robot_delayed", "asked_sensible_questions", "conversation_too_long", "dialog_length", "comment"]
    writer.writerow(header)
    
    for (user_id, code) in ids_with_codes.items() :
        log_file_handle = open(log_path + user_id + '_main.pkl', 'rb')
        log = pickle.load(log_file_handle)
        
        tc_path = path_to_batch + 'completed/target_commands/'
        tc_file = open(tc_path + user_id + '_main.txt', 'r')
        target_command = tc_file.read().strip().split('(')[0]
        
        row = [user_id, code, log[0], target_command, int(log[2] != None), int(log[3])]
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

    
def summarize_results_by_agent_type_and_goal() :
    results_file = open(path_to_batch + 'results.csv', 'r')
    #results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    reader = csv.reader(results_file, delimiter=',')
    sums = dict()
    num_rows = dict()
    cols_to_ignore = header = ["user_id", "code", "agent_type", "target_task", "comment"]
    header = reader.next()
    agent_idx = header.index("agent_type")
    target_idx = header.index("target_task")
    for row in reader :
        agent_type = row[agent_idx] + ':' + row[target_idx]
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
    
    filename = path_to_batch + 'summary_goal.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    write_header = ['agent_type'] + [col for col in header if col not in cols_to_ignore] + ['Number of dialogues']
    writer.writerow(write_header)
    print write_header
    for agent_type in sums.keys() :
        row = [agent_type]
        for col in write_header[1:len(write_header)-1] :
            avg = float(sums[agent_type][col]) / num_rows[agent_type]
            avg = round(avg, 2)
            row.append(avg)
        row.append(num_rows[agent_type])
        print row
        writer.writerow(row)
    
    file_handle.close()

def summarize_results_by_agent_type_and_goal_corrected() :
    results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    reader = csv.reader(results_file, delimiter=',')
    sums = dict()
    num_rows = dict()
    cols_to_ignore = header = ["user_id", "code", "agent_type", "target_task", "comment"]
    header = reader.next()
    agent_idx = header.index("agent_type")
    target_idx = header.index("target_task")
    for row in reader :
        agent_type = row[agent_idx] + ':' + row[target_idx]
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
    
    filename = path_to_batch + 'summary_goal_corrected.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    write_header = ['agent_type'] + [col for col in header if col not in cols_to_ignore] + ['Number of dialogues']
    writer.writerow(write_header)
    print write_header
    for agent_type in sums.keys() :
        row = [agent_type]
        for col in write_header[1:len(write_header)-1] :
            avg = float(sums[agent_type][col]) / num_rows[agent_type]
            avg = round(avg, 2)
            row.append(avg)
        row.append(num_rows[agent_type])
        print row
        writer.writerow(row)
    
    file_handle.close()

def get_statistical_significance() :
    results_file = open(path_to_batch + 'results.csv', 'r')
    results_reader = csv.reader(results_file, delimiter=',')
    results_header = results_reader.next()
    agent_type_idx = results_header.index('agent_type')
    
    summary_file = open(path_to_batch + 'summary.csv', 'r')
    summary_reader = csv.reader(summary_file, delimiter=',')
    summary_header = summary_reader.next()
    print 'summary_header = ', summary_header
    
    cols_to_evaluate = copy.deepcopy(summary_header)
    cols_to_evaluate.remove('agent_type')
    cols_to_evaluate.remove('performed_action')
    cols_to_evaluate.remove('Number of dialogues')
    print 'cols_to_evaluate = ', cols_to_evaluate
     
    means = dict()
    num_rows = dict()
    variances = dict()
    
    for row in summary_reader :
        agent_type = row[0]
        means[agent_type] = dict()
        for (col_num, col_val) in enumerate(row) :
            col_name = summary_header[col_num]
            if col_name in cols_to_evaluate :
                means[agent_type][col_name] = float(col_val)
            elif col_name == 'Number of dialogues' :
                num_rows[agent_type] = int(col_val)
    
    for row in results_reader :
        agent_type = row[agent_type_idx]
        variances[agent_type] = dict()
        for (col_num, col_val) in enumerate(row) :
            col_name = results_header[col_num]
            print 'col_name = ', col_name
            if col_name in cols_to_evaluate :
                if col_name not in variances[agent_type] :
                    variances[agent_type][col_name] = 0.0
                variances[agent_type][col_name] = variances[agent_type][col_name] + (float(col_val) - means[agent_type][col_name]) ** 2
    
    for agent_type in variances :
        for col_name in variances[agent_type] :
            print 'agent_type = ', agent_type, ', col_name = ', col_name
            variances[agent_type][col_name] = variances[agent_type][col_name] / num_rows[agent_type]
    
    agent_types = means.keys()            
    agent_pairs = list()
    for i in range(0, len(agent_types)-1) :
        agent_pairs = agent_pairs + list(itertools.product([agent_types[i]], agent_types[i+1:]))        
    print 'agent_pairs = ', agent_pairs
 
    filename = path_to_batch + 'statistical_significance.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    write_header = ['agent1', 'agent2', 'metric', 'mean1', 'mean2', 'significance', 'winner']
        
    for (agent1, agent2) in agent_pairs :
        for col_name in cols_to_evaluate :
            y1 = means[agent1][col_name]
            y2 = means[agent2][col_name]
            s1 = variances[agent1][col_name]
            s2 = variances[agent2][col_name]
            n1 = num_rows[agent1]
            n2 = num_rows[agent2]
            t = (y1 - y2) / math.sqrt(((s1 ** 2) / n1) + ((s2 ** 2) / n2))
            v = (((s1 ** 2) / n1) + ((s2 ** 2) / n2)) / (((((s1 ** 2) / n1) ** 2) / (n1 - 1)) + ((((s2 ** 2) / n2) ** 2) / (n2 - 1)))
            v = int(round(v))
            t_stat = stats.t.ppf(1-0.025, v)
            if abs(t) > t_stat :
                if y1 > y2 :
                    row = [agent1, agent2, col_name, y1, y2, 'significant', agent1]
                else :
                    row = [agent1, agent2, col_name, y1, y2, 'significant', agent2]
            else :
                trending_t_stat = stats.t.ppf(1-0.05, v)
                if abs(t) > trending_t_stat :
                    if y1 > y2 :
                        row = [agent1, agent2, col_name, y1, y2, 'trending', agent1]
                    else :
                        row = [agent1, agent2, col_name, y1, y2, 'trending', agent2]
                else :
                    row = [agent1, agent2, col_name, y1, y2, 'not significant']
            writer.writerow(row)
            
    file_handle.close()

def get_statistical_significance_corrected() :
    results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    results_reader = csv.reader(results_file, delimiter=',')
    results_header = results_reader.next()
    agent_type_idx = results_header.index('agent_type')
    
    summary_file = open(path_to_batch + 'summary_corrected.csv', 'r')
    summary_reader = csv.reader(summary_file, delimiter=',')
    summary_header = summary_reader.next()
    print 'summary_header = ', summary_header
    
    cols_to_evaluate = copy.deepcopy(summary_header)
    cols_to_evaluate.remove('agent_type')
    cols_to_evaluate.remove('performed_action')
    cols_to_evaluate.remove('Number of dialogues')
    print 'cols_to_evaluate = ', cols_to_evaluate
     
    means = dict()
    num_rows = dict()
    variances = dict()
    
    for row in summary_reader :
        agent_type = row[0]
        means[agent_type] = dict()
        for (col_num, col_val) in enumerate(row) :
            col_name = summary_header[col_num]
            if col_name in cols_to_evaluate :
                means[agent_type][col_name] = float(col_val)
            elif col_name == 'Number of dialogues' :
                num_rows[agent_type] = int(col_val)
    
    for row in results_reader :
        agent_type = row[agent_type_idx]
        variances[agent_type] = dict()
        for (col_num, col_val) in enumerate(row) :
            col_name = results_header[col_num]
            print 'col_name = ', col_name
            if col_name in cols_to_evaluate :
                if col_name not in variances[agent_type] :
                    variances[agent_type][col_name] = 0.0
                variances[agent_type][col_name] = variances[agent_type][col_name] + (float(col_val) - means[agent_type][col_name]) ** 2
    
    for agent_type in variances :
        for col_name in variances[agent_type] :
            print 'agent_type = ', agent_type, ', col_name = ', col_name
            variances[agent_type][col_name] = variances[agent_type][col_name] / num_rows[agent_type]
    
    agent_types = means.keys()            
    agent_pairs = list()
    for i in range(0, len(agent_types)-1) :
        agent_pairs = agent_pairs + list(itertools.product([agent_types[i]], agent_types[i+1:]))        
    print 'agent_pairs = ', agent_pairs
 
    filename = path_to_batch + 'statistical_significance_corrected.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    write_header = ['agent1', 'agent2', 'metric', 'mean1', 'mean2', 'significance', 'winner']
        
    for (agent1, agent2) in agent_pairs :
        for col_name in cols_to_evaluate :
            y1 = means[agent1][col_name]
            y2 = means[agent2][col_name]
            s1 = variances[agent1][col_name]
            s2 = variances[agent2][col_name]
            n1 = num_rows[agent1]
            n2 = num_rows[agent2]
            t = (y1 - y2) / math.sqrt(((s1 ** 2) / n1) + ((s2 ** 2) / n2))
            v = (((s1 ** 2) / n1) + ((s2 ** 2) / n2)) / (((((s1 ** 2) / n1) ** 2) / (n1 - 1)) + ((((s2 ** 2) / n2) ** 2) / (n2 - 1)))
            v = int(round(v))
            t_stat = stats.t.ppf(1-0.025, v)
            if abs(t) > t_stat :
                if y1 > y2 :
                    row = [agent1, agent2, col_name, y1, y2, 'significant', agent1]
                else :
                    row = [agent1, agent2, col_name, y1, y2, 'significant', agent2]
            else :
                trending_t_stat = stats.t.ppf(1-0.05, v)
                if abs(t) > trending_t_stat :
                    if y1 > y2 :
                        row = [agent1, agent2, col_name, y1, y2, 'trending', agent1]
                    else :
                        row = [agent1, agent2, col_name, y1, y2, 'trending', agent2]
                else :
                    row = [agent1, agent2, col_name, y1, y2, 'not significant']
            writer.writerow(row)
            
    file_handle.close()

def summarize_results_by_agent_type() :
    results_file = open(path_to_batch + 'results.csv', 'r')
    #results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    reader = csv.reader(results_file, delimiter=',')
    sums = dict()
    num_rows = dict()
    cols_to_ignore = header = ["user_id", "code", "agent_type", "target_task", "comment"]
    header = reader.next()
    agent_idx = header.index("agent_type")
    target_idx = header.index("target_task")
    for row in reader :
        agent_type = row[agent_idx]
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
    
    write_header = ['agent_type'] + [col for col in header if col not in cols_to_ignore] + ['Number of dialogues']
    writer.writerow(write_header)
    print write_header
    for agent_type in sums.keys() :
        row = [agent_type]
        for col in write_header[1:len(write_header)-1] :
            avg = float(sums[agent_type][col]) / num_rows[agent_type]
            avg = round(avg, 2)
            row.append(avg)
        row.append(num_rows[agent_type])
        print row
        writer.writerow(row)
    
    file_handle.close()

def summarize_results_by_agent_type_corrected() :
    results_file = open(path_to_batch + 'results_corrected.csv', 'r')
    reader = csv.reader(results_file, delimiter=',')
    sums = dict()
    num_rows = dict()
    cols_to_ignore = header = ["user_id", "code", "agent_type", "target_task", "comment"]
    header = reader.next()
    agent_idx = header.index("agent_type")
    target_idx = header.index("target_task")
    for row in reader :
        agent_type = row[agent_idx]
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
    
    write_header = ['agent_type'] + [col for col in header if col not in cols_to_ignore] + ['Number of dialogues']
    writer.writerow(write_header)
    print write_header
    for agent_type in sums.keys() :
        row = [agent_type]
        for col in write_header[1:len(write_header)-1] :
            avg = float(sums[agent_type][col]) / num_rows[agent_type]
            avg = round(avg, 2)
            row.append(avg)
        row.append(num_rows[agent_type])
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
    #rejected_user_ids = ['57053c209db61', '57053619b608', '5705fd9cb9fcb', '5705f87f0b3ab', '5706076b544b5', '5705343360d6a', '570538488b7f5']
    #move_to_invalid(rejected_user_ids)
    collate_results()
    summarize_results_by_agent_type()
    summarize_results_by_agent_type_and_goal()
    collate_results_corrected()
    summarize_results_by_agent_type_corrected()
    summarize_results_by_agent_type_and_goal_corrected()
    get_statistical_significance()
    get_statistical_significance_corrected()
    #compare_task_completion()
