import sys, csv
from os import listdir
from os.path import isfile, join

path_to_batch = '/u/aish/Documents/Research/AMT_results/after_fluency/Batch9/'

def load_submitted_codes() :
    filename = path_to_batch + 'amt.csv'
    file_handle = open(filename, 'r')
    reader = csv.reader(file_handle, delimiter=',')
    submitted_codes = dict()
    repeated_codes = dict()
    reader.next()
    for row in reader :
        code = row[27]
        if code in submitted_codes :
            users = [(submitted_codes[code][14], submitted_codes[code][15]), (row[14], row[15])]
            if code in repeated_codes :
                users = users + repeated_codes[code]
            repeated_codes[code] = users
        else :
            submitted_codes[code] = row
    print 'In submission, repeated codes = '
    print repeated_codes
    return (submitted_codes, repeated_codes)
        
def load_generated_codes() :
    path = path_to_batch + 'completed/codes/'
    files = [f for f in listdir(path) if isfile(join(path, f))]
    generated_codes = dict()
    repeated_codes = dict()
    for filename in files :
        #print 'Processing file ', filename
        file_handle = open(path + filename, 'r')
        line = file_handle.read()
        if len(line) > 0 :
            data = line.split(',')
        else :
            data = [file_handle,0,'',-1,-1,-1]
        #print data
        code = data[3]
        if code in generated_codes :
            users = [generated_codes[code][0], data[0]]
            if code in repeated_codes :
                users = users + repeated_codes[code]
            repeated_codes[code] = users
        else :
           generated_codes[code] = data
    print 'In generation, repeated codes = '
    print repeated_codes
    return generated_codes
           
def combine_stats() :
    (submitted_codes, repeated_codes) = load_submitted_codes()
    generated_codes = load_generated_codes()
    
    filename = path_to_batch + 'combined.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    unmatched_codes = list()
    for code in submitted_codes.keys() :
        if code in repeated_codes :
            if code in generated_codes :
                print 'Repeated code ', code, ' in ', generated_codes[code]
            else :
                print 'Repeated code ', code, ' not generated'
        if code not in generated_codes or code in repeated_codes :
            unmatched_codes.append(code)
        else :
            new_row = submitted_codes[code] + generated_codes[code]
            writer.writerow(new_row)
    print 'unmatched_codes = '
    print unmatched_codes
    
def approve_reject() :
    (submitted_codes, repeated_codes) = load_submitted_codes()
    generated_codes = load_generated_codes()
    
    filename = path_to_batch + 'approve_reject.csv'
    file_handle = open(filename, 'w')
    writer = csv.writer(file_handle, delimiter=',')
    
    for code in submitted_codes.keys() :
        if code not in generated_codes or code in repeated_codes :
            new_row = submitted_codes[code] + ['', 'Incorrect/invalid code entered']
        else :
            walk_sel = generated_codes[code][4]
            bring_sel = generated_codes[code][5]
            search_sel = generated_codes[code][6]
            if walk_sel != '0' or bring_sel != '1' or search_sel != '2' :
                user_id = generated_codes[code][0]
                print 'Incorrect understanding: ', user_id, code
                new_row = submitted_codes[code] + ['', 'Worker does not seem to have understood the task']    
            else :
                if generated_codes[code][1] == '1' :
                    new_row = submitted_codes[code] + ['x', '']
                else :
                    user_id = generated_codes[code][0]
                    print 'Wrong query answer = ', user_id, code 
                    new_row = submitted_codes[code] + ['', generated_codes[code][2]]
            writer.writerow(new_row)

if __name__ == '__main__' :
    approve_reject()
