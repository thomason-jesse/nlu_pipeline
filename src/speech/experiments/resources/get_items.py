item_file = open('ont.txt', 'r')
new_file = open('ontNew.txt', 'w')

in_items = False
in_adj = False

def processLine(line, processing_type):
    words = line.strip().split(':')
    
    if processing_type == 'a':
        result = words[0].strip() + ':<e,t>' + '\n'
    elif processing_type == 'i':
        result = words[0].strip() + ' :- NP : ' + words[0].strip() + '\n'
    
    return result

for line in item_file:
    if line.strip() == "<I>":
        in_items = True
    elif line.strip() == "<A>" or line.strip() == "<NO>":
        in_adj = True
    elif line.strip() == "</C>":
        in_items = False
        in_adj = False
    else:
        if in_items:
            new_file.write(processLine(line, 'i'))
        elif in_adj:
            new_file.write(processLine(line, 'a'))
        else:
            new_file.write(line)
        
            
