__author__ = 'aishwarya'

from HISBeliefState import HISBeliefState
from Partition import Partition
from Utterance import Utterance
from SystemAction import SystemAction

if __name__ == '__main__' :
    #knowledge = Knowledge()
    #b = HisBeliefState(knowledge)
    ##print 1, [str(p) for p in b.partitions], '\n'
    #m1 = SystemAction('confirm_action', 'searchroom', {'patient':'ray', 'location':'3512'})
    #m2 = SystemAction('repeat_goal')    
    #u1 = Utterance('searchroom', {'patient':'ray', 'location':'3512'})
    #u2 = Utterance(None, None, [Knowledge.yes])
    #u3 = Utterance(None, None, [Knowledge.no])
    #b.make_all_matching_partitions(m1, u2)
    ##for p in b.partitions :
        ##print str(p), '\n'
    #b.update_state(m1, [u1, u2, u3])

    #-------------------------------------------------------------------------------------

    #pa = ['searchroom', 'remind', 'askperson']
    #pv = dict()
    #pv['location'] = ['3502', '3414b']
    #pv['patient'] = ['peter', 'ray']
    #p = Partition(pa, pv)
    #knowledge = Knowledge()
    #print 1, [str(pp) for pp in p.split_by_goal('c', knowledge)], '\n'
    #l = p.split_by_goal('remind', knowledge)
    #print 2, [str(pp) for pp in l], '\n'
    #print 3, [str(pp) for pp in l[0].split_by_param('abc', '3502', knowledge)], '\n'
    #print 4, [str(pp) for pp in l[0].split_by_param('location', 'xyz', knowledge)], '\n'
    #print 5, [str(pp) for pp in l[0].split_by_param('location', '3502', knowledge)], '\n'
    
    #q = Partition(pa, pv, 0)
    #r = Partition(pa, pv, 1)
    #s = Partition([], pv, 0)
    #t = Partition(pa, dict(), 0)
    #print str(p)
    #print p == q
    #print p == r
    #print p == s
    #print p == t        
    #print "\n\n"
    
    #m1 = SystemAction('confirm_action', 'searchroom', {'patient':'ray', 'location':'l3512'})
    #m2 = SystemAction('repeat_goal')    
    #u1 = Utterance('searchroom', {'patient':'ray', 'location':'l3512'})
    #u2 = Utterance(None, None, [Knowledge.yes])
    #u3 = Utterance(None, None, [Knowledge.no])
    #p = [0,0,0,0,0,0]
    #p[0] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})
    #p[1] = Partition(['searchroom'], {'patient':[], 'location':[]})    
    #p[2] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512']})    
    #p[3] = Partition(['searchroom', 'speak_t'], {'patient':['ray'], 'location':['l3512']})    
    #p[4] = Partition(['searchroom'], {'patient':['ray', 'peter'], 'location':['l3512']})    
    #p[5] = Partition(['searchroom'], {'patient':['ray'], 'location':['l3512', 'l3416']})        
    
    #for pp in p :
      #print '\n'.join([str(pp), str(m1), str(u2)])
      #print pp.match(m1, u2)
      #print '\n'
      #print '\n'.join([str(pp), str(m1), str(u3)])
      #print pp.match(m1, u3)
      #print '\n'
      #print '\n'.join([str(pp), str(m2), str(u1)])
      #print pp.match(m1, u2)
      #print '\n'
        
    #-------------------------------------------------------------------------------------
    
    #u = [0,0,0,0,0,0,0,0,0,0]
    #u[0] = Utterance('affirm', 'searchroom', {'patient':'ray', 'location':'3512'})
    #u[1] = Utterance('affirm', 'searchroom', {'patient':'ray', 'location':'3512'})
    #u[2] = Utterance('deny', 'searchroom', {'patient':'ray', 'location':'3512'})
    #u[3] = Utterance('inform', 'searchroom', {'patient':'ray', 'location':'3512'})
    #u[4] = Utterance('affirm', 'searchroom', {'patient':'ray'})
    #u[5] = Utterance('affirm', 'searchroom', {'patient':'ray', 'recipient':'peter'})
    #u[6] = Utterance('affirm', 'searchroom', {})
    #u[7] = Utterance('inform', 'searchroom')
    #u[8] = Utterance('affirm', 'remind', {'patient':'ray', 'location':'3512'})
    #print str(u[0])
    #for i in xrange(1,9) :
        #print u[0] == u[i]
    
    u = [0,0,0]
    u[0] = Utterance('inform', 'searchroom', {'patient':'ray', 'location':'3512'})
    u[1] = Utterance('affirm')
    u[2] = Utterance('deny')
    
    p = [0,0,0,0,0,0,0]
    p[0] = Partition(['searchroom'], {'patient':['ray'], 'location':['3512', '3416']})        
    p[1] = Partition(['remind', 'searchroom'], {'patient':['ray'], 'location':['3512', '3416']})        
    p[2] = Partition(['remind'], {'patient':['ray'], 'location':['3512', '3416']})        
    p[3] = Partition(['searchroom'], {'patient':['peter', 'ray'], 'location':['3512', '3416']})        
    p[4] = Partition(['searchroom'], {'patient':['peter'], 'location':['3512', '3416']})        
    p[5] = Partition(['searchroom'], {'patient':['peter', 'ray'], 'location':['3512']})        
    p[6] = Partition(['searchroom'], {'patient':['peter', 'ray'], 'location':['3416']})        
    
    m1 = SystemAction('confirm_action', 'searchroom', {'patient':'ray', 'location':'3512'})
    m2 = SystemAction('repeat_goal')
    
    for i in xrange(0, 3) :
        for j in xrange(0, 7) :
            print str(u[i])
            print str(p[j])
            print str(m1)
            print u[i].match(p[j], m1)
            
            print '\n-------------------------------\n'
            
            print str(u[i])
            print str(p[j])
            print str(m2)
            print u[i].match(p[j], m2)
            
            print '\n-------------------------------\n'
            
    #---------------------------------------------------------------------
    
    #n = 'confirm_action'
    #g = 'searchRoom'
    #p = dict()
    #p['patient'] = 'ray'
    #p['location'] = 'l3512'
    #m = SystemAction(n, g, p)
    #print str(m)
    #m2 = SystemAction(n, '', p)
    #m3 = SystemAction(n, g, {'patient':'ray'})
    #m4 = SystemAction(n, g, {'patient':'ray', 'location':'l3512'})
    #m5 = SystemAction(n, g, {'patient':'ray', 'recipient':'peter'})    
    #print m == m2
    #print m == m3
    #print m == m4
    #print m == m5                

    
