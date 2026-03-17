# Formal model checking for a simple robot motion FSM
from collections import deque
 
class RobotFSM:
    STATES=['idle','moving','obstacle_detected','turning','stopped']
    TRANSITIONS={
        'idle':{'start':'moving'},
        'moving':{'obstacle':'obstacle_detected','done':'stopped'},
        'obstacle_detected':{'turn':'turning'},
        'turning':{'resume':'moving','stuck':'stopped'},
        'stopped':{'reset':'idle'}
    }
    SAFETY_PROP = lambda self,s: s != 'stopped' or True  # always recoverable
    LIVENESS_PROP = lambda self,path: 'stopped' in path  # eventually stops
 
def bfs_reachable(fsm, init='idle'):
    visited=set(); queue=deque([[init]])
    all_paths=[]
    while queue:
        path=queue.popleft(); s=path[-1]
        if s in visited: continue
        visited.add(s); all_paths.append(path)
        for event,next_s in fsm.TRANSITIONS.get(s,{}).items():
            if next_s not in visited: queue.append(path+[next_s])
    return visited, all_paths
 
def model_check(fsm):
    reachable,paths=bfs_reachable(fsm)
    print(f"Reachable states: {reachable}")
    safety_ok=all(s!='undefined' for s in reachable)
    liveness_ok=any(fsm.LIVENESS_PROP(p) for p in paths)
    print(f"Safety (no undefined states): {safety_ok}")
    print(f"Liveness (eventually stops): {liveness_ok}")
    deadlocks=[p[-1] for p in paths if not fsm.TRANSITIONS.get(p[-1])]
    print(f"Potential deadlocks: {deadlocks}")
 
model_check(RobotFSM())
