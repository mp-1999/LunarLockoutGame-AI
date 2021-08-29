#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
    return 0

def heur_manhattan_distance(state):
#OPTIONAL
    '''Manhattan distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses Manhattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the Manhattan distances between each xanadu and the escape hatch.
    return 0

def heur_L_distance(state):
    #IMPLEMENT
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses mahnattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the L distances between each xanadu and the escape hatch.
    centre = (int((state.height - 1) / 2),int((state.width - 1) / 2))
    L_distance=0
    if(len(state.xanadus)==0):
      return 0
    for xanadu in state.xanadus:
      xanadu_distance1 = 0
      xanadu_distance2 = 0
      if (xanadu[0] != centre[0]):
        xanadu_distance1 = 1
      if(xanadu[1]!= centre[1]):
        xanadu_distance2=1
      L_distance=L_distance+xanadu_distance1+xanadu_distance2
    return L


def heur_alternate(state):
  # IMPLEMENT
  '''a better lunar lockout heuristic'''
  '''INPUT: a lunar lockout state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  '''
    An admissible heuristic that improves upon Ldistance would be to use L_distance itself but improving upon it
    by taking into considerations all the constarints such as where the current position of the rover is and wether depending on this position there are
    robots available at the required locations around the hatch for this rover to move in and so on. For example if your rover is anywhere along the y axis of the escape hatch 
    then for it to escape throught the hatch there needs to be a robot either right below or above the escape hatch(depending on the position of the rover wether its above or below)
    so that the rover can move in the direction of the robot and go throught eh hatch. There will be multiple deadlock states whose
    cost value would be infinity since the rover cannot escape throught the hatch.For example if you have a robot that is on the centre
    then the rover cannot go in.
    '''
  centre = (int((state.height - 1) / 2), int((state.width - 1) / 2))
  total = 0
  # checking if the list of xanadus is empty
  if (len(state.xanadus) == 0):
    return 0
  for xanadu in state.xanadus:
    # stores the distance of the rover from each robot and we need to find the minimum distance robot
    distance_lists = []
    for robot in state.robots:
      if (xanadu[0] < centre[0]):
        if (xanadu[1] != centre[1] & robot[0] != centre[0] + 1):
          L_distance = 100
        else:
          L_distance = 1
        distance_lists.append(L_distance)
      elif (xanadu[0] > centre[0]):
        if (xanadu[1] != centre[1]):
          L_distance = 100
        else:
          L_distance = 1
        distance_lists.append(L_distance)
      elif (xanadu[1] > centre[1] & xanadu[0] == centre[0]):
        if (robot[1] != centre[1] - 1):
          L_distance = 100
        else:
          L_distance = 1
        distance_lists.append(L_distance)
      elif (xanadu[0] == centre[0] & xanadu[1] < centre[1]):
        if (robot[1] != centre[1] + 1):
          L_distance = 100
        else:
          L_distance = 1
        distance_lists.append(L_distance)
    if (len(distance_lists) != 0):
      minimum_distance = min(distance_lists)
      total = total + minimum_distance
  return total
def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a LunarLockoutState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return (sN.gval + (weight*sN.hval))

def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 2):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  search_engine = SearchEngine('custom', 'full')
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  search_engine.init_search(initial_state,lockout_goal_state,heur_fn,wrapped_fval_function)
  current_time = os.times()[0]
  remaining_timebound= timebound
  #used to store state if it is able to find one within the timebound else stores False
  final_solution=False
  #initializing the values of the costbound tuple
  costbound = (float('inf'), float('inf'), float('inf'))
  while remaining_timebound>0:
    solution=search_engine.search(remaining_timebound,costbound)
    if solution:
      time_passed=(os.times()[0]-current_time)
      remaining_timebound=remaining_timebound-time_passed
      # resetting current_time
      current_time=os.times()[0]
      if (solution.gval < costbound[0]):
        costbound = (solution.gval ,solution.gval,solution.gval)
        final_solution= solution
    # decreasing weight to find the next best solution
    if (weight > 0):
      weight = weight - 1
    else:
      return final_solution
  return final_solution

def anytime_gbfs(initial_state, heur_fn, timebound = 2):
#OPTIONAL
  '''Provides an implementation of anytime greedy best-first search.  This iteratively uses greedy best first search,'''
  '''At each iteration, however, a cost bound is enforced.  At each iteration the cost of the current "best" solution'''
  '''is used to set the cost bound for the next iteration.  Only paths within the cost bound are considered at each iteration.'''
  '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  return 0

PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3),)),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******") 
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_alternate)
    final = se.search(timebound) 

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime Weighted A-star")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    weight = 4
    final = anytime_weighted_astar(s0, heur_alternate, weight, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime GBFS")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    final = anytime_gbfs(s0, heur_alternate, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************")   



  

