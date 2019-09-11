# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 12:22:02 2018

@author: darwin
"""



from itertools import permutations
from copy import deepcopy
from time import sleep
import sys
LARGE_NUMBER = sys.float_info.max

# The code below was taken from here:
# https://stackoverflow.com/questions/6284396/permutations-with-unique-values

class unique_element:
    def __init__(self,value,occurrences):
        self.value = value
        self.occurrences = occurrences

def perm_unique(elements):
    eset=set(elements)
    listunique = [unique_element(i,elements.count(i)) for i in eset]
    u=len(elements)
    return perm_unique_helper(listunique,[0]*u,u-1)

def perm_unique_helper(listunique,result_list,d):
    if d < 0:
        yield tuple(result_list)
    else:
        for i in listunique:
            if i.occurrences > 0:
                result_list[d]=i.value
                i.occurrences-=1
                for g in  perm_unique_helper(listunique,result_list,d-1):
                    yield g
                i.occurrences+=1



# This function generates all the possible states
# of the game

def generate_states(perms, configurations):
  states = []
  i = 0
  for configuration in configurations:
    for perm in perms:
      tube1 = perm[:configuration[0]]
      tube2 = perm[configuration[0]:configuration[0] + configuration[1]]
      tube3 = perm[configuration[0] + configuration[1]:]
      states.append([list(tube1), list(tube2), list(tube3)])
  return list(states)


### robot planning
MovingDict = { "0to1": ["pickLL","pickMR","pourLtoR","placeLL","placeMR"],
               "0to2": ["pickLL","pickRR","pourLtoR","placeLL","placeRR"],
               "1to2": ["pickML","pickRR","pourLtoR","placeML","placeRR"],
               "1to0": ["pickLL","pickMR","pourRtoL","placeLL","placeMR"],
               "2to0": ["pickLL","pickRR","pourRtoL","placeLL","placeRR"],
               "2to1": ["pickML","pickRR","pourRtoL","placeML","placeRR"]
}



def Planning(pose):
  out = []
  ### add start pose
  out.append(pose[0][0])
  out.append(pose[0][1])
  out.append(pose[0][2])
  
  
  for i in range(len(pose)-1):
    state = [True,True]
    ### exsample cancel pickRR placelRR
    if pose[i][-2][-2:] == pose[i+1][0][-2:]:
      state[0] = False

    if pose[i][-1][-2:] == pose[i+1][1][-2:]:
      state[1] = False
      
    ### cancel pose
    if state[0]:
      out.append(pose[i][3])
      
    if state[1]:
      out.append(pose[i][4])
      
    if state[0]:
      out.append(pose[i+1][0])
      
    if state[1]:
      out.append(pose[i+1][1])
      
    out.append(pose[i+1][2])
    
  ### add final pose  
  out.append(pose[-1][3])
  out.append(pose[-1][4])

  movement = []
 
  for i in range(len(out)):
    if out[i][:4] == "pour" and out[i-1][:4] != "pour":
        movement.append("PourStart")
    movement.append(out[i])
    if out[i][:4] == "pour" and out[i+1][:4] != "pour":
        movement.append("PourEnd")
  #print(movement)
  s = ""
  for i in movement:
    s += str(len(i))
    s += i
     
  return s






class dijkstra:
    def __init__(self):
         # Define the number of balls and their colors

         self.balls = ['red', 'green', 'purple']*2

         # Define the different quantities of balls that could be
         # placed in each tube

         self.configurations = [(4,1,1),(1,4,1),(1,1,4),\
                           (4,2,0),(4,0,2),(2,4,0),(2,0,4),(0,4,2),(0,2,4),\
                           (3,3,0),(3,0,3),(0,3,3),\
                           (3,2,1),(3,1,2),(2,3,1),(2,1,3),(1,3,2),(1,2,3),\
                           (2,2,2)]

         # Create all the possible unique permutations of the six
         # balls removing repeated color configurations

         self.ball_perms = list(perm_unique(self.balls))

         # Create all possible moves

         self.moves = list(permutations((0,1,2),2))

         # Generate all possible states of the game
         self.states = generate_states(self.ball_perms, self.configurations)

         # Create a full adjacency matrix listing all
         # possible moves and state transitions

         total = 0
         self.matrix = [[0 for i in range(len(self.states))] for j in range(len(self.states))]
         for i, state in enumerate(self.states):
             for move in self.moves:
                 if len(state[move[0]]) > 0:
                     if len(state[move[1]]) < 4:
                         new_state = deepcopy(state)
                         new_state[move[1]].append(new_state[move[0]].pop())
                         j = self.state_to_index(self.states, new_state)
                         self.matrix[i][j] = move
                         total = total + 1

    # Dijkstra algorithm
    def cal(self,orig, dest):
        table = [[LARGE_NUMBER, -1, 0] for i in range(len(self.states))]
        table[orig][0] = 0
        curr = orig
        while True:
            for j in range(len(self.states)):
                if self.matrix[curr][j] != 0:
                    if table[j][0] > table[curr][0] + 1:
                        table[j][0] = table[curr][0] + 1
                        table[j][1] = curr
            table[curr][2] = 1
            curr = table.index(min(table, key=lambda x:x[0]+LARGE_NUMBER*x[2]))
            if curr == dest:
                break
        cost = table[dest][0]
        path = [dest]
        while path[0] != orig:
            path = [table[path[0]][1]] + path
        return cost, path  

    def cal_policy(self,orig,dest):

        i = self.state_to_index(self.states, orig)
        j = self.state_to_index(self.states, dest)
        cost, path = self.cal(i, j)
        move = []
        #print('Cost: %d moves\n' % cost)
        for i in range(len(path)):
            if i > 0:
                pass
                #print('\nMove from tube %d to tube %d\n' % self.matrix[path[i-1]][path[i]])
                move.append(str(self.matrix[path[i-1]][path[i]][0])+"to"+str(self.matrix[path[i-1]][path[i]][1]))
            for j in range(3):
                #print('Tube %d:' % j, states[path[i]][j])
                pass

        pose = []
        for m in move:
            pose.append(deepcopy(MovingDict[m]))

        plan = Planning(pose)
        return plan


    # Helper function to translate a state to an index

    def state_to_index(self,states, state):
        
        state = [list(i) for i in state]
        return states.index(state)    


        





