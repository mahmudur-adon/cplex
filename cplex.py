#Importing the package numpy as np.
import numpy as np
# "rnd" is an object that generate random numbers.
rnd = np.random
#"seed(0)" is a method that reset (every time), the same random set of numbers.
rnd.seed(0)
# Number of customers.
n = 10
# Maximum capacity of the vehicle.
Q = 15
#The set of nodes without the depot.
N = [i for i in range(1,n+1)]
# The set of nodes + the depot.
V = [0]+ N
# A collection that contains the demand of each node.
d ={i:rnd.randint(1,10)for i in N}
# Generating random numbers between (0 and 15) * 200.
loc_x = rnd.rand(len(V))*200
# Generating random numbers between (0 and 15) * 100.
loc_y = rnd.rand(len(V))*100
#Importing the package matplotlib.pyplot as plt.
import matplotlib.pyplot as plt
#Plotting the n nodes without the node 0 (depot) and chose the color blue for each node.
plt.scatter(loc_x[1:],loc_y[1:],c='b')
# Associating and plotting each demand in the right of each blue node (customer).
for i in N:
    plt.annotate('$d_%d=%d$'%(i,d[i]),(loc_x[i]+2,loc_y[i]))
    #Ploting the node 0, chosing the red like its color and the square form like a marker.
    plt.plot(loc_x[0],loc_y[0],c='r' ,marker='s')
    #Showing the Initial plot.
    plt.show()
    #Intializing the set of arcs A.
    A = [(i,j) for i in V for j in V if i!=j]
    #Calculating the distance between each node.
    c= {(i,j):np.hypot(loc_x[i]-loc_x[j],loc_y[i]-loc_y[j]) for i,j in A}
    #Importing the docplex.mp.model from the CPLEX as Model
    from docplex.mp.model import Model
    mdl = Model('CVRP')
    #Initializing our binary variable x_i,j
    x=mdl.binary_var_dict (A,name='x')
    #Initializing our cumulative demand u
    u=mdl.continuous_var_dict (N,ub=Q ,name = 'u')
    #Initializing the objectif function
    mdl.minimize(mdl.sum(c[i,j]*x[i,j]for i,j in A))
    #Initialzing the first constraint
    mdl.add_constraints(mdl.sum(x[i,j]for j in V if j!=i)==1 for i in N)
    #Initialzing the second constraint
    mdl.add_constraints(mdl.sum(x[i,j]for i in V if i!=j)==1 for j in N)
    #Initialzing the third constraint
    mdl.add_indicator_constraints_(mdl.indicator_constraint(x[i,j],u[i]+d[j]==u[j])for i,j in A if i!=0 and j!=0)
    #Initialzing the fourth constraint
    mdl.add_constraints(u[i]>=d[i] for i in N)
    #Getting the solution
    solution =mdl.solve(log_output=True)
    #Printing the solution
    print(solution)
    #Identifing the active arcs.
    active_arcs =[  a for a in A if x[a].solution_value> 0.9]
    plt.scatter(loc_x[1:],loc_y[1:],c='b')
for i in N:
    plt.annotate('$d_%d=%d$'%(i,d[i]),(loc_x[i]+2,loc_y[i]))
for i,j in active_arcs :
#Coloring the active arcs
    plt.plot([loc_x[i],loc_x[j]],[loc_y[i],loc_y[j]],c='g',alpha=0.3)
    plt.plot(loc_x[0],loc_y[0],c='r' ,marker='s')
    #Plotting the solution
plt.show()