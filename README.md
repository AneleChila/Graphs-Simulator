# Graphs Simulator  Dijkstra’s algorithm 
This assignment concerns using directed graphs to develop a simulation of an emergency medical service; of ambulances, hospitals, and medical emergencies.  
 Hospitals have emergency units.  Ambulances are stationed at hospitals. 
 The road network that connects hospitals and victim locations can be modelled with a weighted directed graph. 
 Emergency calls are received from victims at particular locations. 
 Ambulances must be routed to victims and then on to hospitals. 
(An ambulance can only take a victim to the hospital at which it is stationed.) 

ROUND TRIP SIMULATION

Given an incoming call, the problem is to identify the ambulance that can make the lowest cost round trip to the victim and then back to the hospital at which it is stationed. 
‘SimulatorOne.java’ accepts as input a file containing data on roads, hospitals, and incoming call events. The program will then, for each call, calculate and output details of the (best) shortest round trip. 
This program uses Dijkstra’s algorithm to calculate the lowest cost path between victims and hospitals. 
A small consideration: since the graph is directed, the lowest cost route to a victim may not be the lowest cost route back to the hospital (in fact, that route may not exist). 
