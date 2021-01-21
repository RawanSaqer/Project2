
### Student Name:
#####  Rawan AlHarbi

### ID: 
##### 421010012 

import pandas as pd
import json
import numpy as np
from ipywidgets import FileUpload
import os
import math
import sys

class Airport:
    def __init__(self,CityName,ID,Latitude,Longitude,Distinations):
        self.ID=ID  
        self.IsVisited=False;
        self.Latitude=math.radians(float(Latitude))                
        self.Longitude=math.radians(float(Longitude))          
        self.Distinations= Distinations  
        self.DistinationObjects= []
        self.CityName=CityName
        self.Distanes=[]
    
    def CalculateDistances(self):
        for d in self.DistinationObjects:
            R = 6373.0   
            dlon =  self.Longitude -  d.Longitude
            dlat = self.Latitude - d.Latitude
             #Haversine formula
            a = math.sin(dlat / 2)**2 + math.cos(self.Latitude) * math.cos(d.Latitude) * math.sin(dlon / 2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance = R * c
            self.Distanes.append(distance)
    def getDistinationObjects(self,Airports):
        for A in Airports:
            if A.ID in self.Distinations:
                self.DistinationObjects.append(A)

class Dijkstra:
    def __init__(self):
        self.Airports=[]    
        self.V=0
        self.AdjecancyMatrix= [[]]   
        
    def initializeAirports(self):
        path=os.getcwd()+"/Airports.json"  
        ALL_Airports = json.loads(open(path, 'r', encoding='utf-8-sig').read())
        for A in ALL_Airports:
            self.Airports.append(Airport(A["City"],A["Airport ID"],A["Latitude"],A["Longitude"],A["destinations"]))
        self.V=len(self.Airports)
        for A in self.Airports:
            A.getDistinationObjects(self.Airports)
            A.CalculateDistances()
        self.AdjecancyMatrix= [[0 for column in range(self.V)]  
                              for row in range(self.V)] 
        self.CalculateMatrix()
        

    def CalculateMatrix(self):
        for i in range(self.V):
            for j in range(self.V):
                for k in range(len(self.Airports[i].Distanes)):
                    if self.Airports[j].ID == self.Airports[i].Distinations[k]:
                        self.AdjecancyMatrix[i][j]=self.Airports[i].Distanes[k]
   
    def dijkstra(self, src):
        print(self.V)
        dist = [sys.maxsize ] * self.V 
        dist[src] = 0
        for count in range(self.V)  :
            u = self.minDistance(dist) 
            self.Airports[u].IsVisited = True
            for v in range(self.V):
                if self.AdjecancyMatrix[u][v] > 0 and self.Airports[v].IsVisited == False and dist[v] > dist[u] + self.AdjecancyMatrix[u][v]:
                    dist[v] = dist[u] + self.AdjecancyMatrix[u][v]
        self.printSolution(dist,u) 

    def minDistance(self, dist): 
        min_index=0
        min = sys.maxsize 
        for v in range(self.V): 
            if dist[v] < min and self.Airports[v].IsVisited == False: 
                min = dist[v] 
                min_index = v 
        return min_index 

    def printSolution(self, dist,u): 
            print ("Vertex tDistance from Source") 
            for node in range(self.V): 
                print ("The Shortest Distance Between the -", self.Airports[u].CityName, "- and the -",self.Airports[node].CityName,"-  is = ", dist[node])

dijk=Dijkstra()
dijk.initializeAirports()
dijk.dijkstra(0)





