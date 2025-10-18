# Traffic simulator

This is a traffic simulator app that allows users to visualise how vehicles on the road interact with each other, within any given road network and traffic conditions. This
is done by utilising Open Street Maps libraries, such as OSMNX in this case, and networkx, in order to build real life road networks organised as nodes and edges, and displayed
on a GUI using PyQt5 library. The real life networks can be built by querying Open Street Maps by supplying the latitude-longitude coordinates of a box that encompasses a specific
area on the world map. It uses car-following algorithms to ensure safe following distance behind the vehicle in front, as well as to stop at junctions when other nearby vehicles are crossing it.

## Features

- Real-world road data powered by osmnx and networkx.

- Dynamic vehicle simulation with position, speed, and route tracking.

- Collision-avoidance logic using shared JSON state files.

- PyQtGraph visualization for smooth and fast animations.

- Threaded architecture for parallel car movement and updates

## How to install and run the traffic simulator

### Clone the repository
```
git clone https://github.com/fahadmohaideen/traffic_simulator.git
cd traffic_simulator
```
### Create a virtual environment
It’s recommended to isolate dependencies:
```
python3 -m venv venv
```
Activate it:

  - macOS/Linux:
  ```
  source venv/bin/activate
  ```
  - Windows(PowerShell):
  ```
  .\venv\Scripts\activate
  ```

### Install Dependencies

Use the provided requirements.txt file:
```
pip install -r requirements.txt
```
This installs all essential libraries including:

- osmnx – for OpenStreetMap graph data

- PyQt5 and pyqtgraph – for GUI and real-time visualization

- networkx, numpy, scipy, geopy, matplotlib – for path, math, and geometry calculations

### Run the Application

Launch the main program:
```
python traffic_simulator.py
```
You’ll see a live GUI window with roads plotted and simulated cars moving along their routes.

## How to use

Users are able to spawn multiple cars at any given nodes on the specific chosen network. This can be done by first specifying the routes taken by each of these cars. This can be done 
by modifying the lines of code below (lines 707-710)

```
route1 = nx.shortest_path(G, 1874855397, 1874854292)
route2 = nx.shortest_path(G, 1874853489, 1874857171)
route3 = nx.shortest_path(G, 1874853025, 1874855397)
route4 = nx.shortest_path(G, 1874855397, 1874852805)
```
For example, in route 1, the starting node is 1874855397 and the end node is 1874854292. However, it is very difficult to know which node numbers map to specific physical, geometrical
locations on the map of the target area, just by looking at the coordinates. For visual confirmation, what can be done is that line 683 can be uncommented to print out all the possible nodes.
```
#print(G.nodes)
```
Once all the nodes are displayed, you can enter any node as the starting node for the routes and notice the location of the car. Through trial and error, you would be able to know which node to position
your car on the map.

Finally, you can also vary the speed of the cars by changing the second argument of each tuple (representing a car), which is the float value (eg 25.0).
```
cars = [(car1, 25.0), (car2, 15.0), (car3, 20.0), (car4, 5.0)]
```


