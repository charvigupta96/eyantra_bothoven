#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_

#include <stdbool.h>
// A utility function to find the vertex with minimum distance
// value, from the set of vertices not yet included in shortest
// path tree
// Number of vertices in the graph

unsigned int path[V],nodes=0,node_count,z=0;
int minDistance(int dist[], bool sptSet[])
{
	// Initialize min value
	int min = INT_MAX, min_index;
	int v;
	for (v = 0; v < V; v++)
	if (sptSet[v] == false && dist[v] < min)
	min = dist[v], min_index = v;

	return min_index;
}

// Function to print shortest path from source to j
// using parent array
void printPath(int parent[], int j)
{
	path[nodes]=j+1;
	nodes=nodes-1;
	if (nodes==-1)
	{
		return;
	}
	printPath(parent, parent[j]);
}
void path_follow(unsigned int first,unsigned int second)
{
	if(dir==0 && first<25 && second==(first-1))
	{
		right_degrees(60);
		stop();
		_delay_ms(1000);
		adjust_right();
	}
	if(dir==-1 && first<25 && second==(first-1))
	{
		node_count=0;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;
		follow_line();
	}
}
// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation
void dijkstra(unsigned char graph[V][V], unsigned char src,unsigned char dest)
{
	int dist[V];  // The output array. dist[i] will hold
	// the shortest distance from src to i
	
	// sptSet[i] will true if vertex i is included / in shortest
	// path tree or shortest distance from src to i is finalized
	bool sptSet[V];
	// Parent array to store shortest path tree
	int parent[V];
	// Initialize all distances as INFINITE and stpSet[] as false
	int i;
	for (i = 0; i < V; i++)
	{
		//parent[0] = -1;
		parent[src]=-1;
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}
	// Distance of source vertex from itself is always 0
	dist[src] = 0;
	// Find shortest path for all vertices
	int count;
	for (count = 1; count < V; count++)
	{
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to src
		// in first iteration.
		int u = minDistance(dist, sptSet);
		// Mark the picked vertex as processed
		sptSet[u] = true;
		// Update dist value of the adjacent vertices of the
		// picked vertex.
		int v;
		for (v = 0; v < V; v++)
		{
			// Update dist[v] only if is not in sptSet, there is
			// an edge from u to v, and total weight of path from
			// src to v through u is smaller than current value of
			// dist[v]
			if (!sptSet[v] && graph[u][v] &&
			dist[u] + graph[u][v] < dist[v])
			{
				parent[v]  = u;//parent is subscript
				dist[v] = dist[u] + graph[u][v];
			}
		}
		if(sptSet[dest])
		{
			break;
		}
	}
	nodes=dist[dest];
	printPath(parent,dest);
	for(z=0;z<dist[dest];z++)
	{
		path_follow(path[z],path[z+1]);
	}
}

#endif 