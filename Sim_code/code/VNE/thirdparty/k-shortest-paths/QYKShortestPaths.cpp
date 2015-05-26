// ____________________________________________________________________________
//
//  General Information:
//
//  File Name:      QYKShortestPaths.cpp
//  Author:         Yan Qi
//  Project:        KShortestPath
//
//  Description:    Implementation of class(es) CQYKShortestPaths
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Revision History:
//
//  11/23/2006   Yan   Initial Version
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Copyright Notice:
//
//  Copyright (c) 2006 Your Company Inc.
//
//  Warning: This computer program is protected by copyright law and
//  international treaties.  Unauthorized reproduction or distribution
//  of this program, or any portion of it, may result in severe civil and
//  criminal penalties, and will be prosecuted to the maximum extent
//  possible under the law.
//
// ____________________________________________________________________________
#include <iostream>
#include "QYKShortestPaths.h"

namespace asu_emit_qyan
{
	using namespace std;
	//////////////////////////////////////////////////////////////////////
	// Construction/Destruction
	//////////////////////////////////////////////////////////////////////

	CQYKShortestPaths::CQYKShortestPaths( const CQYDirectedGraph& rGraph, int nSource, int nTerminal, int nTopk )
		: m_rGraph(rGraph), m_nSourceNodeId(nSource), m_nTargetNodeId(nTerminal), m_nTopK(nTopk)
	{
		m_pIntermediateGraph = NULL;
		m_pShortestPath4IntermediateGraph = NULL;
	}

	CQYKShortestPaths::~CQYKShortestPaths()
	{
		for (std::vector<CQYDirectedPath*>::iterator pos=m_vTopShortestPaths.begin(); pos!=m_vTopShortestPaths.end(); ++pos)
		{
			delete *pos;
		}
		//
		if (m_pShortestPath4IntermediateGraph != NULL)
		{
			delete m_pShortestPath4IntermediateGraph;
		}
	}

	/************************************************************************
	** Get the top k shortest paths.
	************************************************************************/
	vector<CQYDirectedPath*> CQYKShortestPaths::GetTopKShortestPaths()
	{
		_SearchTopKShortestPaths();
		return m_vTopShortestPaths;
	}


	/************************************************************************
	**  The main function to do searching
	************************************************************************/
	void CQYKShortestPaths::_SearchTopKShortestPaths()
	{
		//////////////////////////////////////////////////////////////////////////
		// first, find the shortest path in the graph
		m_pShortestPath4IntermediateGraph = new CQYShortestPath(m_rGraph);
		CQYDirectedPath* the_shortest_path =
			m_pShortestPath4IntermediateGraph->GetShortestPath(m_nSourceNodeId, m_nTargetNodeId);

		// check the validity of the result
		if(the_shortest_path->GetId() < 0) // the shortest path doesn't exist!
		{
			//std::cerr << "The shortest path doesn't exist!" << std::endl;
			return;
		}else
		{
			the_shortest_path->SetId(0);
		}

		// update the intermediate variables
		m_candidatePathsSet.insert(the_shortest_path);
		m_pathDeviatedNodeMap.insert(pair<int, int>(0, m_nSourceNodeId));


		//////////////////////////////////////////////////////////////////////////
		// second, start to find the other results

		int cur_path_id = 0;
		while (m_candidatePathsSet.size() != 0 && cur_path_id < m_nTopK)
		{
			// Fetch the smallest one from a queue of candidates;
			// Note that it's one of results.
			CQYDirectedPath* cur_path = (*m_candidatePathsSet.begin());
			m_candidatePathsSet.erase(m_candidatePathsSet.begin());

			// Put this candidate into the result list.
			m_vTopShortestPaths.push_back(cur_path);
			++cur_path_id;

			// initiate temporal variables
			int deviated_node_id = m_pathDeviatedNodeMap[cur_path->GetId()];
			vector<int> node_list_in_path = cur_path->GetVertexList();

			// Construct a temporal graph in order to determine the next shortest paths
			m_pIntermediateGraph = new CQYDirectedGraph(m_rGraph);

			// Determine the costs of nodes in the graph
			_DetermineCost2Target(node_list_in_path, deviated_node_id);

			// Iterations for the restoration of nodes and edges
			size_t path_length = node_list_in_path.size();
			int i = 0;
			for (i=path_length-2; i>=0 && node_list_in_path.at(i) != deviated_node_id; --i)
			{
				_RestoreEdges4CostAjustment(node_list_in_path, node_list_in_path.at(i), node_list_in_path.at(i+1));
			}

			// Call _Restore4CostAjustment again for the deviated_node
			_RestoreEdges4CostAjustment(node_list_in_path, deviated_node_id, node_list_in_path.at(i+1), true);

			delete m_pIntermediateGraph;
		}
	}

	/************************************************************************
	** Remove vertices in the input, and recalculate the
	************************************************************************/
	void CQYKShortestPaths::_DetermineCost2Target(vector<int> vertices_list, int deviated_node_id)
	{
		// first: generate a temporary graph with only parts of the original graph
		int count4vertices = m_pIntermediateGraph->GetNumberOfVertices();

		/// remove edges according to the algorithm
		size_t count = vertices_list.size();
		for (int i=0; i<count-1; ++i) // i<count-1: because the final element (i.e, the terminal) should be kept.
		{
			int remove_node_id = vertices_list.at(i);
			for (int j=0; j<count4vertices; ++j)
			{
				size_t cur_edges_count = m_pIntermediateGraph->GetNumberOfEdges();
				if (m_pIntermediateGraph->GetWeight(remove_node_id, j) < CQYDirectedGraph::DISCONNECT)
				{
					m_pIntermediateGraph->SetWeight(remove_node_id, j, CQYDirectedGraph::DISCONNECT);
					--cur_edges_count;
				}
				m_pIntermediateGraph->SetNumberOfEdges(cur_edges_count);
			}
		}

		/// reverse the direction of edges in the temporary graph
		_ReverseEdgesInGraph(*m_pIntermediateGraph);

		// second: run the shortest paths algorithm, but with the target as m_nSource.
		// run the shortest paths algorithm to get the cost of each nodes in the rest of the graph
		if (m_pShortestPath4IntermediateGraph != NULL)
		{
			delete m_pShortestPath4IntermediateGraph;
		}
		m_pShortestPath4IntermediateGraph = new CQYShortestPath(*m_pIntermediateGraph);
		m_pShortestPath4IntermediateGraph->ConstructPathTree(m_nTargetNodeId);

		// third: reverse the edges in the graph again, go back to the original
		_ReverseEdgesInGraph(*m_pIntermediateGraph);
	}

	/************************************************************************
	** Restore edges connecting start_node to end_node
	************************************************************************/
	void CQYKShortestPaths::_RestoreEdges4CostAjustment(vector<int> vertices_list,
		int start_node_id, int end_node_id, bool is_deviated_node)
	{
		/// first: restore the arcs from 'start_node_id' except that reaching 'end_node_id';
		// restore the arcs and recalculate the cost of relative nodes
		int i;
		bool is_updated = false;
		int count4vertices = m_pIntermediateGraph->GetNumberOfVertices();
		for (i=0; i<count4vertices; ++i)
		{
			if(i == end_node_id || i == start_node_id) continue;
			double edge_weight = m_rGraph.GetWeight(start_node_id, i);
			if (edge_weight < CQYDirectedGraph::DISCONNECT)
			{
				if (is_deviated_node && _EdgeHasBeenUsed(start_node_id, i)) continue; //???

				// restore the edge from start_node_id to i;
				m_pIntermediateGraph->SetWeight(start_node_id, i, edge_weight); //??? correct? if the node cost below is 'disconnect'??

				// update the distance if the restored arc makes for a shorter path to the target.
				double node_cost = m_pShortestPath4IntermediateGraph->GetDistance(i);
				if ( node_cost < CQYDirectedGraph::DISCONNECT
					&& (edge_weight+node_cost) < m_pShortestPath4IntermediateGraph->GetDistance(start_node_id))
				{
					m_pShortestPath4IntermediateGraph->SetDistance(start_node_id, edge_weight+node_cost);
					m_pShortestPath4IntermediateGraph->SetNextNodeId(start_node_id, i);
					is_updated = true;
				}
			}
		}

		// if possible, correct the labels and update the paths pool
		double cost_of_start_node =
			m_pShortestPath4IntermediateGraph->GetDistance(start_node_id);

		if ( cost_of_start_node < CQYDirectedGraph::DISCONNECT)
		{
			if(is_updated) _UpdateWeight4CostUntilNode(start_node_id); // a condition checking is added @ 20080111

			//// construct the new path into result vector.

			// the next shortest path: the order of nodes is from the source to the terminal.
			vector<int> new_path;

			int i;
			size_t path_length = vertices_list.size();
			for (i=0; vertices_list.at(i) != start_node_id; ++i)
			{
				new_path.push_back(vertices_list.at(i));
			}

			// stop if the cost of the new path is too large, it's required that its cost before deviated node is small enough.
			int next_node_id = start_node_id;
			do
			{
				new_path.push_back(next_node_id);
				next_node_id = m_pShortestPath4IntermediateGraph->GetNextNodeId(next_node_id);

			} while(next_node_id != m_nTargetNodeId);
			new_path.push_back(m_nTargetNodeId);

			// calculate the cost of the new path
			double cost_new_path = 0;
			size_t length_new_path = new_path.size();
			for (i=0; i<length_new_path-1; ++i)
			{
				cost_new_path += m_rGraph.GetWeight(new_path.at(i), new_path.at(1+i));
			}

			// Update the list of shortest paths
			size_t new_node_id = m_candidatePathsSet.size() + m_vTopShortestPaths.size();
			m_candidatePathsSet.insert(new CQYDirectedPath(new_node_id, cost_new_path, new_path));
			m_pathDeviatedNodeMap.insert(pair<int, int>(new_node_id, start_node_id));
		}

		// second: restore the arc from 'start_node_id' to 'end_node_id';
		double edge_weight = m_rGraph.GetWeight(start_node_id, end_node_id);
		double cost_of_end_node = m_pShortestPath4IntermediateGraph->GetDistance(end_node_id);

		m_pIntermediateGraph->SetWeight(start_node_id, end_node_id, edge_weight);

		if (cost_of_start_node > edge_weight+cost_of_end_node)
		{
			m_pShortestPath4IntermediateGraph->SetDistance(start_node_id, edge_weight+cost_of_end_node);
			m_pShortestPath4IntermediateGraph->SetNextNodeId(start_node_id, end_node_id);
			//
			_UpdateWeight4CostUntilNode(start_node_id);
		}
	}

	/************************************************************************
	** Update the weight of arcs before node_id in the graph
	** TODO: Is there any way to improve the function below!??
	************************************************************************/
	void CQYKShortestPaths::_UpdateWeight4CostUntilNode(int node_id)
	{
		int count4vertices = m_pIntermediateGraph->GetNumberOfVertices();
		std::vector<int> candidate_node_list;
		int cur_pos = 0;
		candidate_node_list.push_back(node_id);

		do
		{
			int cur_node_id = candidate_node_list.at(cur_pos++);

			for (int i=0; i<count4vertices; ++i)
			{
				double edge_weight = m_pIntermediateGraph->GetWeight(i, cur_node_id);
				double cost_node = m_pShortestPath4IntermediateGraph->GetDistance(i);
				double cost_cur_node = m_pShortestPath4IntermediateGraph->GetDistance(cur_node_id);

				if (edge_weight < CQYDirectedGraph::DISCONNECT
					&& cost_node > cost_cur_node + edge_weight)
				{
					m_pShortestPath4IntermediateGraph->SetDistance(i, cost_cur_node+edge_weight);
					m_pShortestPath4IntermediateGraph->SetNextNodeId(i, cur_node_id);

					if(std::find(candidate_node_list.begin(), candidate_node_list.end(), i)
						== candidate_node_list.end())
					{
						candidate_node_list.push_back(i);
					}
				}
			}
		} while(cur_pos < candidate_node_list.size());
	}

	/************************************************************************
	** Reverse directions of all edges in the graph
	************************************************************************/
	void CQYKShortestPaths::_ReverseEdgesInGraph( CQYDirectedGraph& g )
	{
		int i;
		int count4vertices = g.GetNumberOfVertices();
		for (i=0; i<count4vertices; ++i)
		{
			for (int j=0; j<i; ++j)
			{
				if(g.GetWeight(i,j) < CQYDirectedGraph::DISCONNECT
					|| g.GetWeight(j,i) < CQYDirectedGraph::DISCONNECT )
				{
					double dTmp = g.GetWeight(i,j);
					g.SetWeight(i, j, g.GetWeight(j,i));
					g.SetWeight(j, i, dTmp);
				}
			}
		}
	}

	/************************************************************************
	** Check if the edge from start_node to end_node has been in the results or not
	************************************************************************/
	bool CQYKShortestPaths::_EdgeHasBeenUsed( int start_node_id, int end_node_id )
	{
		size_t count_of_shortest_paths = m_vTopShortestPaths.size();
		for (int i=0; i<count_of_shortest_paths; ++i)
		{
			CQYDirectedPath* cur_shortest_path = m_vTopShortestPaths.at(i);
			vector<int> cur_path_list = cur_shortest_path->GetVertexList();

			vector<int>::iterator loc_of_start_id =
				std::find(cur_path_list.begin(), cur_path_list.end(), start_node_id);
			/*
			for(vector<int>::iterator it=cur_path_list.begin();it < cur_path_list.end();it++)
				cout<<*it<<"\t";
			cout<<endl;
			cout<<"ÒªÕÒµÄÄÚÈÝ"<<start_node_id<<endl;
			*/
			if (loc_of_start_id == cur_path_list.end())
			{
				continue;
			}else
			{
				++loc_of_start_id;
				if(loc_of_start_id == cur_path_list.end())
					return false;
				if (*loc_of_start_id == end_node_id)
				{
					return true;
				}
			}
		}
		return false;
	}


} // namespace
