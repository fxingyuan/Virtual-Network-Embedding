// ____________________________________________________________________________
//
//  General Information:
//
//  File Name:      QYDirectedGraph.cpp
//  Author:         Yan Qi
//  Project:        KShortestPath
//
//  Description:    Implementation of class(es) CQYDirectedGraph
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Revision History:
//
//  11/21/2006   Yan   Initial Version
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

#ifdef _MSC_VER
#pragma warning(disable: 4786)
#endif // _MSC_VER 

#include <limits>
#include  <stdlib.h>
#include <fstream>
#include <iostream>
#include "QYDirectedGraph.h"

namespace asu_emit_qyan
{
	using namespace std;

	const int CQYDirectedGraph::DEADEND = -1;
	const double CQYDirectedGraph::DISCONNECT = (numeric_limits<double>::max)();

	//////////////////////////////////////////////////////////////////////
	// Construction/Destruction
	//////////////////////////////////////////////////////////////////////

	CQYDirectedGraph::CQYDirectedGraph( const std::string& input_file_name )
	{
		const char* file_name = input_file_name.c_str();
		std::ifstream ifs(file_name);
		if (!ifs)
		{
			std::cout << "The file " << file_name << " can not be opened!" << std::endl;
			exit(1);
		}
		
		// Initiate the members of the class
		_Init();

		// Note the format of the data in the graph file.
		// One integer in the first line is for the number of vertices in the graph.
		ifs >> m_nNumberOfVertices;
		
		// In the following lines, each line contains a directed edge in the graph:
		// the id of starting point, the id of ending point, the weight of the edge. These values 
		// are separated by 'white space'. 
		int i, j;
		double w;
		while(ifs >> i)
		{
			if (i == -1)
			{
				break;
			}
			ifs >> j;
			ifs >> w;
			
			m_pDirectedEdges->insert(pair<pair<int, int>, double>(pair<int, int>(i,j), w));
			
			++m_nNumberOfEdges;
			//
			if (w > m_dMaxWeight)
			{
				m_dMaxWeight = w;
			}
			//
			if (w < m_dMinWeight)
			{
				m_dMinWeight = w;
			}
		}	
		
		m_nNumberOfEdges = m_pDirectedEdges->size();

		ifs.close();	
	}

	CQYDirectedGraph::CQYDirectedGraph( const CQYDirectedGraph& rGraph )
	{
		*this = rGraph;	
	}

	CQYDirectedGraph& CQYDirectedGraph::operator=( const CQYDirectedGraph& rGraph )
	{
		m_nNumberOfVertices = rGraph.m_nNumberOfVertices;
		m_nNumberOfEdges = rGraph.m_nNumberOfEdges;

		m_pDirectedEdges = new CQYConfigCenter::IntPair_Double_Map(*(rGraph.m_pDirectedEdges));
		
		return *this;
	}

	CQYDirectedGraph::~CQYDirectedGraph()
	{
		if (m_pDirectedEdges != NULL)
		{
			delete m_pDirectedEdges;
		}
	}

	void CQYDirectedGraph::_Init()
	{
		m_nNumberOfEdges = 0;
		m_dMaxWeight = 0;
		m_dMinWeight = DISCONNECT;
		m_pDirectedEdges = new CQYConfigCenter::IntPair_Double_Map();
	}


	void CQYDirectedGraph::PrintOut( std::ostream& out_stream ) const
	{
		out_stream << m_nNumberOfVertices << endl << endl;
		PrintOutAllEdges(out_stream);
	}

	void CQYDirectedGraph::PrintOut( const std::string& out_file_name ) const
	{
		ofstream ofs;
		ofs.open(out_file_name.c_str(), ios::out);
		if (!ofs)
		{
			cout << "The file " << out_file_name << " can't be openned!" << endl;
			exit(1);
		}

		PrintOut(ofs);

		ofs.close();
	}

	void CQYDirectedGraph::PrintOutAllEdges( std::ostream& os ) const
	{
		CQYConfigCenter::IntPair_Double_Map_Iterator edge_pos;
		for (edge_pos=m_pDirectedEdges->begin(); edge_pos!=m_pDirectedEdges->end(); ++edge_pos)
		{
			os << edge_pos->first.first << "	"; // start point of the edge
			os << edge_pos->first.second << "	"; // end point of the edge
			os << edge_pos->second << endl;
		}
	}

	void CQYDirectedGraph::RemoveEdge( int i, int j )
	{
		CQYConfigCenter::IntPair_Double_Map_Iterator pos = m_pDirectedEdges->find(pair<int, int>(i,j));
		if (pos != m_pDirectedEdges->end())
		{
			m_pDirectedEdges->erase(pos);
		}
	}

	void CQYDirectedGraph::AddEdge( int i, int j, double weight )
	{
		(*m_pDirectedEdges)[pair<int, int>(i,j)] = weight;	
	}

}
