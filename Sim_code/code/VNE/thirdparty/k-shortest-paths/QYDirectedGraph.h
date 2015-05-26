// ____________________________________________________________________________
//
//  General Information:
//
//  File Name:      QYDirectedGraph.h
//  Author:         Yan Qi
//  Project:        KShortestPath
//
//  Description:    Declaration of class(es) CQYDirectedGraph
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Revision History:
//
//  11/21/2006   Yan   Initial Version
//  01/11/2007   Yan   Modified Version: correct the way to calculate the number of edges in the graph.
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

#ifndef _QYDIRECTEDGRAPH_H_
#define _QYDIRECTEDGRAPH_H_

#include "QYConfigCenter.h"
#include <string>

namespace asu_emit_qyan
{
    // ____________________________________________________________________________
    //
    // Class:       CQYDirectedGraph
    //
    // Purpose:     CQYDirectedGraph defines the directed graph with a list of
    //              directed edges, associated with its weight.
    //
    // Notes:		Two ways to construct a graph:
	//				1. Assign the path of the file recording the graph
	//				2. Transfer an existing object.
    //
    // See Also:
    //
    // ____________________________________________________________________________
	class CQYDirectedGraph
	{
	public:
		const static double DISCONNECT;
		const static int DEADEND;
		//
		CQYDirectedGraph(){_Init();}
		CQYDirectedGraph(const std::string& input_file_name);
		virtual ~CQYDirectedGraph();

		CQYDirectedGraph(const CQYDirectedGraph& rGraph);
		CQYDirectedGraph& operator=(const CQYDirectedGraph& rGraph);

		// The methods below play the persistent role.
		void PrintOut(const std::string& out_file_name) const;
		void PrintOut(std::ostream& out_stream) const;
		void PrintOutAllEdges(std::ostream& os) const;


		void RemoveEdge(int i, int j);
		void AddEdge(int i, int j, double weight);

		// Getter and setter
		int GetNumberOfVertices() const { return m_nNumberOfVertices; }
		void SetNumberOfVertices(int val) { m_nNumberOfVertices = val; }

		size_t GetNumberOfEdges() const { return m_nNumberOfEdges; }
		void SetNumberOfEdges(size_t val) { m_nNumberOfEdges = val; }

		double GetMaxWeight() const { return m_dMaxWeight; }
		void SetMaxWeight(double val) { m_dMaxWeight = val; }

		double GetMinWeight() const { return m_dMinWeight; }
		void SetMinWeight(double val) { m_dMinWeight = val; }

		double GetWeight(int i, int j) const { return m_pDirectedEdges->count(std::pair<int, int>(i,j)) ? (*m_pDirectedEdges)[std::pair<int, int>(i,j)] : DISCONNECT; }
		void SetWeight(int i, int j, double val) { (*m_pDirectedEdges)[std::pair<int, int>(i,j)] = val; }

	private:
		CQYConfigCenter::IntPair_Double_Map* m_pDirectedEdges;

		int m_nNumberOfVertices;
		size_t m_nNumberOfEdges;

		double m_dMaxWeight;
		double m_dMinWeight;

		void _Init();

	};

}

#endif //_QYDIRECTEDGRAPH_H_
