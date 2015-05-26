// ____________________________________________________________________________
//
//  General Information:
//
//  File Name:      QYKShortestPaths.h
//  Author:         Yan Qi
//  Project:        KShortestPath
//
//  Description:    Declaration of class(es) CQYKShortestPaths
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

#ifndef _QYKSHORTESTPATHS_H_
#define _QYKSHORTESTPATHS_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "QYShortestPath.h"

namespace asu_emit_qyan
{
	using namespace std;

	class CQYKShortestPaths
	{
	public:
		CQYKShortestPaths(const CQYDirectedGraph& rGraph, int nSource, int nTerminal, int nTopk);
		virtual ~CQYKShortestPaths();

		vector<CQYDirectedPath*> GetTopKShortestPaths();

	private: // methods

		void _Init();
		void _SearchTopKShortestPaths();

		void _DetermineCost2Target(vector<int> vertices_list, int deviated_node_id);
		void _RestoreEdges4CostAjustment(vector<int> vertices_list, int start_node_id, int end_node_id, bool is_deviated_node = false);
		void _UpdateWeight4CostUntilNode(int node_id);
		void _ReverseEdgesInGraph(CQYDirectedGraph& g);
		bool _EdgeHasBeenUsed(int start_node_id, int end_node_id);

	private: // members

		int m_nTopK;
		int m_nSourceNodeId;
		int m_nTargetNodeId;

		const CQYDirectedGraph& m_rGraph;
		CQYDirectedGraph* m_pIntermediateGraph;
		CQYShortestPath* m_pShortestPath4IntermediateGraph;

		// variable to store the top shortest paths
		vector<CQYDirectedPath*> m_vTopShortestPaths;

		// a queue of candidates
		set<CQYDirectedPath*, CQYDirectedPath::Comparator> m_candidatePathsSet;

		// index for node where the path derives from others
		map<int, int> m_pathDeviatedNodeMap;
	};
}

#endif //_QYKSHORTESTPATHS_H_
