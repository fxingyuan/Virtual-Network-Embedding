// ____________________________________________________________________________
//
//  General Information:
//
//  File Name:      QYDirectedPath.h
//  Author:         Yan Qi
//  Project:        KShortestPath
//
//  Description:    Declaration of class(es) CQYDirectedPath
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Revision History:
//
//  11/21/2006   Yan   Initial Version
//  01/11/2008   Yan   Modified Version
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

#ifndef _QYDIRECTEDPATH_H_
#define _QYDIRECTEDPATH_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <vector>
#include <fstream>
#include <algorithm>

namespace asu_emit_qyan
{
	class CQYDirectedPath
	{
	public:
		CQYDirectedPath(){};
		CQYDirectedPath(int pId, double pCost, const std::vector<int>& pVertexList)
			:m_nId(pId), m_dCost(pCost)
		{
			m_vVertexList.assign(pVertexList.begin(), pVertexList.end());
		}

		virtual ~CQYDirectedPath(){};

		// Getter and Setter
		int GetId() const { return m_nId; }
		void SetId(int val) { m_nId = val; }

		double GetCost() const { return m_dCost; }
		void SetCost(double val) { m_dCost = val; }

		size_t GetLength() const { return m_vVertexList.size(); }

		std::vector<int> GetVertexList() const { return m_vVertexList; }
		void SetVertexList(std::vector<int> val) { m_vVertexList = val; }

		int GetSourceNodeId() const { return m_nSourceNodeId; }
		void SetSourceNodeId(int val) { m_nSourceNodeId = val; }

		int GetTerminalNodeId() const { return m_nTerminalNodeId; }
		void SetTerminalNodeId(int val) { m_nTerminalNodeId = val; }

		// display the content
		void PrintOut(std::ostream& out_stream) const
		{
			out_stream << "Cost: " << m_dCost << " Length: " << m_vVertexList.size() << std::endl;
			std::copy(m_vVertexList.rbegin(), m_vVertexList.rend(), std::ostream_iterator<int>(out_stream, " "));
			out_stream << std::endl <<  "*********************************************" << std::endl;
		}

	private: // members
		int m_nId;
		int m_nLength;
		double m_dCost;
		std::vector<int> m_vVertexList;

		// intermediate variables
		int m_nSourceNodeId;
		int m_nTerminalNodeId;



	public:
		//// Comparator for paths: the smaller path has less cost.
		class Comparator
		{
		public:
			// Lesson: the condition must be checked completely!!!
			bool operator() (const CQYDirectedPath& s1, const CQYDirectedPath& s2) const
			{
				if (s1.GetCost() == s2.GetCost())
				{
					return s1.GetLength() <= s2.GetLength(); // only considering '<' will cause problems!!!
				}
				return s1.GetCost() < s2.GetCost();
			}

			//
			bool operator() (const CQYDirectedPath* s1, const CQYDirectedPath* s2) const
			{
				return operator()(*s1, *s2);
			}
		};
	};
}

#endif //_QYDIRECTEDPATH_H_
