// ____________________________________________________________________________
//
//  General Information:
//
//  File Name:      QYConfigCenter.h
//  Author:         Yan Qi
//  Project:        K-ShortestPath
//
//  Description:    The file defines the class CQYConfigCenter, which defines
//	basic functions and handles parameters inputted by the user.
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Revision History:
//
//  11/21/2006   Yan   Initial Version
//
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Copyright Notice:
//
//  Copyright (c) 2006.
//
//  Warning: This computer program is protected by copyright law and
//  international treaties.  Unauthorized reproduction or distribution
//  of this program, or any portion of it, may result in severe civil and
//  criminal penalties, and will be prosecuted to the maximum extent
//  possible under the law.
//
// ____________________________________________________________________________

#ifndef _QYCONFIGCENTER_H_
#define _QYCONFIGCENTER_H_

#include <map>


namespace asu_emit_qyan
{
	class CQYConfigCenter
	{
	public:
		typedef std::pair<int, int> Int_Pair;
		typedef std::map<Int_Pair, double> IntPair_Double_Map;
		typedef std::map<Int_Pair, double>::iterator IntPair_Double_Map_Iterator;
	};

}

#endif //_QYCONFIGCENTER_H_
