/* $Revision: 2015.03 $ */
/*
 *	static2xls.cpp
 *  For virtual network embedding 
 *	Transform txt file to excel file using MATLAB
 *	Engine functions from a C++ program.
 *  By Xingli , Inc.
 */
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"
using namespace std;

//#define  BUFSIZE 256

int main(int argc, char *argv[])
{
	Engine *ep;
	mxArray *importdir = NULL,*outputdir = NULL, *result = NULL;
	if(argc!=3)
	{
		cerr << "usage: static2xls <importdir> <outputdir>" << endl;
		cerr << "<importdir>: directory containing source file(.statics)" << endl;
		cerr << "<outputdir>: directory for saving results(excels files) " << endl;
	}
	/*
	 * Call engOpen with a NULL string. This starts a MATLAB process 
     * on the current host using the command "matlab".
	 */
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}

	/*
	 * Create a variable for our data
	 */
	importdir = mxCreateStringFromNChars(argv[1],strlen(argv[1]));
	outputdir = mxCreateStringFromNChars(argv[2],strlen(argv[2]));
	/*
	 * Place the variable importdir,outputdir into the MATLAB workspace
	 */
	engPutVariable(ep, "importdir", importdir);
	engPutVariable(ep, "outputdir", outputdir);

	/*
	 * Evaluate a function 
	 */
	engEvalString(ep, "static2xls(importdir,outputdir);");

	/*
	 * Buffer setting
	 */

	//char buffer[BUFSIZE+1];
	//buffer[BUFSIZE]='\0';
	//engOutputBuffer(ep, buffer, BUFSIZE);
	//engEvalString(ep,"userpath('F:\matlab\work');");
	//engEvalString(ep,"savepath;");
	//printf("%s",buffer);

	/*
	 * memory destroy
	 */
	mxDestroyArray(importdir);
	mxDestroyArray(outputdir);
	engEvalString(ep, "close all;");
	engEvalString(ep, "exit;");
	return EXIT_SUCCESS;
}








