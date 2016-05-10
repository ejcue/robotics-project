//     Copyright (c) 2013 Energid Technologies. All rights reserved. ////
//
// Filename:    main.cpp
//
// Description: Remote commands tester for Schunk
//
/////////////////////////////////////////////////////////////////////////
#include "ecNetworkCommunicationTester.h"
#include <controlCore/ecFrameEndEffector.h>
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <manipulation/ecManipulationActionManager.h>
#include <remoteCommand/ecRemoteCommand.h>
#include <xmlReaderWriter/ecXmlObjectReaderWriter.h>
#include <foundCommon/ecCoordSysXForm.h> 
#include <iostream>
#include <iomanip> 
#include <fstream> 

#include <boost/assign/list_of.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <foundCore/ecMath.h>
#include <matrixUtilities/ecReArray.h>
#include <boost/test/floating_point_comparison.hpp>

namespace bpo = boost::program_options;
using namespace std;

//------------------------------------------------------------------------------
#define RC_CHECK(fun) do \
   { \
      std::cout << "Calling " << #fun << std::endl; \
      if(!fun) \
      { \
         std::cerr << "Problem with command " << #fun << "\n"; \
      } \
      else \
      {\
        EcPrint(None) << #fun << " successfully completed!" << "\n"; \
      }\
   } while(0)

int main
   (
   int argc, 
   char **argv
   )
{
    // get command line options
   bpo::options_description options("Options");
   options.add_options()
	   (
	   "help,h", "Show this help message"
	   )
	   (
	   "ipaddress,i",
	   bpo::value<EcString>()->default_value("127.0.0.1"),
      "IpAddress of the computer to connect to"
      )
      (
      "simulationfilename,s",
      bpo::value<EcString>()->default_value("cyton.ecz"),
      "Simulation file to load"
      )
      (
      "performTestType,p",
      bpo::value<EcU32>()->default_value(0),
      "perform required tests"
      )
      ;

    // Parse the command line options
   bpo::parsed_options parsed = bpo::command_line_parser(argc, argv).options(options).run();
   bpo::variables_map vm;
   bpo::store(parsed, vm);
   bpo::notify(vm);

   // get config file
   EcString ipAddress = vm["ipaddress"].as<EcString>();
   EcString simulationfilename = vm["simulationfilename"].as<EcString>();
   EcU32 performTestTypeId = vm["performTestType"].as<EcU32>(); 
   
   EcNetworkCommunicationTester networkComTester;
   


   EcString cytonDir = Ec::Application::getDataDirectory("cyton");
   if(cytonDir.empty())
   { 
      cytonDir = ".";
   }
   simulationfilename = cytonDir + "/" + simulationfilename;
   std::cout << "\n Simulation File :::" << simulationfilename << "\n" << endl;
   networkComTester.openNetwork(ipAddress,simulationfilename);

   //call your functions here
   

   EcReArray current;
   current.setToZeros(7, 7);

   EcReArray goal;
   goal.setToZeros(7, 7);

   EcReArray pseudo;
   pseudo.setToZeros(7, 6);

   EcReArray error;
   error.setToZeros(6, 1);

   EcReArray deltaq;
   deltaq.setToZeros(7, 1);




   int advance;
	
   cout << "\n Move to Goal Position, press 1 when ready" << endl;
	cin >> advance;
	goal = networkComTester.lab3();


	
	cout << "\n Move to current Position, press 1 when ready" << endl;
	cin >> advance;
	current = networkComTester.lab3();

	cout << "\n main check 1" << endl;

	double norm = 1.0;
	int whilestop = 0;

	while (norm > 0.1 && whilestop < 200)
	{

		cout << "\n beginning of while loop" << endl;
		current = networkComTester.lab3();

		error[0][0] = goal[7][0] - current[7][0];
		error[1][0] = goal[7][1] - current[7][1];
		error[2][0] = goal[7][2] - current[7][2];
		error[3][0] = goal[7][3] - current[7][3];
		error[4][0] = goal[7][4] - current[7][4];
		error[5][0] = goal[7][5] - current[7][5];
	

		cout << "error vector \n" << endl;
		networkComTester.printMatrix(error, 6, 1);

/*
		pseudo = {
			current[0][0], current[0][1], current[0][2], current[0][3], current[0][4], current[0][5], current[0][6],
			current[1][0], current[1][1], current[1][2], current[1][3], current[1][4], current[1][5], current[1][6],
			current[2][0], current[2][1], current[2][2], current[2][3], current[2][4], current[2][5], current[2][6],
			current[3][0], current[3][1], current[3][2], current[3][3], current[3][4], current[3][5], current[3][6],
			current[4][0], current[4][1], current[4][2], current[4][3], current[4][4], current[4][5], current[4][6],
			current[5][0], current[5][1], current[5][2], current[5][3], current[5][4], current[5][5], current[5][6]
		};
*/

		pseudo[0][0] = current[0][0];
		pseudo[0][1] = current[0][1];
		pseudo[0][2] = current[0][2];
		pseudo[0][3] = current[0][3];
		pseudo[0][4] = current[0][4];
		pseudo[0][5] = current[0][5];
	

		pseudo[1][0] = current[1][0];
		pseudo[1][1] = current[1][1];
		pseudo[1][2] = current[1][2];
		pseudo[1][3] = current[1][3];
		pseudo[1][4] = current[1][4];
		pseudo[1][5] = current[1][5];
	

		pseudo[2][0] = current[2][0];
		pseudo[2][1] = current[2][1];
		pseudo[2][2] = current[2][2];
		pseudo[2][3] = current[2][3];
		pseudo[2][4] = current[2][4];
		pseudo[2][5] = current[2][5];
	

		pseudo[3][0] = current[3][0];
		pseudo[3][1] = current[3][1];
		pseudo[3][2] = current[3][2];
		pseudo[3][3] = current[3][3];
		pseudo[3][4] = current[3][4];
		pseudo[3][5] = current[3][5];
	

		pseudo[4][0] = current[4][0];
		pseudo[4][1] = current[4][1];
		pseudo[4][2] = current[4][2];
		pseudo[4][3] = current[4][3];
		pseudo[4][4] = current[4][4];
		pseudo[4][5] = current[4][5];
	

		pseudo[5][0] = current[5][0];
		pseudo[5][1] = current[5][1];
		pseudo[5][2] = current[5][2];
		pseudo[5][3] = current[5][3];
		pseudo[5][4] = current[5][4];
		pseudo[5][5] = current[5][5];

		pseudo[6][0] = current[6][0];
		pseudo[6][1] = current[6][1];
		pseudo[6][2] = current[6][2];
		pseudo[6][3] = current[6][3];
		pseudo[6][4] = current[6][4];
		pseudo[6][5] = current[6][5];

		
	
		cout << "\n \n";
		cout << "pseudo vector \n" << endl;


		networkComTester.printMatrix(pseudo, 7, 6);
		deltaq = pseudo*error;
		
		EcReArray send;
		send.setToZeros(7, 1);

		send[0][0] = 0.1 * deltaq[0][0];
		send[1][0] = 0.1 * deltaq[1][0];
		send[2][0] = 0.1 * deltaq[2][0];
		send[3][0] = 0.1 * deltaq[3][0];
		send[4][0] = 0.1 * deltaq[4][0];
		send[5][0] = 0.1 * deltaq[5][0];
		send[6][0] = 0.1 * deltaq[6][0];


		networkComTester.lab4(send);

		norm = sqrt((error[0][0] * error[0][0]) + (error[1][0] * error[1][0]) + (error[2][0] * error[2][0]) + (error[3][0] * error[3][0]) + (error[4][0] * error[4][0]) + (error[5][0] * error[5][0]));
		cout << "\n norm is" << endl;
		cout << norm;
		whilestop +=1;

	}


	cout << "\n end of lab. press 1 to end" << endl;
	cin >> advance;

    return 1;
	}

