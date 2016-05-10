#ifndef ecNetworkCommunicationTester_H_
#define ecNetworkCommunicationTester_H_
//     Copyright (c) 2004-2011 Energid Technologies. All rights reserved. ////
//
// Filename:    ecNetworkCommunicationTester.h
//
// Description: Holds the quick-start code described in the Users Guide.
//
// Contents:    class EcNetworkCommunicationTester 
//
/////////////////////////////////////////////////////////////////////////
#ifndef ACTIN_VERSION
#define ACTIN_VERSION "4.0.0"
#endif 

#include <foundCore/ecTypes.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <fstream>
#include <foundCore/ecMath.h>
#include <matrixUtilities/ecReArray.h>


/// This class tests the different remote commands API's to communicate with the robot running with actin software
class EcNetworkCommunicationTester
{
public:
	std::ofstream outputFile;

   /// constructor
   EcNetworkCommunicationTester
      (
      );

   /// destructor
   virtual ~EcNetworkCommunicationTester
      (
      );

   /// copy constructor
   EcNetworkCommunicationTester
      (
      const EcNetworkCommunicationTester& orig
      );

   /// overloading = operator
   const EcNetworkCommunicationTester& operator=
      (
      const EcNetworkCommunicationTester& orig
      )const;

   /// overloading == operator
   EcBoolean operator==
      (
      const EcNetworkCommunicationTester& orig
      )const;

   /// initialize the network
   /// @param[in] ipAddress          (EcString&) address of the network to be connected to.
   /// @param[in] simulationFilename (EcString&) simulation filename to be loaded. Caution: need to load the same simulation file as on the server end
   /// @return                       (EcBoolean) which returns the status of command
   virtual EcBoolean openNetwork
      (
      const EcString& ipAddress,
      const EcString& simulationFilename
      )const;

   /// shut down the network
   /// @return    flag (EcBoolean) which returns the status of command
   virtual EcBoolean closeNetwork
      (
      )const;



   /// set the control descriptor for the simulation file on the server
   /// @param[in] controldDescriptorId (EcU32) Id of the control descriptor
   /// @return    flag (EcBoolean) which returns the status of command
   virtual EcBoolean setControlDescriptorTest
      (
      const EcU32 controldDescriptorId
      )const;

   /// set the control descriptor for the simulation file on the server
   /// @param[in] endEffectorSetId  (EcU32) of the end effector set
   /// @return    flag              (EcBoolean) which returns the status of command
   virtual EcBoolean setEndEffectorSetTest
      (
      const EcU32 endEffectorSetId
      )const;

   /// Test executing manipulation actions
   /// @param[in] filename (EcString&) manipulation action manager filename
   /// @return    flag     (EcBoolean) which returns the status of command
   virtual EcBoolean manipulationActionTest
      (
      const EcString& filename
      )const;

   /// move the robot to the given position, for cyton1500, default endeffector(0) is Wrist Roll.
   /// @param[in] pose (EcCoordinateSystemTransformation&) offset position
   /// @return         (EcBoolean) flag which returns the status of command
   virtual EcBoolean placementTest
      (
      const EcCoordinateSystemTransformation& pose
      )const;

   /// circular movement of the robot as a demonstration of placing the EE at different locations
   /// @return    (EcBoolean) flag which returns the status of command


   /// enable hardware test
   /// @param[in] flag     (EcBoolean) Whether to enable/disable hardware
   /// @return    flag     (EcBoolean)which returns the status of command
   virtual EcBoolean hardwareEnableTest
      (
      const EcBoolean flag
      )const;

   //student defined functions
   //If you create new functions, add declarations here
	EcReArray lab3();
	EcBoolean lab4(EcReArray jim);


	//functions to output to screen or files
	void printVectorFile(EcRealVector temp, int row, char* filename);
	void printVectorFile(EcVector temp, int row, char* filename);
	void printVector(EcVector temp, int row);
	void printVector(EcRealVector temp, int row);
	void printMatrixFile(EcReArray temp, int row, int col, char* filename);
	void printMatrix(EcReArray temp, int row, int col);


	
protected:

};

#endif //ecNetworkCommunicationTester_H_
