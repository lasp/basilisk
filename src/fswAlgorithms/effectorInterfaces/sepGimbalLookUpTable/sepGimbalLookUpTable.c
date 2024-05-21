/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

/* Import the module header file */
#include "sepGimbalLookUpTable.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

/* Other required files to import */
#include <stdbool.h>
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#define MAXCHAR 4091
#define MAX_ROWS 4091
#include <stdio.h>  /* defines FILENAME_MAX */
#define WINDOWS  /* uncomment this line to use it for windows.*/
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */

void SelfInit_sepGimbalLookUpTable(SepGimbalLookUpTableConfig* configData, int64_t moduleID)
{
    // Initialize the output messages
    HingedRigidBodyMsg_C_init(&configData->motor1AngleOutMsg);
    HingedRigidBodyMsg_C_init(&configData->motor2AngleOutMsg);
    const char* fileName = "C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\\sepGimbalLookUpTable\\platformAngle_motorAngle.csv";

    // Allocate memory for rows
    for (int i = 0; i < MAX_ROWS; i++) {
        configData->rows[i] = malloc(sizeof(LookUpTableRowElements));
        if (configData->rows[i] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error gracefully
        }
    }

    // Allocate memory for stored
    for (int i = 0; i < MAX_ROWS; i++) {
        configData->selectedTipRows[i] = malloc(sizeof(LookUpTableRowElements));
        if (configData->selectedTipRows[i] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error gracefully
        }
    }
    int result = readData(fileName, configData->rows);
}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/

void Reset_sepGimbalLookUpTable(SepGimbalLookUpTableConfig* configData, uint64_t callTime, int64_t moduleID){
    // Check if the required input message is linked
    if (!HingedRigidBodyMsg_C_isLinked(&configData->desiredGimbalTipAngleInMsg) && !HingedRigidBodyMsg_C_isLinked(&configData->desiredGimbalTiltAngleInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: sepGimbalLookUpTable.desiredGimbalTipAngleInMsg and sepGimbalLookUpTable.desiredGimbalTiltAngleInMsg wasn't connected.");
    }

    // Set the platform tip angle to zero
    configData->inputTipAngle =0.0;                            

    // Set the platform tilt angle to zero  
    configData->inputTiltAngle = 0.0;                      

    // Set the motor1 angle to zero                              
    configData->motor1Angle = 0.0;

    // Set the motor2 angle to zero  
    configData->motor2Angle = 0.0;   

    //Set the norm to zero  
    configData->norm = 0.0;

}

int findMatchingTipAngles(LookUpTableRowElements *rows[], double inputTipAngle, double inputTiltAngle, LookUpTableRowElements **selectedTipRows) {
    int numMatches = 0;
    double minDifferences[2] = {DBL_MAX, DBL_MAX}; // Initialize the minimum differences to large floating values

    //changes  made was adding norm, 2 arrays in .h file and addding the if len loop 
    
    // Iterate through all rows to find the two closest tip angle values
    for (int i = 0; i < MAX_ROWS; i++) {
        double norm = sqrt((inputTipAngle - rows[i]->desiredTipAngle)*(inputTipAngle - rows[i]->desiredTipAngle) + (inputTiltAngle - rows[i]->desiredTiltAngle)*(inputTiltAngle - rows[i]->desiredTiltAngle));
    
    if len LookUpTableRowElements *smallest_distances[] < 4 or double norm < maxLookUpTableRowElements *smallest_distances[]:
        # If there are less than 4 smallest distances recorded or the current distance is smaller than the largest recorded distance
        # Add the current distance and its corresponding index to the lists
        if len(smallest_distances) == 4:
            # If there are already 4 smallest distances recorded, remove the largest one
            max_distance_index = smallest_distances.index(max(smallest_distances))
            del smallest_distances[max_distance_index]
            del smallest_index[max_distance_index]
        smallest_distances.append(norm)
        smallest_index.append(i)

        // Update the minimum differences and select the nearest rows
        if (difference < minDifferences[0]) {
            // Shift the current closest row to the second position
            minDifferences[1] = minDifferences[0];
            selectedTipRows[1] = selectedTipRows[0];
            // Store the new closest row
            minDifferences[0] = difference;
            selectedTipRows[0] = rows[i];
            numMatches = 1; // Reset the count to 1 since we found a closer row
        } else if (difference == minDifferences[0]) {
            // If the current difference is equal to the minimum difference, store this row
            // Increment numMatches to account for the duplicate closest row
            selectedTipRows[++numMatches] = rows[i];
        } else if (difference < minDifferences[1]) {
            // If the difference is less than the second minimum difference, update
            // Only update minDifferences[1] and selectedTipRows[1] here, don't reset numMatches
            minDifferences[1] = difference;
            selectedTipRows[1] = rows[i];
        }   
    }
    
    return numMatches + 1; // Return the number of nearest rows found (plus one for indexing)
}

// // Function to find the rows with equal or nearest tip angle value
// int findMatchingTipAngle(LookUpTableRowElements *rows[], double inputTipAngle, LookUpTableRowElements **selectedTipRows) {
//     int numMatches = 0;
//     double minDifference = DBL_MAX; // Initialize the minimum difference to a large floating value
    
//     // Iterate through all rows to find the closest tip angle values
//     for (int i = 0; i < MAX_ROWS; i++) {
//         double difference = fabs(inputTipAngle - rows[i]->desiredTipAngle); // Calculate the absolute difference

//         if (difference < minDifference) {
//             // If the current difference is less than the minimum difference,
//             // update the minimum difference and select this row.
//             numMatches = 0; // Reset selected rows
//             minDifference = difference;
//             selectedTipRows[numMatches++] = rows[i];
//         } else if (difference == minDifference) {
//             // If the current difference is equal to the minimum difference,
//             // store this row as well (multiple rows with the same difference).
//             selectedTipRows[numMatches++] = rows[i];
//         }
//     }
    
//     return numMatches;
// }

// Function to find the rows with equal or nearest tilt angle value
    int findMatchingTiltAngle(LookUpTableRowElements *matchingRowsTip[], int numMatchingRowsTip, double inputTiltAngle, LookUpTableRowElements **selectedTiltRows) {
    int numMatches = 0;
    double minDifference = DBL_MAX; // Initialize the minimum difference to a large floating value
    
    // Iterate through all rows to find the closest tilt angle values
    for (int i = 0; i < numMatchingRowsTip; i++) {
        double difference = fabs(inputTiltAngle - matchingRowsTip[i]->desiredTiltAngle); // Calculate the absolute difference

        if (difference < minDifference) {
            // If the current difference is less than the minimum difference,
            // update the minimum difference and select this row.
            numMatches = 0; // Reset selected rows
            minDifference = difference;
            selectedTiltRows[numMatches++] = matchingRowsTip[i];
        } else if (difference == minDifference) {
            // If the current difference is equal to the minimum difference,
            // store this row as well (multiple rows with the same difference).
            selectedTiltRows[numMatches++] = matchingRowsTip[i];
        }
    }
    
    return numMatches;
}

/*! This method recieves the desired tip and tilt angles and outputs the motor angles.
The desired gimbal angles are then written to the output message to find the corresponding motor angles from the lookup table.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] The current time of simulation
 @param moduleID The module identifier
*/

void Update_sepGimbalLookUpTable(SepGimbalLookUpTableConfig *configData, uint64_t callTime, int64_t moduleID){

    // Create the buffer messages
    HingedRigidBodyMsgPayload motor1AngleOut;
    HingedRigidBodyMsgPayload motor2AngleOut;
    HingedRigidBodyMsgPayload desiredGimbalTipAngleIn;
    HingedRigidBodyMsgPayload desiredGimbalTiltAngleIn;

    // Zero the output messages
    motor1AngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();
    motor2AngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();

    // Read the gimbal tip angle input message
    desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&configData->desiredGimbalTipAngleInMsg)){
        desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_read(&configData->desiredGimbalTipAngleInMsg);
        printf("desiredGimbalTipAngleIn.theta %f\n", desiredGimbalTipAngleIn.theta);
    }

    // Read the gimbal tilt angle input message
     desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&configData->desiredGimbalTiltAngleInMsg)){
        desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_read(&configData->desiredGimbalTiltAngleInMsg);
        printf("desiredGimbalTiltAngleIn.theta %f\n", desiredGimbalTiltAngleIn.theta);
    }
    
    // Assign the input tip and tilt angles as the desired theta In message
    configData->inputTipAngle = desiredGimbalTipAngleIn.theta;
    configData->inputTiltAngle = desiredGimbalTiltAngleIn.theta;

    // Print inputTipAngle and inputTiltAngle
    printf("Input Tip Angle: %f\n", configData->inputTipAngle);
    printf("\nInput Tilt Angle: %f\n", configData->inputTiltAngle);

   // Call findMatchingTipAngle to get selected rows based on inputTipAngle
    LookUpTableRowElements *matchingRowsTip[MAX_ROWS];
    int numMatchingRowsTip = findMatchingTipAngle(configData->rows, configData->inputTipAngle, matchingRowsTip);
    printf("\nNumber of matching rows based on Tip Angle: %d\n\n", numMatchingRowsTip);

    // Print the selected rows based on tip angle
    printf("Matching Rows based on Tip Angle:\n");
    for (int i = 0; i < numMatchingRowsTip; i++) {
        printf("\n%d: rowNum=%.2f, desiredTipAngle=%.2f, desiredTiltAngle=%.2f, motor1Angle=%.2f, motor2Angle=%.2f",
            i + 1,
            matchingRowsTip[i]->rowNum,
            matchingRowsTip[i]->desiredTipAngle,
            matchingRowsTip[i]->desiredTiltAngle,
            matchingRowsTip[i]->motor1Angle,
            matchingRowsTip[i]->motor2Angle);
    }

    // Call findMatchingTiltAngle to get selected rows with needed inputTiltAngle from the tip rows founded
    LookUpTableRowElements *matchingRowsTilt[MAX_ROWS];
    int numMatchingRowsTilt = findMatchingTiltAngle(matchingRowsTip, numMatchingRowsTip, configData->inputTiltAngle, matchingRowsTilt);
    printf("\nNumber of matching rows based on Tilt Angle within the matching tip angle subset: %d\n\n", numMatchingRowsTilt);

    // Declare variables to store the desired tip and tilt angles
    double selectedDesiredTipAngle = 0.0;
    double selectedDesiredTiltAngle = 0.0;

    // Print the selected rows based on tilt angle within the matching tip angle subset
    printf("Matching Rows based on Tilt Angle within the matching tip angle subset:\n");
    for (int i = 0; i < numMatchingRowsTilt; i++) {
        printf("Row %d: rowNum=%.2f, desiredTipAngle=%.2f, desiredTiltAngle=%.2f, motor1Angle=%.2f, motor2Angle=%.2f\n",
            i + 1,
            matchingRowsTilt[i]->rowNum,
            matchingRowsTilt[i]->desiredTipAngle,
            matchingRowsTilt[i]->desiredTiltAngle,
            matchingRowsTilt[i]->motor1Angle,
            matchingRowsTilt[i]->motor2Angle);

            // Assign the values to the variables from the first selected row
        if (i == 0) {
            selectedDesiredTipAngle = matchingRowsTilt[i]->desiredTipAngle;
            selectedDesiredTiltAngle = matchingRowsTilt[i]->desiredTiltAngle;
        }
    }

    // Declare variables to store the corresponding motor 1 and 2 angles
    double motor1Angle = 0.0; 
    double motor2Angle = 0.0;

    // Plook1 and motor 2 angles
    // Loop through the matching rows and print motor 1 and motor 2 angles
    for (int i = 0; i < numMatchingRowsTilt; i++) {
        motor1Angle = matchingRowsTilt[i]->motor1Angle;
        motor2Angle = matchingRowsTilt[i]->motor2Angle;
        printf("\nmotor1Angle=%.2f\nmotor2Angle=%.2f\n", motor1Angle, motor2Angle);
        
        // // Output the desired motor 1 angle
        // motor1AngleOut.theta = motor1Angle;
        // HingedRigidBodyMsg_C_write(&motor1AngleOut, &configData->motor1AngleOutMsg, moduleID, callTime);
        // printf("\n motor1 Out Msg %.2f", motor1AngleOut.theta);

        // // Output the desired motor 2 angle
        // motor2AngleOut.theta = motor2Angle;
        // HingedRigidBodyMsg_C_write(&motor2AngleOut, &configData->motor2AngleOutMsg, moduleID, callTime);
        // printf("\n motor2 Out Msg %.2f", motor2AngleOut.theta);
    }
}

int readData(const char *filename, LookUpTableRowElements* rows[]) {
    FILE *csvFile;
    printf("%s\n", filename);
    char cwd[2000];
    GetCurrentDir( cwd, FILENAME_MAX );
    printf("Current working dir: %s\n", cwd);
    
    csvFile = fopen("platformAngle_motorAngle_small.csv", "r");
    if (csvFile == NULL) {
        printf("Error opening CSV file.\n");
        return 1;
    }

    // SepGimbalLookUpTableConfig lookup[MAXCHAR];
    int numColumnsMatched = 0;
    int record = 0;
    int numRecords = 0;
    
    do
    {
        LookUpTableRowElements tempRowArray;
        // Initialize the struct with default values
        tempRowArray.rowNum = 0.0; // Default character value
        tempRowArray.desiredTipAngle = 0.0; // Default double value
        tempRowArray.desiredTiltAngle = 0.0; // Default double value
        tempRowArray.motor1Angle = 0.0; // Default double value
        tempRowArray.motor2Angle = 0.0; // Default double value
        
        numColumnsMatched = fscanf(csvFile,
                            "%lf, %lf, %lf, %lf, %lf\n",
                            &tempRowArray.rowNum,
                            &tempRowArray.desiredTipAngle,
                            &tempRowArray.desiredTiltAngle,
                            &tempRowArray.motor1Angle,
                            &tempRowArray.motor2Angle);

        if (numColumnsMatched == 5) {
            // Allocate memory for a new LookUpTableRowElements pointer and copy the data
            rows[record] = malloc(sizeof(LookUpTableRowElements));
            if (rows[record] == NULL) {
                printf("Memory allocation error.\n");
                return 1;
            }
            
            // Copy the data field by field
            rows[record]->rowNum = tempRowArray.rowNum;
            rows[record]->desiredTipAngle = tempRowArray.desiredTipAngle;
            rows[record]->desiredTiltAngle = tempRowArray.desiredTiltAngle;
            rows[record]->motor1Angle = tempRowArray.motor1Angle;
            rows[record]->motor2Angle = tempRowArray.motor2Angle;

            record++;
        }

        if (numColumnsMatched !=5 && feof(csvFile)){
            printf("error file format.\n");
            return 1;
        }

        if (ferror(csvFile)){
            printf("error file reading.\n");
            return 1;
        }

    } while (!feof(csvFile));

    fclose(csvFile);
    printf("\n%d records read.\n\n", record);

    for (int i=0; i<record; i++) {
        printf("%.3f %.3f %.3f %.3f %.3f",
                rows[i]->rowNum,
                rows[i]->desiredTipAngle,
                rows[i]->desiredTiltAngle,
                rows[i]->motor1Angle,
                rows[i]->motor2Angle);
        printf("\n");
    }
    return 0;

}    


