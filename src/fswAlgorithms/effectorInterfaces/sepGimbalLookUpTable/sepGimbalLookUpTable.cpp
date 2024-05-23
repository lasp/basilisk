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
#include <cstdio>

/* Other required files to import */
#include <stdbool.h>
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

#define MAXCHAR 4091
#define MAX_ROWS 114
#define MAX_COLUMNS 78

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
    const char* fileName_1 = "C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\\sepGimbalLookUpTable\\P_M1.csv";
    const char* fileName_2 = "C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\\sepGimbalLookUpTable\\P_M2.csv";

    // Allocate memory for rows
    for (int i = 0; i < MAX_ROWS; i++) {
        configData->rows[i] = malloc(sizeof(LookUpTableRowElements));
        if (configData->rows[i] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error of rows
        }
    }

    // Allocate memory for stored rows
    for (int i = 0; i < MAX_ROWS; i++) {
        configData->selectedTipRows[i] = malloc(sizeof(LookUpTableRowElements));
        if (configData->selectedTipRows[i] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error of columns
        }
    }

    int result_row1 = readData1(fileName_1, configData->rows);
    int result_row2 = readData2(fileName_2, configData->rows);

    // Allocate memory for columns
    for (int j = 0; j< MAX_COLUMNS; ij+) {
        configData->columns[j] = malloc(sizeof(LookUpTableRowElements));
        if (configData->columns[j] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error gracefully
        }
    }

    // Allocate memory for stored columns
    for (int j = 0; j < MAX_COLUMNS; j++) {
        configData->selectedTipColumnss[j] = malloc(sizeof(LookUpTableRowElements));
        if (configData->selectedTipColumns[j] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error gracefully
        }
    }

    int result_column1 = readData1(fileName_1, configData->columns);
    int result_column2 = readData2(fileName_2, configData->columns);

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
}



// Function to find the nearest 2 rows with equal or nearest tilt angle value
int findMatchingTiltAngle(LookUpTableRowElements *rows[], double inputTiltAngle, LookUpTableRowElements **selectedTiltRows) {
    int numMatchesRows = 0;
    double minDifference = DBL_MAX; // Initialize the minimum difference 
    double minDifference2 = DBL_MAX; // Initialize the minimum difference 
    LookUpTableRowElements *nearestValue1 = 0.0;
    LookUpTableRowElements *nearestValue2 = 0.0;

    // Iterate through all rows to find the closest tilt angles values
    for (int i = 0; i < MAX_ROWS; i++) {
        double difference = fabs(inputTiltAngle - rows[i]->desiredTiltAngle); // Calculate the absolute difference

        if (difference < minDifference) {
            // If the current difference is less than the minimum difference1, update the minimum difference and select this row.
            numMatchesRows = 0; // Reset selected rows
            minDifference = difference;
            nearestValue1 = rows[i]
            i_nearestValue1 = i;
        } else if (minDifference < minDifference2) {
            // If the current difference is equal to the minimum difference,
            // store this row as well (multiple rows with the same difference).
            minDifference2 = minDifference;
            nearestValue2 = rows[i]   
            i_nearestValue2 = i;     }
    }
    selectedTiltRows[0] = nearestValue1;
    selectedTiltRows[1] = nearestValue2;
    return selectedTiltRows;
}

// Function to find the nearest 2 columns tip angle value
int findMatchingTipAngle(LookUpTableRowElements *columns[], double inputTipAngle, LookUpTableRowElements **selectedTipColumns) {
    int numMatchesColumns = 0;
    double minDifference = DBL_MAX; // Initialize the minimum difference 
    double minDifference2 = DBL_MAX;
    LookUpTableRowElements *nearestValue1 = 0.0;
    LookUpTableRowElements *nearestValue2 = 0.0;

    // Iterate through all columns to find the closest tip angles values
    for (int j = 0; j < MAX_COLUMNS; j++) {
        double difference = fabs(inputTipAngle - columns[j]->desiredTipAngle); // Calculate the absolute difference

        if (difference < minDifference) {
            // If the current difference is less than the minimum difference1, update the minimum difference and select this row.
            numMatchesColumns = 0; // Reset selected rows
            minDifference = difference;
            nearestValue1 = columns[j];
            j_nearestValue1 = j;
        } else if (minDifference < minDifference2) {
            // If the current difference is equal to the minimum difference,
            // store this row as well (multiple rows with the same difference).
            minDifference2 = minDifference;
            nearestValue2 = columns[j];
            j_nearestValue2 = j;        }
    }

    selectedTipColumns[0] = nearestValue1;
    selectedTipColumns[1] = nearestValue2;
    return selectedTiltColumns;
}

int BilinearInterpolation(LookUpTableRowElements *rows[], LookUpTableRowElements *columns[], double inputTipAngle, double inputTiltAngle, double z11, double z12, double z21, double z22, double x1, double x2, double y1, double y2) 
{
    double x1 = selectedTiltRows[0];
    double x2 = selectedTiltRows[1];
    double y1 = selectedTipColumns[0];
    double y2 = selectedTipColumns[1];
    z11 = rows[i_nearestValue1]->columns[j_nearestValue1].value;
    z12 = rows[i_nearestValue1]->columns[j_nearestValue2].value;
    z21 = rows[i_nearestValue2]->columns[j_nearestValue1].value;
    z22 = rows[i_nearestValue2]->columns[j_nearestValue2].value;


    double x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;
    y2y1 = y2 - y1;
    x2x = x2 - configData->inputTipAngle;
    y2y = y2 - configData->inputTiltAngle;
    yy1 = configData->inputTiltAngle - y1;
    xx1 = configData->inputTipAngle - x1;
    return (1.0 / (x2x1 * y2y1)) * (
        z11 * x2x * y2y +
        z21 * xx1 * y2y +
        z12 * x2x * yy1 +
        z22 * xx1 * yy1
    );
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
        
        // Output the desired motor 1 angle
        motor1AngleOut.theta = motor1Angle;
        HingedRigidBodyMsg_C_write(&motor1AngleOut, &configData->motor1AngleOutMsg, moduleID, callTime);
        printf("\n motor1 Out Msg %.2f", motor1AngleOut.theta);

        // Output the desired motor 2 angle
        motor2AngleOut.theta = motor2Angle;
        HingedRigidBodyMsg_C_write(&motor2AngleOut, &configData->motor2AngleOutMsg, moduleID, callTime);
        printf("\n motor2 Out Msg %.2f", motor2AngleOut.theta);
    }
}

int readData1(const char *filename_1, LookUpTableRowElements *rows[], LookUpTableRowElements *columns[]) {
    FILE *csvFile;
    printf("%s\n", filename_1);
    char cwd[2000];
    GetCurrentDir( cwd, FILENAME1_MAX );
    printf("Current working dir: %s\n", cwd);
    
    csvFile = fopen("p_M1.csv", "r");
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
int readData2(const char *filename_2, LookUpTableRowElements *rows[], LookUpTableRowElements *columns[]) {
    FILE *csvFile;
    printf("%s\n", filename_2);
    char cwd[2000];
    GetCurrentDir( cwd, FILENAME2_MAX );
    printf("Current working dir: %s\n", cwd);
    
    csvFile = fopen("p_M.csv", "r");
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

