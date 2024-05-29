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
 @param this The configuration data associated with this module
 @param moduleID The module identifier
 */

void SepGimbalLookUpTable::SelfInit() {
    // Initialize the output messages
    HingedRigidBodyMsg_C_init(&this->motor1AngleOutMsg);
    HingedRigidBodyMsg_C_init(&this->motor2AngleOutMsg);
    const char *fileName_1 = "C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\\sepGimbalLookUpTable\\P_M1.csv";
    const char *fileName_2 = "C:\\Users\\ShamsaS\\Documents\\software-projects\\basilisk-lasp\\src\\fswAlgorithms\\effectorInterfaces\\sepGimbalLookUpTable\\P_M2.csv";

    // Allocate memory for rows
    for (int i = 0; i < MAX_ROWS; i++) {
        this->rows[i] = malloc(sizeof(LookUpTableRowElements));
        if (this->rows[i] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error of rows
        }
    }

    // Allocate memory for stored rows
    for (int i = 0; i < MAX_ROWS; i++) {
        this->selectedTipRows[i] = malloc(sizeof(LookUpTableRowElements));
        if (this->selectedTipRows[i] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error of columns
        }
    }

    int result_row1 = readData1(fileName_1, this->rows);
    int result_row2 = readData2(fileName_2, this->rows);

    // Allocate memory for columns
    for (int j = 0; j < MAX_COLUMNS; j++) {
        this->columns[j] = malloc(sizeof(LookUpTableRowElements));
        if (this->columns[j] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error gracefully
        }
    }

    // Allocate memory for stored columns
    for (int j = 0; j < MAX_COLUMNS; j++) {
        this->selectedTipColumnss[j] = malloc(sizeof(LookUpTableRowElements));
        if (this->selectedTipColumns[j] == NULL) {
            printf("Memory allocation error.\n");
            exit(1); // Handle the error gracefully
        }
    }

    int result_column1 = readData1(fileName_1, this->columns);
    int result_column2 = readData2(fileName_2, this->columns);

}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param this The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/

void SepGimbalLookUpTable::Reset(uint64_t callTime) {
    // Check if the required input message is linked
    if (!HingedRigidBodyMsg_C_isLinked(&this->desiredGimbalTipAngleInMsg)
        && !HingedRigidBodyMsg_C_isLinked(&this->desiredGimbalTiltAngleInMsg)) {
        this->bskLogger->bskLog(BSK_ERROR, "Error: sepGimbalLookUpTable.desiredGimbalTipAngleInMsg and "
                                           "sepGimbalLookUpTable.desiredGimbalTiltAngleInMsg wasn't connected.");
    }

    // Set the motor1 angle to zero                              
    this->motor1Angle = 0.0;

    // Set the motor2 angle to zero  
    this->motor2Angle = 0.0;
}

// Function to find the nearest 2 rows with equal or nearest tilt angle value
LookUpTableRowElements SepGimbalLookUpTable::findMatchingTiltAngle(double inputTiltAngle) {
    int numMatchesRows = 0;
    double minDifference = DBL_MAX; // Initialize the minimum difference 
    double minDifference2 = DBL_MAX; // Initialize the minimum difference 
    double nearestValue1 = 0.0;
    double nearestValue2 = 0.0;

    // Iterate through all rows to find the closest tilt angles values
    for (auto & row : this->rows) {
        double difference = fabs(inputTiltAngle - row->desiredTiltAngle); // Calculate the absolute difference

        if (difference < minDifference) {
            // If the current difference is less than the minimum difference1, update the minimum difference and select this row.
            numMatchesRows = 0; // Reset selected rows
            minDifference = difference;
            nearestValue1 = row;
        } else if (minDifference < minDifference2) {
            // If the current difference is equal to the minimum difference,
            // store this row as well (multiple rows with the same difference).
            minDifference2 = minDifference;
            nearestValue2 = row;
        }
    }
    selectedTiltRows[0] = nearestValue1;
    selectedTiltRows[1] = nearestValue2;
    return selectedTiltRows;
}

// Function to find the nearest 2 columns tip angle value
LookUpTableRowElements SepGimbalLookUpTable::findMatchingTipAngle(double inputTipAngle) {
    int numMatchesColumns = 0;
    double minDifference = DBL_MAX; // Initialize the minimum difference 
    double minDifference2 = DBL_MAX;

    // Iterate through all columns to find the closest tip angles values
    for (auto & column : this->columns) {
        double difference = fabs(inputTipAngle - column->desiredTipAngle); // Calculate the absolute difference

        if (difference < minDifference) {
            // If the current difference is less than the minimum difference1, update the minimum difference and select this row.
            numMatchesColumns = 0; // Reset selected rows
            minDifference = difference;
            this->selectedTipColumns[0] = column;
        } else if (minDifference < minDifference2) {
            // If the current difference is equal to the minimum difference,
            // store this row as well (multiple rows with the same difference).
            minDifference2 = minDifference;
            this->selectedTipColumns[1] = column;
        }
    }
    return selectedTiltColumns;
}

Eigen::Vector2d SepGimbalLookUpTable::bilinearInterpolation(double tipAngle,
                                                            double tiltAngle,
                                                            double z11,
                                                            double z12,
                                                            double z21,
                                                            double z22,
                                                            double x1,
                                                            double x2,
                                                            double y1,
                                                            double y2) {
//    double x2x1 = x2 - x1;
//    double y2y1 = y2 - y1;
//    double x2x = x2 - tipAngle;
//    double y2y = y2 - tiltAngle;
//    double yy1 = tiltAngle - y1;
//    double xx1 = tipAngle - x1;
//    return (1.0 / (x2x1 * y2y1)) * (
//            z11 * x2x * y2y +
//            z21 * xx1 * y2y +
//            z12 * x2x * yy1 +
//            z22 * xx1 * yy1
//    );
    return Eigen::Vector2d::Identity();
}

/*! This method recieves the desired tip and tilt angles and outputs the motor angles.
The desired gimbal angles are then written to the output message to find the corresponding motor angles from the lookup table.
 @return void
 @param callTime [ns] The current time of simulation
*/
void SepGimbalLookUpTable::UpdateState(uint64_t callTime) {
    // Read the gimbal tip angle input message
    HingedRigidBodyMsgPayload desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&this->desiredGimbalTipAngleInMsg)) {
        desiredGimbalTipAngleIn = HingedRigidBodyMsg_C_read(&this->desiredGimbalTipAngleInMsg);
        printf("desiredGimbalTipAngleIn.theta %f\n", desiredGimbalTipAngleIn.theta);
    }

    // Read the gimbal tilt angle input message
    HingedRigidBodyMsgPayload desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&this->desiredGimbalTiltAngleInMsg)) {
        desiredGimbalTiltAngleIn = HingedRigidBodyMsg_C_read(&this->desiredGimbalTiltAngleInMsg);
        printf("desiredGimbalTiltAngleIn.theta %f\n", desiredGimbalTiltAngleIn.theta);
    }

    // Print inputTipAngle and inputTiltAngle
    printf("Input Tip Angle: %f\n", desiredGimbalTipAngleIn.theta);
    printf("\nInput Tilt Angle: %f\n", desiredGimbalTiltAngleIn.theta);

    // Call findMatchingTipAngle to get selected rows based on inputTipAngle
    LookUpTableRowElements *matchingRowsTip[MAX_ROWS];
    int numMatchingRowsTip = findMatchingTipAngle(desiredGimbalTipAngleIn.theta);
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
    matchingRowsTip = findMatchingTiltAngle(inputTiltAngle);
    printf("\nNumber of matching rows based on Tilt Angle within the matching tip angle subset: %d\n\n",
           numMatchingRowsTilt);

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
    HingedRigidBodyMsgPayload motor1AngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();
    HingedRigidBodyMsgPayload motor2AngleOut = HingedRigidBodyMsg_C_zeroMsgPayload();
    for (int i = 0; i < numMatchingRowsTilt; i++) {
        motor1Angle = matchingRowsTilt[i]->motor1Angle;
        motor2Angle = matchingRowsTilt[i]->motor2Angle;
        printf("\nmotor1Angle=%.2f\nmotor2Angle=%.2f\n", motor1Angle, motor2Angle);

        // Output the desired motor 1 angle
        motor1AngleOut.theta = motor1Angle;
        HingedRigidBodyMsg_C_write(&motor1AngleOut, &this->motor1AngleOutMsg, moduleID, callTime);
        printf("\n motor1 Out Msg %.2f", motor1AngleOut.theta);

        // Output the desired motor 2 angle
        motor2AngleOut.theta = motor2Angle;
        HingedRigidBodyMsg_C_write(&motor2AngleOut, &this->motor2AngleOutMsg, moduleID, callTime);
        printf("\n motor2 Out Msg %.2f", motor2AngleOut.theta);
    }
}

int SepGimbalLookUpTable::readData1(const char *filename_1,
                                    LookUpTableRowElements *rows[],
                                    LookUpTableRowElements *columns[]) {
    FILE *csvFile;
    printf("%s\n", filename_1);
    char cwd[2000];
    GetCurrentDir(cwd, FILENAME1_MAX);
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

    do {
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

        if (numColumnsMatched != 5 && feof(csvFile)) {
            printf("error file format.\n");
            return 1;
        }

        if (ferror(csvFile)) {
            printf("error file reading.\n");
            return 1;
        }

    } while (!feof(csvFile));

    fclose(csvFile);
    printf("\n%d records read.\n\n", record);

    for (int i = 0; i < record; i++) {
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

int SepGimbalLookUpTable::readData2(const char *filename_2,
                                    LookUpTableRowElements *rows[],
                                    LookUpTableRowElements *columns[]) {
    FILE *csvFile;
    printf("%s\n", filename_2);
    char cwd[2000];
    GetCurrentDir(cwd, FILENAME2_MAX);
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

    do {
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

        if (numColumnsMatched != 5 && feof(csvFile)) {
            printf("error file format.\n");
            return 1;
        }

        if (ferror(csvFile)) {
            printf("error file reading.\n");
            return 1;
        }

    } while (!feof(csvFile));

    fclose(csvFile);
    printf("\n%d records read.\n\n", record);

    for (int i = 0; i < record; i++) {
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