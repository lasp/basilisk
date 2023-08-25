#
#  ISC License

#  Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.

#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
# EMA Inertia Calculation Script
#
# Purpose:  Given the hub inertia about point B in B frame components and the solar array inertias about their CM in
#           their respective body frames, this script calculates (1) the hub inertia about the hub CM in B frame
#           components and (2) the total spacecraft inertia about the system CM in B frame components. The user can
#           specify if they want to include the solar arrays to the spacecraft model or not.
#
# Author:   Leah Kiner
# Creation Date:    April 24, 2023
#

import os
import numpy as np
np.set_printoptions(precision=16)

def run():
    # Define masses
    massSC = 2215.09816   # [kg]
    massSolarArray = 82.79  # [kg]

    # Spacecraft inertia about point B expressed in B frame components
    Pxx = 4995.119267   # [kg m^2]
    Pyy = 8794.038297   # [kg m^2]
    Pzz = 4809.587466   # [kg m^2]
    Pxy = 4.924921893   # [kg m^2]
    Pxz = 33.08314604   # [kg m^2]
    Pyz = -69.65723097  # [kg m^2]
    ISpacecraftB_B = [[Pxx,  -Pxy,  -Pxz],
                      [-Pxy,  Pyy,  -Pyz],
                      [-Pxz,  -Pyz,  Pzz]]

    # Position vector from hub frame origin to hub CM in B frame components
    r_BcB_B = [7.84713085978795 / 1000,  -9.966952366 / 1000,  1214.854881 / 1000]   # [m]
    r_BcB_BTilde = [[0, -r_BcB_B[2], r_BcB_B[1]],
                    [r_BcB_B[2], 0, -r_BcB_B[0]],
                    [-r_BcB_B[1], r_BcB_B[0], 0]]

    # DCMs representing the attitude of the B frame with respect to the solar array body frames
    dcm_BS1 = [[0,  1,  0],
               [0,  0, -1],
               [-1,  0,  0]]
    dcm_BS2 = [[0, -1,  0],
               [0,  0, -1],
               [1,  0,  0]]

    # Solar array inertia about CM expressed in S frame components
    IArraySc_S = [[319.0, 0.0,  0.0],
                  [0.0,  185.0, 0.0],
                  [0.0,  0.0, 495.0]]   # [kg m^2]

    # Solar array CM location relative to S frame origin expressed in S frame components
    r_ScS_S = [-0.036,  2.827,  -0.469]   # [m]
    r_ScS_STilde = [[0, -r_ScS_S[2], r_ScS_S[1]],
                    [r_ScS_S[2], 0, -r_ScS_S[0]],
                    [-r_ScS_S[1], r_ScS_S[0], 0]]

    # Solar array inertia about point S expressed in S frame components
    IArrayS_S = np.add(IArraySc_S, massSolarArray * np.matmul(r_ScS_STilde, np.transpose(r_ScS_STilde)))   # [kg m^2]

    # Solar array inertia about their CM in B frame
    IArray1Sc_B = np.matmul(dcm_BS1, np.matmul(IArraySc_S, np.transpose(dcm_BS1)))
    IArray2Sc_B = np.matmul(dcm_BS2, np.matmul(IArraySc_S, np.transpose(dcm_BS2)))

    # Solar array CM location relative to their respective body frame origin expressed in B frame components
    rArray1ScS_B = np.matmul(dcm_BS1, np.transpose(r_ScS_S))
    rArray2ScS_B = np.matmul(dcm_BS2, np.transpose(r_ScS_S))

    # Solar array body frame origins with respect to point B expressed in B frame components
    rArray1SB_B = [0.5 * 1.53, 0.0, 0.44]   # [m]
    rArray2SB_B = [-0.5 * 1.53, 0.0, 0.44]   # [m]

    # Solar array CM location relative to point B expressed in B frame components
    rArray1_ScB_B = np.add(rArray1ScS_B, rArray1SB_B)
    rArray2_ScB_B = np.add(rArray2ScS_B, rArray2SB_B)
    rArray1_ScB_BTilde = [[0, -rArray1_ScB_B[2], rArray1_ScB_B[1]],
                          [rArray1_ScB_B[2], 0, -rArray1_ScB_B[0]],
                          [-rArray1_ScB_B[1], rArray1_ScB_B[0], 0]]
    rArray2_ScB_BTilde = [[0, -rArray2_ScB_B[2], rArray2_ScB_B[1]],
                          [rArray2_ScB_B[2], 0, -rArray2_ScB_B[0]],
                          [-rArray2_ScB_B[1], rArray2_ScB_B[0], 0]]

    # Solar array inertias about point B expressed in B frame components
    IArray1B_B = np.add(IArray1Sc_B, massSolarArray * np.matmul(rArray1_ScB_BTilde, np.transpose(rArray1_ScB_BTilde)))
    IArray2B_B = np.add(IArray2Sc_B, massSolarArray * np.matmul(rArray2_ScB_BTilde, np.transpose(rArray2_ScB_BTilde)))

    # Case 1: Hub With Solar Arrays
    print("\n** ** HUB AND SOLAR ARRAYS ** **")

    # Re-define hub mass by removing solar array mass
    massHub = massSC - 2 * massSolarArray  # [kg]

    # Hub inertia about point B expressed in B frame components
    IHubB_B = np.subtract(ISpacecraftB_B, np.add(IArray1B_B, IArray2B_B))

    # Hub inertia about hub CM expressed in B frame components
    IHubBc_B = np.subtract(IHubB_B, massHub * np.matmul(r_BcB_BTilde, np.transpose(r_BcB_BTilde)))

    # Spacecraft inertia about system CM in B frame components
    rSpacecraft_CMB_B = np.add(np.add(np.multiply(massHub / massSC, r_BcB_B), np.multiply(massSolarArray / massSC, rArray1_ScB_B)), np.multiply(massSolarArray / massSC, rArray2_ScB_B))
    rSpacecraft_CMB_BTilde = [[0, -rSpacecraft_CMB_B[2], rSpacecraft_CMB_B[1]],
                              [rSpacecraft_CMB_B[2], 0, -rSpacecraft_CMB_B[0]],
                              [-rSpacecraft_CMB_B[1], rSpacecraft_CMB_B[0], 0]]
    ISpacecraftCM_B = np.subtract(ISpacecraftB_B, massSC * np.matmul(rSpacecraft_CMB_BTilde, np.transpose(rSpacecraft_CMB_BTilde)))

    # Print results
    print("\nIHubBc_B:")
    print(IHubBc_B)
    print("\nISpacecraftCM_B:")
    print(ISpacecraftCM_B)

    # CASE 2: Hub With No Solar Arrays
    print("\n** ** HUB WITH NO SOLAR ARRAYS ** **")
    massHub = massSC  # [kg]

    # Hub about point B expressed in B frame components
    IHubB_B = ISpacecraftB_B

    # # Hub inertia about hub CM expressed in B frame components
    IHubBc_B = np.subtract(IHubB_B, massHub * np.matmul(rSpacecraft_CMB_BTilde, np.transpose(rSpacecraft_CMB_BTilde)))

    # Spacecraft inertia about system CM in B frame components
    ISpacecraftCM_B = np.subtract(ISpacecraftB_B, massSC * np.matmul(rSpacecraft_CMB_BTilde, np.transpose(rSpacecraft_CMB_BTilde)))

    # Print results
    print("\nIHubBc_B:")
    print(IHubBc_B)
    print("\nISpacecraftCM_B:")
    print(ISpacecraftCM_B)

if __name__ == "__main__":
    run()





















