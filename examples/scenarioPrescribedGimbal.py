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
# Gimbal Prescribed Motion Scenario
#
# Purpose:  Simulate the prescribed gimbal motion using two single-axis stepper motor actuators.
# Author:   Leah Kiner
# Creation Date:  July 13, 2023
#

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def run():
    path_to_file = "/Users/leahkiner/Repositories/MAX_Basilisk/examples/gimbal_data.csv"
    gimbal_data = pd.read_csv(path_to_file)

    print(gimbal_data.head())
    print(gimbal_data.shape)

if __name__ == "__main__":
    run()
