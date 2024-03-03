"""This scripts takes in a CSV file with the following format: each row has a name and a hex value. Eg:
BASE_CMD_VEL, 0x10
BASE_CURRENT_VEL, 0x11
...
It then generates a C++ header file with the following format:
#pragma once
namespace can_ids {
    int BASE_CMD_VEL = 0x10;
    int BASE_CURRENT_VEL = 0x11;
    ...
}
And a Python file with the following format:
class CanIds:
    BASE_CMD_VEL = 0x10
    BASE_CURRENT_VEL = 0x11
    ...
"""

import csv
import os
import sys


def main():
    # if len(sys.argv) != 2:
    #     print("Usage: python gen_ids.py <input_csv>")
    #     sys.exit(1)
    #
    # input_csv = sys.argv[1]
    # output_cpp = os.path.splitext(input_csv)[0] + ".hpp"
    # output_py = os.path.splitext(input_csv)[0] + ".py"

    input_csv = "ids.csv"

    # Create folder "out"
    if not os.path.exists("out"):
        os.makedirs("out")

    output_cpp = "out/can_ids.hpp"
    output_py = "out/can_ids.py"

    with open(input_csv, newline='') as csvfile:
        reader = csv.reader(csvfile)
        rows = list(reader)

    with open(output_cpp, 'w') as f:
        f.write("#pragma once\n")
        f.write("namespace can_ids {\n")
        for row in rows:
            f.write(f"    int {row[0]} = {row[1]};\n")
        f.write("}\n")

    with open(output_py, 'w') as f:
        f.write("class CanIds:\n")
        for row in rows:
            f.write(f"    {row[0]} = {row[1]}\n")

    print(f"Generated {output_cpp} and {output_py}")

if __name__ == "__main__":
    main()