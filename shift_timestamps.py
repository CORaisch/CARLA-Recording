#!/usr/bin/env python

import argparse

argparser = argparse.ArgumentParser(description="shift timestamps s.t. they start at 0.0 sec")
argparser.add_argument('--timestamps', '-stamps', type=str, default=None, help="path to timestamps.txt file")
args = argparser.parse_args()

if args.timestamps == None:
    print("ERROR: expect path to timestamps.txt file as argument")
else:
    output = ""
    f = open(args.timestamps, 'r'); lines = f.readlines(); f.close();
    ref_line = float(lines[0])
    for line in lines:
        output += str(float(line)-ref_line) + "\n"

    with open(args.timestamps.replace('.txt', '_shifted.txt'), 'w') as output_file:
        output_file.write(output)
