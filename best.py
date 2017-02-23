#!/usr/bin/env python3
import sys
import os
from os import path

'''
Expects 'max_path.exe' and 'OpenTK.dll' in same directory.
'''

NUM_WORLDS = 30
MAX_LINE_LENGTH = 131072

if __name__ == '__main__':
    print(sys.argv[1:])
    files = list(map(lambda x: open(x, 'r'), sys.argv[1:]))
    output = open('best.out', 'w')

    global_best = [None] * (NUM_WORLDS + 1)
    scorer_exe = path.join(path.dirname(path.abspath(__file__)), 'max_path.exe')

    print("Read {} files".format(len(files)))

    for file_id, f in enumerate(files):
        lines = list(filter(lambda x: ':' in x, f.readlines()))

        for line_id, line in enumerate(lines):
            exec_cmd = 'mono \"{}\" \"{}\"'.format(scorer_exe, line)
            world_id = int(line.split(':')[0])

            if len(line) < MAX_LINE_LENGTH:
                score = float(os.popen(exec_cmd).read().replace('\n', ''))
                entry = (score, world_id, line)

                if global_best[world_id] is None:
                    global_best[world_id] = entry
                else:
                    global_best[world_id] = min([global_best[world_id], entry], key=lambda x: x[0])
            else:
                print("World {} in file {} doesn't fit!".format(world_id, sys.argv[1+file_id]))

    output.write("lindworm\nf2k19t9k1o1o7i34q7o06jbph6\n")

    global_best = global_best[1:]

    # for w_id in range(1, len(global_best)):
    #     if global_best[w_id] is not None:
    #         output.write(global_best[w_id][2])
    # #

    for world in global_best:
        if world is not None:
            output.write(world[2])

    score_list = list(map(lambda x: x[0], global_best))
    print(score_list)