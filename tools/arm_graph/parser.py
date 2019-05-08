#!/usr/bin/env python3
from graphviz import Digraph
import sys
import argparse


def parse(file):
    result = []
    with open(file) as textFile:
        data = [data.split('\n') for data in textFile]
    for i in range(len(data[:])):
        for j in range(len(data[i][:])):
            if data[i][j].find('.connect') != -1:
                result.append(data[i][j])
            elif data[i][j].find('pathfinding::connect_bidirectional') != -1:
                result.append(data[i][j])

    return result

def export_fig(result,file):
    output = file + '.gv'
    dot = Digraph(comment='Graph export', filename=output)

    for i in range(len(result)):
        result[i] = result[i].replace(' ','')
        result[i] = result[i].replace('].connect(nodes[', '->')
        result[i] = result[i].replace('nodes[','')
        result[i] = result[i].replace(']);','')
        result[i] = result[i].replace('],','<->')
        result[i] = result[i].replace('pathfinding::connect_bidirectional(','')

    for i in range(len(result)):
        if result[i].find('<->') != -1:
            result[i] = result[i].split('<->')
            dot.edge(result[i][0],result[i][1],dir="both")
        elif result[i].find('->') != -1:
            result[i] = result[i].split('->')
            dot.edge(result[i][0],result[i][1])
        else:
            print('/!\\ Got unexpected result:',result[i])
    
    dot.view()

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", help="<path/to/file>")
    return parser.parse_args()

def main():
    args = parse_args()    
    result = parse(args.file)
    export_fig(result,args.file)

if __name__ == "__main__":
   main()