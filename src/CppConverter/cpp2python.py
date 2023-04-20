# import argparse
import re

"""
待处理问题：
1. exit(-1)
2. if () {} else {}
3. ** = new **()

"""


def removeTypeAssignment(line):
    types = ['int', 'float', 'bool']
    for type in types:
        line = line.replace(f"{type} ", "")
    line = line.replace('true', 'True')
    line = line.replace('false', 'False')
    return line

def handleComments(line):
    line = line.replace('//','#')
    return line

def handleCout(line):
    if 'cout' in line:
        coms = line.split('cout')
        prefix = coms[0]
        contents = coms[1].split('<<')
        endl_cnt = 0
        str_all = ""
        for content in contents:
            if 'endl' in content:
                str_all += '\\n'
            elif '"' in content:
                str_all += content.split('"')[1]
            else:
                str = content.strip()
                if str != '':
                    str_all += f"{{{str}}}"
        line = f"{prefix}print(f'{str_all}',end='')\n"
        print(f"line_raw = {line}")
    return line

def handleJudge(line):
    # 还要考虑 key 在一个单词中的情况
    keys = ['if', 'else if', 'while']
    for key in keys:
        if key in line:
            coms = line.split(key)
            prefix = coms[0]
            
            content = coms[1].strip().split('(')[0].split(')')[0].split(':')[0]
            new_line = f"{prefix}{key} {content}:\n"
            print(line)
            print("check = ", coms[1].strip().split('('))
            print(new_line)
    return line

if __name__=="__main__":
    file_path = f"/home/lj/Documents/personal_SLAM_python/src/orb_slam3/ORB_SLAM3/Setting1.py"
    fr = open(file_path, 'r')
    lines = fr.readlines()
    fr.close()
    for i in range(len(lines)):
        line = lines[i]
        line = line.replace(';','')
        line = line.replace('||','or')
        line = line.replace('&&','and')
        line = line.replace('::','.')
        line = handleComments(line)
        line = handleCout(line)
        line = handleJudge(line)
        line = removeTypeAssignment(line)
        lines[i] = line
    
    fw = open(file_path, 'w')
    for line in lines:
        fw.write(line)
    fw.close()
