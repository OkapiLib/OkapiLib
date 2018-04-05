#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re

class Comment:
    description = ""
    parameters = []
    returnval = ""

    def __init__(self, comment):
        self.description = ""
        self.parameters = []
        self.returnval = ""
        
        self.description = comment
        
    def printComment(self):
        print(self.description)

content = open('include/okapi/control/iterative/posPidController.hpp').read()
result = re.search(re.compile(r'/\*\*.+?\*/', re.DOTALL), content)
comments = []
declarations = []

while True:
    match = re.search(re.compile(r'/\*\*.+?\*/', re.DOTALL), content)
    
    if match is None: # No match found
        break
    
    try:
        comment = match.group(0)
    except IndexError: # No match found
        break
    
    comments.append(comment)
    print(Comment(comment).printComment())
    
    # Cut out the comment
    content = content[content.index(comment) + len(comment):]
    
    declarations.append(content[:content.index(';')])

print("\n\n".join([x + "\n" + y for (x,y) in zip(comments, declarations)]))