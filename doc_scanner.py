#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re

content = open('include/okapi/control/pidController.hpp').read()
result = re.search(re.compile(r'/\*\*.+?\*/', re.DOTALL), content)
comments = []

while True:
    match = re.search(re.compile(r'/\*\*.+?\*/', re.DOTALL), content)
    
    if match is None: # No match found
        break
    
    try:
        comment = match.group(0)
    except IndexError: # No match found
        break
    
    comments.append(comment)
    
    # Cut out the comment
    content = content[content.index(comment) + len(comment):]
    
    print(content[:content.index(';')])

print("\n\n".join(comments))
